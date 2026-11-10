#!/usr/bin/env python3
"""Single ROS 2 pipeline: camera -> ArUco pose -> click goal -> RRT* -> smooth feedback.

Requires robot_localize.py from the existing project and the same two calibration
files used by robot_navigation_preview.py. Dry-run by default.

Live motor output uses the same TCP protocol as keyboard-wifi-control.py:
ESP32_IP:1234, one ASCII command per line: "v,w\n" (normalized [-1,1]).

Obstacle JSON (optional): {"rectangles_cm": [[xmin,ymin,xmax,ymax], ...]}
Obstacles are manually configured; this script does NOT detect obstacles.
"""
import argparse
import csv
import json
import math
import random
import socket
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

import robot_localize as loc

WIN = "Acebott navigation | click to start | S stop | Q quit"


def load_homography(path, frame):
    path = Path(path).expanduser()
    metadata = json.loads(path.with_suffix('.json').read_text())
    if metadata.get('coordinate_frame_version') != loc.FRAME_VERSION:
        raise ValueError(f"Coordinate frame mismatch: {path}")
    cal_w, cal_h = int(metadata['image_width_px']), int(metadata['image_height_px'])
    h, w = frame.shape[:2]
    if abs(cal_w / cal_h - w / h) > 0.01:
        raise ValueError(f"Camera orientation/aspect mismatch: {cal_w}x{cal_h} vs {w}x{h}")
    scale = np.diag([cal_w / w, cal_h / h, 1.0])
    return np.load(path) @ scale, metadata


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def collision_free(a, b, rects, limits, resolution=1.0):
    n = max(1, math.ceil(distance(a, b)/resolution))
    xmin, ymin, xmax, ymax = limits
    for i in range(n+1):
        t = i/n
        x, y = a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t
        if not (xmin <= x <= xmax and ymin <= y <= ymax):
            return False
        if any(rx0 <= x <= rx1 and ry0 <= y <= ry1 for rx0,ry0,rx1,ry1 in rects):
            return False
    return True


def rrt(start, goal, limits, rects, *, iterations=600, step=8.0,
        seed=0, neighbor_radius=18.0, smooth=True, direct_shortcut=False):
    """RRT* in cm: best-parent selection, local rewiring, best-goal extraction.

    Uses a fixed connection radius on this small bounded tabletop. It is an
    RRT* variant; a straight segment remains the shortest path in free space.
    """
    if not collision_free(start, start, rects, limits):
        raise ValueError(f'Start {tuple(round(z, 1) for z in start)} outside safe area {limits} or inside inflated obstacle')
    if not collision_free(goal, goal, rects, limits):
        raise ValueError(f'Goal {tuple(round(z, 1) for z in goal)} outside safe area {limits} or inside inflated obstacle')
    if distance(start, goal) < 1e-6:
        return [start, goal]
    if direct_shortcut and collision_free(start, goal, rects, limits):
        return [start, goal]

    rng = random.Random(seed)
    nodes = [start]
    parents = [-1]

    def cost(k):
        total = 0.0
        while parents[k] != -1:
            p = parents[k]
            total += distance(nodes[k], nodes[p])
            k = p
        return total

    def is_ancestor(candidate, child):
        # Is candidate an ancestor of child? Avoid rewiring cycles.
        while child != -1:
            if child == candidate:
                return True
            child = parents[child]
        return False

    for _ in range(iterations):
        target = goal if rng.random() < 0.12 else (
            rng.uniform(limits[0], limits[2]),
            rng.uniform(limits[1], limits[3]))
        nearest = min(range(len(nodes)), key=lambda i: distance(nodes[i], target)**2)
        a = nodes[nearest]
        d = distance(a, target)
        if d < 1e-6:
            continue
        new = (a[0] + (target[0]-a[0])*min(1.0, step/d),
               a[1] + (target[1]-a[1])*min(1.0, step/d))
        if not collision_free(a, new, rects, limits):
            continue

        near = [i for i in range(len(nodes)) if distance(nodes[i], new) <= neighbor_radius]
        best_parent = nearest
        best_cost = cost(nearest) + distance(a, new)
        for i in near:
            trial_cost = cost(i) + distance(nodes[i], new)
            if trial_cost + 1e-9 < best_cost and collision_free(nodes[i], new, rects, limits):
                best_parent, best_cost = i, trial_cost

        new_index = len(nodes)
        nodes.append(new)
        parents.append(best_parent)

        for i in near:
            if i == best_parent or is_ancestor(i, best_parent):
                continue
            candidate_cost = best_cost + distance(new, nodes[i])
            if candidate_cost + 1e-9 < cost(i) and collision_free(new, nodes[i], rects, limits):
                parents[i] = new_index

    # Select the least-cost collision-free connection to the goal.
    candidates = [i for i, node in enumerate(nodes)
                  if collision_free(node, goal, rects, limits)]
    if not candidates:
        return None
    end = min(candidates, key=lambda i: cost(i) + distance(nodes[i], goal))
    path = [goal]
    k = end
    while k != -1:
        path.append(nodes[k])
        k = parents[k]
    path.reverse()
    if not smooth:
        return path
    # Optional line-of-sight smoothing can collapse a valid route to a straight line.
    smoothed = [path[0]]
    i = 0
    while i < len(path)-1:
        j = len(path)-1
        while j > i+1 and not collision_free(path[i], path[j], rects, limits):
            j -= 1
        smoothed.append(path[j])
        i = j
    return smoothed


def follow_target(path, pose, previous_progress_cm, lookahead_cm):
    """Project onto polyline, advance monotonically, target lookahead arc-length."""
    lengths = [distance(a,b) for a,b in zip(path[:-1],path[1:])]
    accum = [0.0]
    for d in lengths:
        accum.append(accum[-1]+d)
    x,y = pose[:2]
    progress = previous_progress_cm
    best_d2 = float('inf')
    for i,(a,b) in enumerate(zip(path[:-1],path[1:])):
        dx,dy = b[0]-a[0],b[1]-a[1]
        length2 = dx*dx+dy*dy
        t = max(0.0,min(1.0,((x-a[0])*dx+(y-a[1])*dy)/length2)) if length2 > 1e-8 else 0.0
        px,py = a[0]+t*dx,a[1]+t*dy
        s = accum[i] + t*lengths[i]
        d2 = (x-px)**2+(y-py)**2
        if d2 < best_d2 and s >= previous_progress_cm - lookahead_cm:
            progress = max(previous_progress_cm, s)
            best_d2 = d2
    target_s = min(accum[-1],progress+lookahead_cm)
    for i,(a,b) in enumerate(zip(path[:-1],path[1:])):
        if target_s <= accum[i+1] or i == len(path)-2:
            t = (target_s-accum[i])/max(1e-9,lengths[i])
            return (a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])), progress
    return path[-1],progress


class Navigator(Node):
    def __init__(self,args):
        super().__init__('acebott_rrt_navigation')
        self.a = args
        self.tag_H = self.goal_H = None
        self.tag_meta = self.goal_meta = None
        self.detector = loc.create_detector('4X4_50')
        self.pose = None
        self.pose_received_at = None
        self.detected = False
        self.state = 'IDLE'
        self.goal = None
        self.path = []
        self.progress = 0.0
        self.phase = 'TRACKING'
        self.last_tick_at = None
        self.pulse_deadline = None
        self.settle_deadline = None
        self.observation_after = None
        self.last_command = (0.0, 0.0)
        self.session_start = None
        self.session_id = 0
        self.logfile = None
        self.logwriter = None
        self.obstacles = []
        if args.obstacles:
            data = json.loads(Path(args.obstacles).read_text())
            for r in data.get('rectangles_cm',[]):
                if len(r)!=4 or r[2]<r[0] or r[3]<r[1]:
                    raise ValueError(f'Invalid rectangle: {r}')
                self.obstacles.append(tuple(float(v) for v in r))
        self.motor_socket = None
        self.last_command_print = 0.0
        if args.enable_commands:
            # Fail closed: refuse to run live control without the ESP32 connection.
            try:
                self.motor_socket = socket.create_connection(
                    (args.esp_ip, args.esp_port), timeout=args.socket_timeout
                )
                self.motor_socket.settimeout(args.socket_timeout)
                self.get_logger().info(
                    f'ESP32 TCP connected: {args.esp_ip}:{args.esp_port}'
                )
                self.send(0.0, 0.0)
            except OSError as exc:
                if self.motor_socket is not None:
                    self.motor_socket.close()
                    self.motor_socket = None
                raise RuntimeError(
                    f'Cannot connect to Acebott ESP32 at '
                    f'{args.esp_ip}:{args.esp_port}: {exc}'
                ) from exc
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(CompressedImage,args.topic,self.on_image,qos)
        self.timer = self.create_timer(1.0/args.control_hz,self.tick)
        self.get_logger().info('Preview live. Click to plan/start navigation. S=stop, C=clear, Q=quit.')
        self.get_logger().warning('LIVE MOTOR OUTPUT ENABLED: continuous, low-power TCP commands' if args.enable_commands else 'DRY RUN: no motor output')

    def margins(self):
        w,h = float(self.goal_meta['width_cm']),float(self.goal_meta['height_cm'])
        # RRT* plans the circle CENTER; keep its footprint inside the table.
        m=self.a.robot_radius_cm + self.a.clearance_cm
        return (m,m,w-m,h-m)

    def inflated_obstacles(self):
        r=self.a.robot_radius_cm + self.a.clearance_cm
        return [(x0-r,y0-r,x1+r,y1+r) for x0,y0,x1,y1 in self.obstacles]

    def fresh(self):
        return self.pose is not None and self.pose_received_at is not None and time.monotonic()-self.pose_received_at < self.a.pose_timeout

    def on_click(self,event,x,y,flags,userdata):
        if event!=cv2.EVENT_LBUTTONDOWN or self.goal_H is None:
            return
        if self.state=='NAVIGATING':
            self.get_logger().warning('Already navigating; press S before selecting another goal')
            return
        if not self.fresh():
            self.get_logger().warning('No fresh robot pose; cannot start navigation')
            return
        goal = tuple(map(float,loc.transform_points([(x,y)],self.goal_H,self.goal_meta)[0]))
        start = self.pose[:2]
        self.state='PLANNING'
        # Planning runs synchronously; this is fine for a small tabletop RRT,
        # but move it off the ROS callback for heavier planners.
        self.session_id += 1
        self.get_logger().info(f'Session {self.session_id}: plan {start} -> {goal}')
        try:
            path = rrt(start, goal, self.margins(), self.inflated_obstacles(),
                       iterations=self.a.rrt_iterations,
                       step=self.a.rrt_step_cm,
                       seed=self.session_id,
                       neighbor_radius=self.a.rrt_neighbor_radius_cm,
                       smooth=not self.a.no_path_smoothing,
                       direct_shortcut=self.a.direct_path_shortcut)
        except ValueError as exc:
            self.get_logger().warning(f'Planning rejected: {exc}')
            path = None
        if path is None:
            self.get_logger().warning(
                f'No valid path. Safe center bounds={self.margins()}, '
                f'obstacles={len(self.obstacles)}, '
                f'iterations={self.a.rrt_iterations}. '
                'Check boundary margins, robot footprint, and start/goal positions.'
            )
            self.state='IDLE'
            self.goal=None
            return
        self.goal = goal
        self.path = path
        self.progress = 0.0
        self.phase = 'TRACKING'
        self.last_command = (0.0, 0.0)
        self.last_tick_at = None
        self.pulse_deadline = None
        self.settle_deadline = None
        self.observation_after = None
        self.session_start=time.monotonic()
        self.open_log()
        self.state='NAVIGATING'
        self.get_logger().info(f'Session {self.session_id} NAVIGATING: {len(path)} path points, final goal {goal}')

    def open_log(self):
        Path(self.a.log_dir).mkdir(parents=True,exist_ok=True)
        stamp=time.strftime('%Y%m%d_%H%M%S')
        filename=Path(self.a.log_dir)/f'nav_{stamp}_{self.session_id}.csv'
        self.logfile=filename.open('w',newline='')
        self.logwriter=csv.writer(self.logfile)
        self.logwriter.writerow(['pc_monotonic_s','session','state','pose_x_cm','pose_y_cm','heading_deg','goal_x_cm','goal_y_cm','target_x_cm','target_y_cm','goal_error_cm','linear_mps','angular_rps','detection_fresh','phase','tcp_v','tcp_w','heading_error_deg','pose_age_s'])
        self.logfile.flush()
        self.get_logger().info(f'Session log: {filename}')

    def send(self, v=0.0, w=0.0, normalized=False):
        """Send keyboard-equivalent v,w to ESP32 using the working TCP format.

        Controller v is m/s and w is rad/s. The keyboard controller's
        turning sign is reversed relative to ROS/world-positive CCW:
        keyboard 'a' (left) -> w=-1; 'd' (right) -> w=+1.
        Normalization gains MUST be calibrated against measured robot movement.
        """
        # Values are normalized commands, NOT physical velocities on the ESP32.
        if normalized:
            v_cmd = max(-self.a.max_motor_command, min(self.a.max_motor_command, v))
            w_cmd = max(-self.a.max_motor_command, min(self.a.max_motor_command, w))
        else:
            v_cmd = max(-self.a.max_motor_command, min(
                self.a.max_motor_command, v * self.a.linear_command_gain))
            w_cmd = max(-self.a.max_motor_command, min(
                self.a.max_motor_command, -w * self.a.angular_command_gain))
        self.last_command = (v_cmd, w_cmd)
        if not self.a.enable_commands:
            if self.state == 'NAVIGATING' and time.monotonic() - self.last_command_print >= 1.0:
                self.get_logger().info(
                    f'DRY RUN: v={v:.3f}m/s w={w:.3f}rad/s '
                    f'-> TCP({v_cmd:+.3f},{w_cmd:+.3f})'
                )
                self.last_command_print = time.monotonic()
            return True
        if self.motor_socket is None:
            return False
        try:
            self.motor_socket.sendall(f'{v_cmd:.4f},{w_cmd:.4f}\n'.encode('ascii'))
            if self.state == 'NAVIGATING' and time.monotonic() - self.last_command_print >= 1.0:
                self.get_logger().info(
                    f'ESP32 TX v={v_cmd:+.3f} w={w_cmd:+.3f} '
                    f'(controller: {v:.3f}m/s, {w:.3f}rad/s)'
                )
                self.last_command_print = time.monotonic()
            return True
        except OSError as exc:
            self.get_logger().error(f'ESP32 TCP send failed: {exc}')
            try:
                self.motor_socket.close()
            finally:
                self.motor_socket = None
            return False

    def close_session(self,reason):
        self.send(0.0,0.0)
        if self.state in ('NAVIGATING','PLANNING'):
            self.get_logger().info(f'Session {self.session_id}: {reason}')
        self.state=reason
        self.goal=None
        self.path=[]
        self.progress=0.0
        self.phase='TRACKING'
        self.last_tick_at=None
        self.pulse_deadline=None
        self.settle_deadline=None
        self.observation_after=None
        self.session_start=None
        if self.logfile:
            self.logfile.flush()
            self.logfile.close()
            self.logfile=None
            self.logwriter=None

    @staticmethod
    def approach(value, target, maximum_change):
        """Slew-limit a normalized motor command to avoid sharp changes."""
        return value + max(-maximum_change, min(maximum_change, target - value))

    def tick(self):
        """Continuous feedback: latest pose -> bounded low-power TCP (v, w).

        Sign convention from the WORKING keyboard controller:
        W=(+v, 0), S=(-v, 0), A=(0, -w), D=(0, +w).
        A positive world-frame heading error therefore needs negative TCP w.
        """
        if self.state != 'NAVIGATING':
            self.send(0.0, 0.0, normalized=True)
            self.last_tick_at = None
            return

        now = time.monotonic()
        if now - self.session_start > self.a.session_timeout:
            self.close_session('ABORTED_TIMEOUT')
            return
        if not self.fresh():
            self.close_session('ABORTED_TRACKING_LOST')
            return

        x, y, theta = self.pose
        if not collision_free((x,y), (x,y), self.inflated_obstacles(), self.margins()):
            self.close_session('ABORTED_CLEARANCE')
            return
        goal_error = distance((x,y), self.goal)
        if goal_error <= self.a.goal_tolerance_cm:
            self.close_session('GOAL_REACHED')
            return

        target, self.progress = follow_target(
            self.path, self.pose, self.progress, self.a.lookahead_cm)
        heading_error = wrap(math.atan2(target[1]-y, target[0]-x) - math.radians(theta))
        heading_error_deg = math.degrees(heading_error)

        # Alignment mode only for large heading errors; otherwise blend turn
        # into forward motion for smooth arcs. Angular sign matches A/D.
        if abs(heading_error_deg) > self.a.align_threshold_deg:
            v_target = 0.0
            w_target = -math.copysign(
                min(self.a.turn_power, max(self.a.min_turn_power,
                    self.a.heading_kp * abs(heading_error))), heading_error)
            self.phase = 'ALIGN'
        else:
            # Constant small forward command, reduced close to the goal.
            # No physical-velocity claim: these are normalized ESP32 inputs.
            near_scale = min(1.0, max(0.0,
                (goal_error - self.a.goal_tolerance_cm) / self.a.slowdown_distance_cm))
            v_target = self.a.drive_power * near_scale
            if near_scale > 0.0:
                v_target = max(self.a.min_drive_power, v_target)
            v_target *= max(0.0, math.cos(heading_error))
            w_target = -max(-self.a.steer_power, min(
                self.a.steer_power, self.a.heading_kp * heading_error))
            self.phase = 'DRIVE'

        # Avoid commanding more combined differential effort than we tested.
        effort = abs(v_target) + abs(w_target)
        if effort > self.a.max_motor_command:
            factor = self.a.max_motor_command / effort
            v_target *= factor
            w_target *= factor

        # Ramp toward command, rather than abrupt step changes.
        dt = (1.0 / self.a.control_hz if self.last_tick_at is None
              else min(0.5, max(0.0, now - self.last_tick_at)))
        self.last_tick_at = now
        step = self.a.command_slew_per_s * dt
        old_v, old_w = self.last_command
        v = self.approach(old_v, v_target, step)
        w = self.approach(old_w, w_target, step)
        if not self.send(v, w, normalized=True):
            self.close_session('ABORTED_MOTOR_CONNECTION')
            return

        if self.logwriter:
            self.logwriter.writerow([
                f'{now:.6f}', self.session_id, self.state,
                f'{x:.3f}',f'{y:.3f}',f'{theta:.3f}',
                f'{self.goal[0]:.3f}',f'{self.goal[1]:.3f}',
                f'{target[0]:.3f}',f'{target[1]:.3f}',
                f'{goal_error:.3f}', '', '', int(self.fresh()),
                self.phase, f'{v:.4f}',f'{w:.4f}',
                f'{heading_error_deg:.2f}',
                f'{(now-self.pose_received_at):.3f}',
            ])
            self.logfile.flush()

    def on_image(self,msg):
        frame=cv2.imdecode(np.frombuffer(msg.data,dtype=np.uint8),cv2.IMREAD_COLOR)
        if frame is None:
            return
        frame=cv2.rotate(frame,cv2.ROTATE_90_COUNTERCLOCKWISE)
        if self.tag_H is None:
            try:
                self.tag_H,self.tag_meta=load_homography(self.a.tag_homography,frame)
                self.goal_H,self.goal_meta=load_homography(self.a.goal_homography,frame)
                for k in ('width_cm','height_cm','bev_width_px','bev_height_px'):
                    if float(self.tag_meta[k])!=float(self.goal_meta[k]):
                        raise ValueError(f'Calibration mismatch: {k}')
                cv2.namedWindow(WIN,cv2.WINDOW_AUTOSIZE)
                cv2.setMouseCallback(WIN,self.on_click)
            except Exception as e:
                self.get_logger().error(f'Calibration error: {e}')
                self.close_session('ABORTED_CALIBRATION')
                rclpy.shutdown()
                return
        overlay=frame.copy()
        corners=loc.detect_marker(frame,self.detector,self.a.marker_id)
        self.detected=corners is not None
        if corners is not None:
            tag_pose=loc.estimate_pose(corners,self.tag_H,self.tag_meta,self.a.heading_offset_deg)
            rx,ry=loc.robot_center_from_tag(tag_pose,self.a.tag_forward_offset_cm)
            self.pose=(float(rx),float(ry),float(tag_pose[2]))
            self.pose_received_at=time.monotonic()
            loc.annotate_marker(overlay,corners,tag_pose,self.a.marker_id,(rx,ry))
            cv2.putText(overlay,f'POSE x={rx:.1f} y={ry:.1f} heading={tag_pose[2]:.1f}',(10,58),cv2.FONT_HERSHEY_SIMPLEX,.53,(0,255,255),2)
        else:
            cv2.putText(overlay,'MARKER NOT DETECTED',(10,58),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,255),2)
        cv2.putText(overlay,'Click: navigate | S: stop | C: clear | Q: quit',(10,25),cv2.FONT_HERSHEY_SIMPLEX,.52,(255,255,255),2)
        cv2.putText(overlay,f'{self.state}/{self.phase} | {"LIVE COMMANDS" if self.a.enable_commands else "DRY RUN"}',(10,83),cv2.FONT_HERSHEY_SIMPLEX,.53,(0,255,255),2)
        if self.goal is not None:
            # Project ground-plane waypoints into the raw camera image for overlay.
            # loc.transform_points maps pixels -> cm, so use inverse map manually.
            w=float(self.goal_meta['width_cm']);h=float(self.goal_meta['height_cm'])
            bw=float(self.goal_meta['bev_width_px']);bh=float(self.goal_meta['bev_height_px'])
            points=np.array([[(px/w*(bw-1),(1-py/h)*(bh-1))] for px,py in self.path],dtype=np.float32)
            try:
                pix=cv2.perspectiveTransform(points,np.linalg.inv(self.goal_H)).reshape(-1,2)
                pix=[tuple(map(int,p)) for p in pix]
                for a,b in zip(pix[:-1],pix[1:]):
                    cv2.line(overlay,a,b,(255,0,255),2)
                cv2.drawMarker(overlay,pix[-1],(255,0,255),cv2.MARKER_CROSS,18,2)
            except (cv2.error,np.linalg.LinAlgError):
                pass
            if self.pose is not None:
                cv2.putText(overlay,f'Goal distance: {distance(self.pose,self.goal):.1f} cm',(10,108),cv2.FONT_HERSHEY_SIMPLEX,.53,(255,0,255),2)
        cv2.imshow(WIN,overlay)
        if not self.a.no_map:
            mapimg=np.zeros((520,650,3),np.uint8)
            scale=min(540/float(self.goal_meta['width_cm']),440/float(self.goal_meta['height_cm']))
            def pixel(p): return (round(50+p[0]*scale),round(470-p[1]*scale))
            lo,bo,hi,top=0,0,float(self.goal_meta['width_cm']),float(self.goal_meta['height_cm'])
            cv2.rectangle(mapimg,pixel((lo,top)),pixel((hi,bo)),(100,180,100),2)
            for x0,y0,x1,y1 in self.obstacles:
                cv2.rectangle(mapimg,pixel((x0,y1)),pixel((x1,y0)),(0,0,255),-1)
            for a,b in zip(self.path[:-1],self.path[1:]):
                cv2.line(mapimg,pixel(a),pixel(b),(255,0,255),2)
            if self.goal is not None: cv2.drawMarker(mapimg,pixel(self.goal),(255,0,255),cv2.MARKER_CROSS,16,2)
            if self.pose is not None:
                cv2.circle(mapimg,pixel(self.pose),5,(0,255,255),-1)
                th=math.radians(self.pose[2])
                cv2.arrowedLine(mapimg,pixel(self.pose),pixel((self.pose[0]+8*math.cos(th),self.pose[1]+8*math.sin(th))),(0,255,255),2)
            cv2.putText(mapimg,self.state,(10,25),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
            cv2.imshow('Navigation map',mapimg)
        key=cv2.waitKey(1)&0xFF
        if key==ord('s') or key==ord('c'):
            self.close_session('CANCELLED')
        elif key==ord('q'):
            self.close_session('QUIT')
            rclpy.shutdown()

    def destroy_node(self):
        self.close_session('SHUTDOWN')
        if self.motor_socket is not None:
            self.motor_socket.close()
            self.motor_socket = None
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--tag-homography',type=Path,required=True)
    p.add_argument('--goal-homography',type=Path,required=True)
    p.add_argument('--topic',default='/conduit/camera/front/image_raw/compressed')
    p.add_argument('--esp-ip',default='10.157.163.120',help='ESP32 IP (verify after Wi-Fi/hotspot changes)')
    p.add_argument('--esp-port',type=int,default=1234)
    p.add_argument('--socket-timeout',type=float,default=5.0)
    p.add_argument('--linear-command-gain',type=float,default=2.0,help='Normalized forward command per m/s; CALIBRATE')
    p.add_argument('--angular-command-gain',type=float,default=0.3,help='Normalized turning command per rad/s; CALIBRATE')
    p.add_argument('--max-motor-command',type=float,default=1.0,help='Absolute limit of ESP32 normalized v,w in [0,1]')
    p.add_argument('--marker-id',type=int,default=0)
    p.add_argument('--tag-forward-offset-cm',type=float,default=4.5)
    p.add_argument('--heading-offset-deg',type=float,default=0.0)
    p.add_argument('--enable-commands',action='store_true',help='Send direct TCP velocity commands to ESP32; independent ESP32 watchdog REQUIRED')
    p.add_argument('--no-map',action='store_true')
    p.add_argument('--obstacles',type=Path,help='JSON with rectangles_cm: [[xmin,ymin,xmax,ymax],...]')
    p.add_argument('--clearance-cm',type=float,default=2.0,help='Additional gap beyond robot radius (cm)')
    p.add_argument('--robot-radius-cm',type=float,default=6.0,help='Circular robot footprint radius in cm; MEASURE before live runs')
    p.add_argument('--rrt-iterations',type=int,default=600)
    p.add_argument('--rrt-neighbor-radius-cm',type=float,default=18.0)
    p.add_argument('--no-path-smoothing',action='store_true',help='Show raw RRT* waypoint path; useful for inspecting planner')
    p.add_argument('--direct-path-shortcut',action='store_true',help='Skip RRT* exploration when straight line is collision free')
    p.add_argument('--rrt-step-cm',type=float,default=8.0)
    p.add_argument('--lookahead-cm',type=float,default=8.0)
    p.add_argument('--align-threshold-deg',type=float,default=32.0,
                   help='Rotate-in-place above this heading error; otherwise steer while moving')
    p.add_argument('--drive-power',type=float,default=0.38,
                   help='Normalized forward TCP command (W direction), tune for actual motor deadband')
    p.add_argument('--min-drive-power',type=float,default=0.25,
                   help='Minimum forward power while driving, must exceed actual motor deadband')
    p.add_argument('--turn-power',type=float,default=0.36,
                   help='Maximum rotate-in-place power (A/D direction)')
    p.add_argument('--min-turn-power',type=float,default=0.22,
                   help='Minimum turning power while aligning, tune for actual motor deadband')
    p.add_argument('--steer-power',type=float,default=0.18,
                   help='Maximum turning component while also driving forward')
    p.add_argument('--heading-kp',type=float,default=0.85,
                   help='Normalized angular correction per radian of heading error')
    p.add_argument('--slowdown-distance-cm',type=float,default=20.0)
    p.add_argument('--command-slew-per-s',type=float,default=1.5,
                   help='Maximum normalized command change per second')
    p.add_argument('--goal-tolerance-cm',type=float,default=4.0)
    p.add_argument('--control-hz',type=float,default=10.0)
    p.add_argument('--pose-timeout',type=float,default=0.7)
    p.add_argument('--session-timeout',type=float,default=90.0)
    p.add_argument('--log-dir',default='navigation_logs')
    a=p.parse_args()
    if a.clearance_cm<0 or a.robot_radius_cm<=0 or a.control_hz<=0 or a.goal_tolerance_cm<=0 or a.rrt_neighbor_radius_cm<=0 or a.rrt_iterations<=0:
        p.error('Safety margins, control frequency and tolerance must be positive')
    if not (0 < a.max_motor_command <= 1) or a.linear_command_gain <= 0 or a.angular_command_gain <= 0 or a.socket_timeout <= 0:
        p.error('Motor command limit must be in (0,1], gains and timeout must be positive')
    if not (0 < a.min_drive_power <= a.drive_power <= a.max_motor_command):
        p.error('Require 0 < min-drive-power <= drive-power <= max-motor-command')
    if not (0 < a.min_turn_power <= a.turn_power <= a.max_motor_command):
        p.error('Require 0 < min-turn-power <= turn-power <= max-motor-command')
    if not (0 <= a.steer_power <= a.max_motor_command):
        p.error('steer-power must fit in [0,max-motor-command]')
    if a.heading_kp <= 0 or a.slowdown_distance_cm <= 0 or a.command_slew_per_s <= 0:
        p.error('heading-kp, slowdown-distance-cm, command-slew-per-s must be positive')
    rclpy.init()
    try:
        node=Navigator(a)
    except Exception:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__':
    main()
