from setuptools import setup

package_name = "person_follower"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/follow.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Shyam Sreenivasan",
    maintainer_email="sreenivasan.sh@northeastern.edu",
    description="Person detection, tracking and following for the ACEBOTT car",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_viewer = person_follower.camera_viewer_node:main",
            "person_detector = person_follower.person_detector_node:main",
            "person_tracker = person_follower.person_tracker_node:main",
            "pan_tracker = person_follower.pan_tracker_node:main",
            "calibrate_room = person_follower.calibrate_room_node:main",
            "follow_controller = person_follower.follow_controller_node:main",
            "acebott_bridge = person_follower.acebott_bridge_node:main",
        ],
    },
)
