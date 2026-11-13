#!/usr/bin/env python3
"""Project menu. Pick an option and it runs the matching script."""

from pathlib import Path
import subprocess
import sys


# --------------------------------------------------
# PROJECT PATHS
# --------------------------------------------------

PROJECT_DIR = Path(__file__).resolve().parent
CALIBRATION_DIR = PROJECT_DIR / "calibration"


# --------------------------------------------------
# GENERAL HELPERS
# --------------------------------------------------

def run_script(script_name, arguments=()):
    """Run another project script. Returns True if it succeeded."""

    script_path = PROJECT_DIR / script_name

    if not script_path.exists():
        print(f"\nError: {script_name} not found.")
        return False

    command = [sys.executable, str(script_path), *[str(a) for a in arguments]]

    try:
        result = subprocess.run(command, cwd=PROJECT_DIR)
    except KeyboardInterrupt:
        print("\nOperation interrupted.")
        return True

    return result.returncode == 0


def pause():
    input("\nPress Enter to return to the main menu...")


# --------------------------------------------------
# 1. CHECK CAMERA STREAM
# --------------------------------------------------

def check_camera_stream():
    print("\nOpening camera stream... (press Q in the video window to close)")

    if not run_script("setup/check_conduit_camera.py"):
        print("\nProblem with the camera stream. Check if the topics are publishing")

    pause()


# --------------------------------------------------
# 2. CALIBRATE WORKSPACE
# --------------------------------------------------

def calibrate_workspace():
    print("\nCalibrating workspace from the live camera...")

    if not run_script("setup/calibrate.py"):
        print("\nCalibration was not completed.")

    pause()


# --------------------------------------------------
# 3. LIVE LOCALIZATION
# --------------------------------------------------

def list_calibrations():
    """Names of calibrations that have both a .npy and a .json file."""
    if not CALIBRATION_DIR.exists():
        return []

    names = [
        path.stem for path in CALIBRATION_DIR.glob("*.npy")
        if path.with_suffix(".json").exists()
    ]

    # workspace_2 before workspace_10
    def sort_key(name):
        prefix, _, number = name.rpartition("_")
        return (prefix, int(number)) if number.isdigit() else (name, 0)

    return sorted(names, key=sort_key)


def live_localization():
    calibrations = list_calibrations()

    if not calibrations:
        print("\nNo calibrations found. Run \"Calibrate workspace\" first.")
        pause()
        return

    print("\nAvailable calibrations:")
    for name in calibrations:
        print(f"  {name}")

    name = input("\nEnter the calibration to use (or press Enter to cancel): ").strip()

    if not name:
        return

    print("\nStarting live localization... (press Q in the video window to close)")

    if not run_script("robot-live-detection.py", ["--calibration", name]):
        print("\nLive localization stopped with an error.")

    pause()


# --------------------------------------------------
# MAIN MENU
# --------------------------------------------------

MENU = [
    ("Check camera stream", check_camera_stream),
    ("Calibrate workspace", calibrate_workspace),
    ("Live localization (Manual Control)", live_localization),
]


def main():
    while True:
        print("\n" + "=" * 45)
        print("          ROBOT LOCALIZATION")
        print("=" * 45)
        print("\nSelect an operation:\n")

        for number, (label, _) in enumerate(MENU, start=1):
            print(f"{number}. {label}")
        print("0. Exit")

        choice = input("\nEnter your choice: ").strip()

        if choice == "0":
            print("\nExiting.")
            break

        if choice.isdigit() and 1 <= int(choice) <= len(MENU):
            MENU[int(choice) - 1][1]()
        else:
            print("\nInvalid choice. Try again.")


if __name__ == "__main__":
    main()
