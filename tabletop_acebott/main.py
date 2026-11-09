
#!/usr/bin/env python3

from pathlib import Path
import subprocess
import sys
import platform


# --------------------------------------------------
# PROJECT PATHS
# --------------------------------------------------

PROJECT_DIR = Path(__file__).resolve().parent

VIDEO_DIR = PROJECT_DIR / "data" / "videos"
IMAGE_DIR = PROJECT_DIR / "data" / "images"
CALIBRATION_DIR = PROJECT_DIR / "calibration"
RESULTS_DIR = PROJECT_DIR / "results"


# --------------------------------------------------
# GENERAL HELPERS
# --------------------------------------------------

def get_files(folder, extensions):
    """Return files in a folder matching the given extensions."""

    folder.mkdir(parents=True, exist_ok=True)

    return sorted([
        f for f in folder.iterdir()
        if f.is_file() and f.suffix.lower() in extensions
    ])


def select_file(files, description):
    """Display files and let the user select one."""

    if not files:
        print(f"\nNo {description} found.")
        return None

    print(f"\nAvailable {description}:")

    for i, file in enumerate(files, start=1):
        print(f"{i}. {file.name}")

    while True:
        choice = input("\nSelect a number (or 0 to cancel): ").strip()

        if choice == "0":
            return None

        if choice.isdigit() and 1 <= int(choice) <= len(files):
            return files[int(choice) - 1]

        print("Invalid selection. Try again.")


def run_script(script_name, arguments):
    """Execute another project script."""

    script_path = PROJECT_DIR / script_name

    if not script_path.exists():
        print(f"\nError: {script_name} not found.")
        return

    command = [
        sys.executable,
        str(script_path),
        *[str(arg) for arg in arguments]
    ]

    print("\nStarting operation...")
    print("-" * 50)

    try:
        subprocess.run(command, cwd=PROJECT_DIR, check=True)
    except subprocess.CalledProcessError as error:
        print(f"\nOperation failed (exit code {error.returncode}).")
    except KeyboardInterrupt:
        print("\nOperation interrupted.")


def choose_calibration():
    """Select an existing calibration."""

    files = get_files(CALIBRATION_DIR, {".npy"})

    calibration = select_file(files, "calibrations")

    return calibration


def pause():
    input("\nPress Enter to return to the main menu...")


# --------------------------------------------------
# 1. EXTRACT IMAGE FROM VIDEO
# --------------------------------------------------

def extract_image():

    videos = get_files(VIDEO_DIR, {".mp4", ".avi", ".mov", ".mkv"})

    video = select_file(videos, "videos")

    if video is None:
        return

    default_output = f"{video.stem}_frame.png"

    output = input(
        f"Output image name [{default_output}]: "
    ).strip()

    if not output:
        output = default_output

    if not Path(output).suffix:
        output += ".png"

    run_script(
        "convert.py",
        [video.name, "--output", output]
    )

    pause()


# --------------------------------------------------
# 2. CALIBRATE WORKSPACE
# --------------------------------------------------

def calibrate_workspace():

    images = get_files(
        IMAGE_DIR,
        {".png", ".jpg", ".jpeg", ".bmp"}
    )

    image = select_file(images, "images")

    if image is None:
        return

    name = input(
        "\nEnter a calibration name (e.g. video1_tag_plane): "
    ).strip()

    if not name:
        print("Calibration name cannot be empty.")
        pause()
        return

    height = input(
        "Calibration plane height in cm [0]: "
    ).strip()

    if not height:
        height = "0"

    try:
        height = float(height)
    except ValueError:
        print("Invalid height.")
        pause()
        return

    arguments = [
        image.name,
        "--name", name,
        "--plane-height-cm", str(height)
    ]

    run_script("calibrate.py", arguments)

    pause()


# --------------------------------------------------
# 3. ESTIMATE ROBOT POSE
# --------------------------------------------------

def estimate_robot_pose():

    images = get_files(
        IMAGE_DIR,
        {".png", ".jpg", ".jpeg", ".bmp"}
    )

    image = select_file(images, "images")

    if image is None:
        return

    calibration = choose_calibration()

    if calibration is None:
        return

    arguments = [
        "pose",
        image.name,
        "--tag-homography",
        calibration
    ]

    run_script("robot_localize.py", arguments)

    pause()


# --------------------------------------------------
# 4. TRACK ROBOT IN VIDEO
# --------------------------------------------------

def track_robot():

    videos = get_files(VIDEO_DIR, {".mp4", ".avi", ".mov", ".mkv"})

    video = select_file(videos, "videos")

    if video is None:
        return

    calibration = choose_calibration()

    if calibration is None:
        return

    output_folder = RESULTS_DIR / f"{video.stem}_tracking"

    arguments = [
        "track",
        video.name,
        "--tag-homography",
        calibration,
        "--display",
        "--show-map",
        "--save-video",
        "--output-dir",
        output_folder
    ]

    run_script("robot_localize.py", arguments)

    pause()


# --------------------------------------------------
# 6. LIVE ROBOT LOCALIZATION
# --------------------------------------------------

def localize_robot_live():
    """Run live localization using a selected saved calibration."""

    calibration = choose_calibration()

    if calibration is None:
        return

    print(f"\nSelected transformation: {calibration.name}")

    arguments = [
        "--tag-homography",
        calibration
    ]

    run_script("robot_localize_live.py", arguments)

    pause()


# --------------------------------------------------
# 5. VIEW PREVIOUS RESULTS
# --------------------------------------------------

def view_results():

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    items = sorted(
        [p for p in RESULTS_DIR.iterdir()],
        key=lambda p: p.stat().st_mtime,
        reverse=True
    )

    if not items:
        print("\nNo results available yet.")
        pause()
        return

    print("\nPrevious results:")

    for i, item in enumerate(items, start=1):
        kind = "Folder" if item.is_dir() else "File"
        print(f"{i}. {item.name} ({kind})")

    print("0. Return to main menu")

    choice = input("\nSelect a result to open: ").strip()

    if choice == "0":
        return

    if choice.isdigit() and 1 <= int(choice) <= len(items):

        selected = items[int(choice) - 1]

        try:
            system = platform.system()

            if system == "Linux":
                subprocess.Popen(["xdg-open", str(selected)])
            elif system == "Windows":
                import os
                os.startfile(selected)
            elif system == "Darwin":
                subprocess.Popen(["open", str(selected)])

        except Exception as error:
            print(f"Could not open result: {error}")

    else:
        print("Invalid selection.")

    pause()


# --------------------------------------------------
# MAIN MENU
# --------------------------------------------------

def main():

    while True:

        print("\n")
        print("=" * 45)
        print("          ROBOT LOCALIZATION")
        print("=" * 45)

        print("\nSelect an operation:\n")

        print("1. Extract image from video")
        print("2. Calibrate workspace")
        print("3. Estimate robot pose")
        print("4. Track robot in video")
        print("5. View previous results")
        print("6. Live robot localization")
        print("0. Exit")

        choice = input("\nEnter your choice: ").strip()

        if choice == "1":
            extract_image()

        elif choice == "2":
            calibrate_workspace()

        elif choice == "3":
            estimate_robot_pose()

        elif choice == "4":
            track_robot()

        elif choice == "5":
            view_results()

        elif choice == "6":
            localize_robot_live()

        elif choice == "0":
            print("\nExiting Robot Localization.")
            break

        else:
            print("\nInvalid choice. Try again.")


if __name__ == "__main__":
    main()
