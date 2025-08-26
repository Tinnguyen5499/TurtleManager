import os
import sys
import subprocess
from pathlib import Path

def bash(cmd: str):
    # Run inside a login shell so 'source' works
    return subprocess.check_call(["bash", "-lc", cmd])

def main():
    # In editable installs, this points to your repo root
    repo = Path(__file__).resolve().parents[1]

    run_script = repo / "scripts" / "run_turtlemanager.sh"
    install_script = repo / "scripts" / "install.sh"
    ws_setup = repo / "turtle_ws" / "install" / "setup.bash"

    if not run_script.exists():
        print("[ERROR] Could not find scripts/run_turtlemanager.sh.")
        print("This entrypoint expects an *editable* install (pip install -e .).")
        sys.exit(1)

    # First-run: build the embedded ROS 2 workspace once
    if not ws_setup.exists():
        if not install_script.exists():
            print("[ERROR] scripts/install.sh not found; cannot build workspace.")
            sys.exit(1)
        print("==> TurtleManager: building embedded ROS 2 workspace (first run)…")
        bash(f"cd '{repo}' && chmod +x scripts/install.sh && scripts/install.sh")

    # Launch the GUI (script sources ROS Foxy + workspace for the process)
    bash(f"cd '{repo}' && chmod +x scripts/run_turtlemanager.sh && scripts/run_turtlemanager.sh")

if __name__ == "__main__":
    main()
