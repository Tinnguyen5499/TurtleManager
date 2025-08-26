import subprocess
from pathlib import Path
import sys

def bash(cmd: str) -> int:
    # run in a login shell so "source" works
    return subprocess.check_call(["bash", "-lc", cmd])

def main():
    repo = Path(__file__).resolve().parents[1]
    scripts = repo / "scripts"
    run_script = scripts / "run_turtlemanager.sh"
    install_script = scripts / "install.sh"
    ws_setup = repo / "turtle_ws" / "install" / "setup.bash"

    if not run_script.exists():
        print("[ERROR] scripts/run_turtlemanager.sh not found.")
        sys.exit(1)

    # first run: build the embedded ROS 2 workspace once
    if not ws_setup.exists():
        if not install_script.exists():
            print("[ERROR] scripts/install.sh not found; cannot build workspace.")
            sys.exit(1)
        print("==> TurtleManager: building embedded ROS 2 workspace (first run)…")
        bash(f"cd '{repo}' && chmod +x scripts/install.sh && scripts/install.sh")

    # launch GUI (script sources Foxy + the workspace)
    bash(f"cd '{repo}' && chmod +x scripts/run_turtlemanager.sh && scripts/run_turtlemanager.sh")

if __name__ == "__main__":
    main()
