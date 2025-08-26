# TurtleManager 🐢

A lightweight GUI to run multi‑TurtleBot experiments with **OptiTrack** on **ROS 2 Foxy** (plus ROS 1 and rosbridge for TurtleBot2).  
It vendors a small ROS 2 workspace (`turtle_ws/`) and a mapper that turns OptiTrack rigid‑body poses into per‑robot topics `/<robot_ns>/pose` — **no manual ROS sourcing required** (the app handles it).

---

## ✨ Highlights

- One‑click start/stop of **OptiTrack client + mapper** from the GUI (managed in `tmux`)
- Two mapping modes from OptiTrack → robots:
  - **Name mode** — name Motive rigid bodies exactly `robot_2`, `robot_3`, …
  - **First‑seen mode (default)** — first body → `robot_2`, next → `robot_3`, …  
    *(We intentionally skip `robot_1`.)*
- Publishes `geometry_msgs/Pose` on `/<robot_ns>/pose` for each robot
- Includes Python package **`process_mocap`** (`ros2 run process_mocap mapper_node`) and a patched OptiTrack client that adds a **`name`** field to each rigid body message
- **ROS1 + ROS2 + ros1_bridge** workflow built in: control **TurtleBot2 (ROS1)** from a modern **ROS2** environment
- GUI can start and monitor up to **6 TurtleBots**, automatically **SSH** to each bot, launch per‑bot `roscore`, and pass the main PC’s IP

---

## 🧩 Repository Layout

```
TurtleManager/
├─ TurtleManager.py                # GUI launcher (handles sourcing & tmux)
├─ GUI_interface.py
├─ GUI_setting.py
├─ turtle_ws/
│  └─ src/
│     ├─ ros2-mocap_optitrack/     # OptiTrack client (with name support)
│     └─ process_mocap/            # mapper node (mapper_node)
├─ scripts/                        # optional helper scripts (if present)
├─ requirement.txt                 # Python deps (PyQt6, psutil, numpy, rowan)
├─ LICENSE                         # MIT (or your chosen license)
└─ README.md
```

> **Note:** the file is named **`requirement.txt`** (singular) in this repo.

---

## 🚀 Quick Start — Docker (recommended)

This repo ships with a **Dockerfile** that has ROS1, ROS2 Foxy, and rosbridge pre‑installed and is the most reliable way to run the GUI with minimal host setup.

### One‑block setup (copy‑paste on your host)

```bash
# 0) Allow local X11 (so the container can open the GUI window)
xhost +local:root

# 1) Grab the code
git clone git@github.com:<YOUR_USER>/TurtleManager.git  # or use https URL
cd TurtleManager

# 2) Build the image (has ROS1+ROS2+rosbridge and needed libs)
docker build -t turtlemanager .

# 3) Run the container with display + host networking
docker run --rm -it \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$(pwd)":/workspace/TurtleManager \
  --workdir /workspace/TurtleManager \
  --name turtlemanager \
  turtlemanager /bin/bash -lc "
    # Install Python deps inside the container (no venv needed)
    python3 -m pip install --upgrade pip setuptools wheel sip && \
    python3 -m pip install --break-system-packages --ignore-installed -r requirement.txt && \
    # Run the GUI
    python3 TurtleManager.py
  "
```

If you see the GUI window, you’re good to go. Toggling **OptiTrack ON** will start both the **OptiTrack client** and the **mapper** in a `tmux` session named `optitrack_session`.  
You can view logs at any time with:

```bash
tmux attach -t optitrack_session
```

> If you prefer to keep the container running in the background, remove `--rm` and add `-d`.

---

## 🐧 Native Install (advanced / optional)

If you don’t want Docker, you can run natively on **Ubuntu 20.04** with **ROS 2 Foxy** pre‑installed at `/opt/ros/foxy` (and ROS1 if you need rosbridge). Install dependencies:

```bash
sudo apt-get update
# GUI & build tools
sudo apt-get install -y tmux gnome-terminal python3-pip \
  libxcb-cursor0 libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev
# ROS tooling
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init || true
rosdep update

# Python deps (no virtualenv needed)
python3 -m pip install --upgrade pip setuptools wheel sip
python3 -m pip install --break-system-packages --ignore-installed -r requirement.txt
```

Run the app:

```bash
cd ~/TurtleManager
python3 TurtleManager.py
```

> The GUI **auto‑sources** what it needs and will build the embedded ROS2 workspace (`turtle_ws/`) if missing — you don’t need to source any setup files by hand.

---

## 🎯 Mapping Rules

Two exclusive modes (select in GUI, default **first_seen**):

1) **Name mode**  
   - In Motive, rename your rigid bodies to **`robot_2`**, **`robot_3`**, …  
   - The mapper forwards poses to matching namespaces.

2) **First‑seen mode (default)**  
   - The first rigid body the client sees is assigned **`robot_2`**, next **`robot_3`**, …  
   - Helpful when people forget to rename assets; still skips `robot_1`.

**Published topics**: for each assigned robot, `/<robot_ns>/pose` (`geometry_msgs/Pose`).  
**Raw OptiTrack topic**: `/mocap_rigid_bodies` (`mocap_optitrack_interfaces/RigidBodyArray`) with fields: `id`, `valid`, `mean_error`, `pose_stamped`, and `name`.

---

## 🧪 Verifying Data Flow

In a new terminal (with the GUI running OptiTrack):

```bash
# See the raw stream with rigid-body names coming from Motive
ros2 topic echo /mocap_rigid_bodies --qos-reliability best_effort

# See per-robot pose (example for robot_2)
ros2 topic echo /robot_2/pose
```

You should see entries like:

```
rigid_bodies:
- id: 105
  ...
  name: robot_3
```

---

## 🔌 TurtleBot2 + ROS1/ROS2 + rosbridge

- The GUI can **SSH** into up to **6 TurtleBots**, start their **ROS1 roscore**, and connect them to the **main PC’s ROS master**.
- The host container runs **ros1_bridge** so your ROS1 bots can be controlled from ROS2 nodes.
- The GUI detects the host IP and passes it to bots for correct networking.

> Make sure your TurtleBots can SSH to/from the main machine and that firewalls allow ROS/bridge ports.

---

## 🧯 Troubleshooting

- **Qt “xcb” plugin error** (on native installs):  
  Install the required libs:
  ```bash
  sudo apt-get install -y libxcb-cursor0 libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev
  ```
  (Dockerfile already includes these.)

- **`/mocap_rigid_bodies` silent or missing**  
  Ensure Motive is streaming (Multicast or Unicast) to your machine and ports **1510/1511** are open.

- **Deserialization errors when echoing**  
  Mismatch between message types and running nodes. Rebuild interfaces:
  ```bash
  cd ~/TurtleManager/turtle_ws
  source /opt/ros/foxy/setup.bash
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install --packages-select \
    mocap_optitrack_interfaces mocap_optitrack_client process_mocap
  ```

- **No `/robot_* /pose` updates**  
  - In **first_seen** mode, ensure at least one rigid body is visible.
  - In **name** mode, ensure the Motive name matches exactly (e.g., `robot_2`).

- **Terminal not opening**  
  If `gnome-terminal` isn’t installed, GUI will still run; use `tmux attach -t optitrack_session` to view logs.

---

## 🧩 What We Changed in the OptiTrack Client

- Added `string name` to `mocap_optitrack_interfaces/msg/RigidBody.msg`.
- Built an **ID→name** map from NatNet data descriptions in the C++ client.
- Populated `RigidBody.name` when publishing `RigidBodyArray`.
- The `process_mocap` mapper reads this `name` to support **Name mode**.

These changes are vendored inside `turtle_ws/src/ros2-mocap_optitrack`, so users don’t need to patch upstream.

---

## 📦 Developer Notes

- GUI starts a tmux session `optitrack_session` with two panes (client + mapper).
- If you edit ROS packages under `turtle_ws/src/`, rebuild:
  ```bash
  cd ~/TurtleManager/turtle_ws
  source /opt/ros/foxy/setup.bash
  colcon build --symlink-install
  ```
- Keep `build/`, `install/`, and `log/` out of version control (see `.gitignore`).

---

## 📄 License

See `LICENSE` (MIT). OptiTrack SDK and ROS dependencies retain their own licenses.

---

## 🙌 Acknowledgements

- OptiTrack NatNet SDK  
- ROS 1 / ROS 2 community  
- All upstream packages vendored or referenced here