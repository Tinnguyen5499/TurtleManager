# TurtleManager 🐢

A lightweight GUI to run multi‑TurtleBot experiments with **OptiTrack** on **ROS 2 Foxy** — with **ROS 1 + ros1_bridge** built in so you can control **TurtleBot2 (ROS1)** from a modern ROS2 environment.

It vendors a small ROS 2 workspace (`turtle_ws/`) and a mapper that turns OptiTrack rigid‑body poses into per‑robot topics `/<robot_ns>/pose`. **No manual ROS `source` required** — the GUI does this internally.

---

## ✨ Highlights

- One‑click start/stop of **OptiTrack client + mapper** from the GUI (managed in `tmux`)
- Two mapping modes from OptiTrack → robots:
  - **Name mode** — name Motive rigid bodies exactly `robot_2`, `robot_3`, …
  - **First‑seen mode (default)** — first body → `robot_2`, next → `robot_3`, … *(we intentionally skip `robot_1`)*
- Publishes `geometry_msgs/Pose` on `/<robot_ns>/pose` for each robot
- Patched OptiTrack client adds a **`name`** field per rigid body; mapper reads it
- **ROS1 + ROS2 + ros1_bridge** workflow baked in
- Manage up to **6 TurtleBots**: auto‑SSH to each bot, launch per‑bot `roscore`, and pass the main PC’s IP to them

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
├─ scripts/                        # optional helper scripts
├─ requirement.txt                 # Python deps (PyQt6, psutil, numpy, rowan)
├─ Dockerfile                      # ROS1 + ROS2 + rosbridge base
├─ LICENSE                         # MIT (or your chosen license)
└─ README.md
```
> Note: the file is named **`requirement.txt`** (singular) in this repo.

---

## 🚀 Quick Install — **Docker (recommended)**

This is the most reliable way to run the GUI with minimal host setup. The provided **Dockerfile** includes **ROS1**, **ROS2 Foxy**, **rosbridge**, Qt X11 deps, and common tools.

### 🔹 Block 1 — run on the **host**

```bash
# Allow local X11 access for GUI apps from the container
xhost +local:root

# Get the Dockerfile and code
git clone https://github.com/Tinnguyen5499/TurtleManager
cd TurtleManager

# Build the image (includes ROS1 + ROS2 + rosbridge + Qt deps)
docker build -t turtlemanager .

# Run the container with display + host networking
docker run -it \
  --privileged \
  --env="DISPLAY=${DISPLAY}" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network=host \
  --name turtlemanager \
  turtlemanager
```

> When you’re done, you can revoke the temporary X11 permission with `xhost -local:root`.

### 🔹 Block 2 — run **inside the container** (one‑shot)

Copy and paste **this single block** after the container starts:

```bash
set -e
# 1) Grab the repo (if not already present inside the container)
[ -d /workspace ] || mkdir -p /workspace
cd /workspace
if [ ! -d TurtleManager ]; then
  git clone https://github.com/Tinnguyen5499/TurtleManager
fi
cd TurtleManager

# 2) Install Python deps (no virtualenv required in this container)
python3 -m pip install --upgrade pip setuptools wheel sip
python3 -m pip install --break-system-packages --ignore-installed -r requirement.txt

# 3) Launch the GUI
python3 TurtleManager.py
```

If the GUI appears, you’re set. Toggling **OptiTrack ON** starts both the OptiTrack client and the mapper in a `tmux` session named **`optitrack_session`**. View logs anytime:

```bash
tmux attach -t optitrack_session
```

> To re‑enter the container later: `docker start -ai turtlemanager`

---

## 🐧 Native Install (advanced / optional)

If you don’t want Docker, you can run natively on **Ubuntu 20.04** with **ROS 2 Foxy** installed at `/opt/ros/foxy` (and ROS1 if you need rosbridge).

```bash
sudo apt-get update

# GUI & Qt bits commonly needed for PyQt6 on 20.04
sudo apt-get install -y tmux gnome-terminal \
  libxcb-cursor0 libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev

# ROS tooling
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init || true
rosdep update

# Python deps (no venv)
python3 -m pip install --upgrade pip setuptools wheel sip
python3 -m pip install --break-system-packages --ignore-installed -r requirement.txt

# Run the app
python3 TurtleManager.py
```

> The GUI auto‑sources what it needs and will build the embedded ROS2 workspace (`turtle_ws/`) if missing — **you don’t need to manually `source` anything**.

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
**Raw OptiTrack topic**: `/mocap_rigid_bodies` (`mocap_optitrack_interfaces/RigidBodyArray`) with fields: `id`, `valid`, `mean_error`, `pose_stamped`, and **`name`**.

---

## 🧪 Verifying Data Flow

In a new terminal (with the GUI running OptiTrack):

```bash
# See the raw stream with rigid-body names coming from Motive
ros2 topic echo /mocap_rigid_bodies --qos-reliability best_effort

# See per-robot pose (example for robot_2)
ros2 topic echo /robot_2/pose
```

You should see entries similar to:
```
rigid_bodies:
- id: 105
  ...
  name: robot_3
```

---

## 🔌 TurtleBot2 + ROS1/ROS2 + rosbridge

- The GUI can SSH into up to **6 TurtleBots**, start their **ROS1 `roscore`**, and connect them to the main PC’s ROS master.
- The host/container runs **ros1_bridge**, so your **ROS1** bots can be controlled from **ROS2** nodes.
- The GUI detects the **host IP** and passes it to the bots for correct networking.

Make sure your TurtleBots can SSH to/from the main machine and that firewalls allow ROS/bridge ports.

---

## 🧯 Troubleshooting

- **Qt “xcb” plugin error (native installs)** → install:
  ```bash
  sudo apt-get install -y libxcb-cursor0 libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev
  ```
  *(Already handled by the Dockerfile.)*

- **`/mocap_rigid_bodies` silent or missing**  
  Ensure Motive is streaming (Multicast/Unicast) to your machine and NatNet ports **1510/1511** are reachable.

- **Deserialization errors when echoing** → likely mismatch; rebuild interfaces:
  ```bash
  cd ~/TurtleManager/turtle_ws
  source /opt/ros/foxy/setup.bash
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install --packages-select \
    mocap_optitrack_interfaces mocap_optitrack_client process_mocap
  ```

- **No `/<robot_*>/pose` updates**  
  - In **first_seen** mode, ensure at least one rigid body is visible.  
  - In **name** mode, ensure the Motive name matches exactly (e.g., `robot_2`).

- **Terminal not opening**  
  If `gnome-terminal` isn’t installed, GUI still runs; use `tmux attach -t optitrack_session` to view logs.

---

## 🧩 What We Changed in the OptiTrack Client

- Added `string name` to `mocap_optitrack_interfaces/msg/RigidBody.msg`.
- Built an **ID→name** map from NatNet data descriptions in the C++ client.
- Populated `RigidBody.name` when publishing `RigidBodyArray`.
- The `process_mocap` mapper reads this **`name`** to support **Name mode**.

These changes are vendored inside `turtle_ws/src/ros2-mocap_optitrack`, so users don’t need to patch upstream.

---

## 📦 Developer Notes

- GUI starts a `tmux` session **`optitrack_session`** with two panes (client + mapper).
- If you edit ROS packages under `turtle_ws/src/`, rebuild:
  ```bash
  cd ~/TurtleManager/turtle_ws
  source /opt/ros/foxy/setup.bash
  colcon build --symlink-install
  ```
- Keep `build/`, `install/`, and `log/` out of version control (see `.gitignore`).

---

## 📄 License

MIT — see `LICENSE` in this repo.  
OptiTrack SDK and ROS dependencies retain their own licenses.

---

## 🙌 Acknowledgements

- OptiTrack NatNet SDK  
- ROS 1 / ROS 2 community  
- All upstream packages vendored or referenced here