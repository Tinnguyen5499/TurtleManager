# TurtleManager 🐢

A lightweight GUI to run multi‑TurtleBot experiments with **OptiTrack** on **ROS 2 Foxy**.  
It vendors a small ROS 2 workspace (`turtle_ws/`) and a mapper that turns OptiTrack rigid‑body poses into per‑robot topics `/<robot_ns>/pose` — **no manual ROS sourcing required** (the app handles it).

---

## ✨ Features

- One‑click start/stop of OptiTrack + mapper from the GUI
- Two mapping modes from OptiTrack → robots:
  - **Name mode** — name Motive rigid bodies exactly `robot_2`, `robot_3`, …
  - **First‑seen mode (default)** — first body → `robot_2`, next → `robot_3`, …  
    *(We intentionally skip `robot_1`.)*
- Publishes `geometry_msgs/Pose` on `/<robot_ns>/pose` for each robot
- Includes Python package **`process_mocap`** (`ros2 run process_mocap mapper_node`) and a patched OptiTrack client that adds a **`name`** field to each rigid body message

---

## 🧩 Repository Layout

```
TurtleManager/
├─ TurtleManager.py                # GUI launcher (handles sourcing & tmux)
├─ GUI_interface.py
├─ GUI_setting.py
├─ scripts/
│  ├─ install.sh                   # one-time builder for turtle_ws
│  └─ run_turtlemanager.sh         # optional convenience launcher
├─ turtle_ws/
│  └─ src/
│     ├─ ros2-mocap_optitrack/     # OptiTrack client (with name support)
│     └─ process_mocap/            # mapper node (mapper_node)
├─ requirements.txt
├─ .gitignore
└─ README.md
```

---

## ✅ Prerequisites

- Ubuntu **20.04**
- **ROS 2 Foxy** installed at `/opt/ros/foxy`
- OptiTrack Motive streaming on the same network (NatNet **1510/1511** reachable)
- First‑time tools:
  ```bash
  sudo apt-get update
  sudo apt-get install -y python3-colcon-common-extensions python3-rosdep tmux gnome-terminal
  sudo rosdep init || true
  rosdep update
  ```

---

## 🚀 Quick Start (no manual `source` needed)

From the repo root (where `TurtleManager.py` lives):

```bash
python3 -m pip install -r requirements.txt
python3 TurtleManager.py
```

What happens:

- On first run, the embedded workspace (`turtle_ws/`) is built if needed.
- The GUI shows a short **OptiTrack Mapping Guide** (name vs first‑seen).
- Toggling **OptiTrack ON** launches in `tmux`:
  1) OptiTrack client (`ros2-mocap_optitrack`)  
  2) Mapper (`process_mocap/mapper_node`)

You’ll see live logs in the app and in tmux panes.

---

## 🎯 Mapping Rules

Two exclusive modes (set in GUI, default **first_seen**):

1) **Name mode**  
   - In Motive, rename your rigid bodies to **`robot_2`**, **`robot_3`**, …  
   - The mapper forwards poses to matching namespaces.

2) **First‑seen mode (default)**  
   - The first rigid body the client sees is assigned **`robot_2`**, next **`robot_3`**, …  
   - Helpful when people forget to rename assets; still skips `robot_1`.

**Published topics**: for each assigned robot, `/<robot_ns>/pose` (`geometry_msgs/Pose`).

**Raw OptiTrack topic**: `/mocap_rigid_bodies` (`mocap_optitrack_interfaces/RigidBodyArray`)  
Each entry contains: `id`, `valid`, `mean_error`, `pose_stamped`, and `name` (from Motive).

---

## 🛠️ Headless / CLI (optional)

You can run the mapper directly without the GUI, e.g.:

```bash
# Assuming the workspace was already built by the GUI or scripts/install.sh
source /opt/ros/foxy/setup.bash
source ~/TurtleManager/turtle_ws/install/setup.bash
ros2 run process_mocap mapper_node --ros-args \
  -p mode:=first_seen \
  -p robot_namespaces:="['robot_2','robot_3','robot_4','robot_5']"
```

---

## 🧪 Verifying Data Flow

In a new terminal (after the GUI started OptiTrack):

```bash
# See the raw stream with rigid-body names coming from Motive
ros2 topic echo /mocap_rigid_bodies --qos-reliability best_effort

# See per-robot pose (example for robot_2)
ros2 topic echo /robot_2/pose
```

If names are present in the raw stream, you will see e.g.:
```
rigid_bodies:
- id: 105
  ...
  name: robot_3
```

---

## 🧩 What We Changed in OptiTrack ROS 2 Client

- **Added `string name`** to `mocap_optitrack_interfaces/msg/RigidBody.msg`
- In the OptiTrack C++ client:
  - Built an **ID→name** map from the NatNet data descriptions (rigid body descriptors)
  - Populated `RigidBody.name` when publishing `RigidBodyArray`
- The `process_mocap` mapper reads this `name` to support **Name mode**

These modifications are vendored inside `turtle_ws/src/ros2-mocap_optitrack` so users don’t need to patch upstream.

---

## 🧯 Troubleshooting

- **`/mocap_rigid_bodies` silent or missing**  
  - Ensure Motive is streaming (Multicast or Unicast) to your machine.
  - Verify NatNet ports **1510/1511** are open and not firewalled.
  - In the GUI, stop/start OptiTrack again.

- **Deserialization errors when echoing**  
  - Mismatch between message types and running nodes. Rebuild interfaces:
    ```bash
    cd ~/TurtleManager/turtle_ws
    source /opt/ros/foxy/setup.bash
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install --packages-select \
      mocap_optitrack_interfaces mocap_optitrack_client process_mocap
    ```

- **No `/robot_* /pose` updates**  
  - In **first_seen** mode, ensure at least one rigid body is visible.
  - In **name** mode, ensure the Motive name matches robot namespace exactly.

- **GUI can’t open terminal**  
  - If `gnome-terminal` isn’t installed, GUI will still run; use `tmux attach -t optitrack_session` to view logs.

---

## 📦 Developer Notes

- GUI starts tmux session `optitrack_session` with two panes (client + mapper).
- If you change ROS code under `turtle_ws/src/`, rebuild:
  ```bash
  cd ~/TurtleManager/turtle_ws
  source /opt/ros/foxy/setup.bash
  colcon build --symlink-install
  ```
- Keep `build/`, `install/`, and `log/` out of version control (`.gitignore`).

---

## 📄 License

MIT License — see `LICENSE` in the repo.  
OptiTrack SDK and ROS 2 dependencies retain their respective licenses.

---

## 🙌 Acknowledgements

- OptiTrack NatNet SDK
- ROS 2 Foxy Fitzroy
- Community packages the project vendors and builds upon

---

## 📫 Support

Please open an issue in the repository with a short description, your Ubuntu/ROS versions, and relevant logs (copy/paste from the GUI/tmux).