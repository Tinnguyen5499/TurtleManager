import os
import sys
import subprocess
import psutil  # To manage processes
import time
import signal
from threading import Thread
from threading import Event
from PyQt6 import QtWidgets
from PyQt6.QtGui import QPalette, QColor
from PyQt6.QtCore import QMetaObject, Qt
from PyQt6.QtCore import QTimer
from GUI_setting import Ui_Dialog
from PyQt6.QtCore import QObject, pyqtSignal, QThread
from GUI_interface import Ui_MainWindow  # Replace this with your actual UI file if named differently


class MonitorThread(QThread):
    status_signal = pyqtSignal(int, str)  # (TurtleBot index, "red" or "green")

    def __init__(self, turtlebots, parent=None):
        super().__init__(parent)
        self.turtlebots = turtlebots
        self.running = True

    def run(self):
        while self.running:
            for idx, turtlebot in enumerate(self.turtlebots):
                node_filter = f"robot_{idx + 1}"
                topic_filter = f"/robot_{idx + 1}"

                is_node_running = self.check_turtlebot_node(node_filter)
                is_topic_active = self.check_turtlebot_topic(topic_filter)

                status = "green" if is_node_running and is_topic_active else "red"
                self.status_signal.emit(idx, status)

            time.sleep(5)  # Poll every 5 seconds

    def stop(self):
        self.running = False

    def check_turtlebot_node(self, node_filter):
        """Check if any node contains the specified filter (e.g., 'robot_')."""
        try:
            command = "source /opt/ros/noetic/setup.bash && rosnode list"
            result = subprocess.run(command, capture_output=True, text=True, shell=True, executable='/bin/bash')
            if result.returncode != 0:
                return False

            nodes = result.stdout.splitlines()
            return any(node_filter in node for node in nodes)
        except Exception as e:
            print(f"[ERROR] Node check failed: {e}")
            return False

    def check_turtlebot_topic(self, topic_filter):
        """Check if any topic contains the specified filter (e.g., 'robot_')."""
        try:
            command = "source /opt/ros/noetic/setup.bash && rostopic list"
            result = subprocess.run(command, capture_output=True, text=True, shell=True, executable='/bin/bash')
            if result.returncode != 0:
                return False

            topics = result.stdout.splitlines()
            return any(topic_filter in topic for topic in topics)
        except Exception as e:
            print(f"[ERROR] Topic check failed: {e}")
            return False

class SettingsDialog(QtWidgets.QDialog, Ui_Dialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        # Set default TurtleBot IPs
        self.lineEdit_3.setText("192.168.0.141")
        self.lineEdit_4.setText("192.168.0.142")
        self.lineEdit_5.setText("192.168.0.143")
        self.lineEdit_6.setText("192.168.0.144")
        self.lineEdit_7.setText("192.168.0.145")
        self.lineEdit_8.setText("192.168.0.146")

        # Get and set the host IP
        self.host_ip = self.get_host_ip()
        self.lineEdit_2.setText(self.host_ip)
        self.lineEdit.setText(f"http://{self.host_ip}:11311")

    def get_host_ip(self):
        """Get the host IP address using `hostname -I` (local network IP)."""
        try:
            # Get the first available local IP address (usually Wi-Fi or Ethernet)
            output = subprocess.check_output(["hostname", "-I"], text=True).strip()
            ip = output.split()[0]
            return ip
        except Exception as e:
            print(f"Error getting local host IP: {e}")
            return "127.0.0.1"  # Fallback to localhost if something goes wrong



    def apply_settings(self):
        """Export environment variables based on the dialog inputs."""
        ros_master_uri = self.lineEdit.text()
        host_ros_ip = self.lineEdit_2.text()

        os.environ["ROS_MASTER_URI"] = ros_master_uri
        os.environ["ROS_IP"] = host_ros_ip
        print(f"Exported: ROS_MASTER_URI={ros_master_uri}, ROS_IP={host_ros_ip}")


class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.setupUi(self)

        self.password = "pw"
        # Track selected mode: "run", "scripts", or "launch"
        self.current_mode = "run"

        # Renaming buttons for ease of use
        self.selectAllButton = self.pushButton
        self.addScriptButton = self.pushButton_10
        self.editScriptButton = self.pushButton_13
        self.deleteScriptButton = self.pushButton_14  # New button to delete scripts
        self.runScriptButton = self.pushButton_11
        self.turtleBot1CheckBox = self.checkBox
        self.turtleBot2CheckBox = self.checkBox_2
        self.turtleBot3CheckBox = self.checkBox_3
        self.turtleBot4CheckBox = self.checkBox_4
        self.turtleBot5CheckBox = self.checkBox_5
        self.turtleBot6CheckBox = self.checkBox_6
        # Optitrack setup
        self.optitrackCheckBox = self.checkBox_7  # Assuming checkbox_7 is assigned to OptiTrack
        self.optitrackCheckBox.toggled.connect(self.toggleOptiTrack)

        self.optitrack_session_name = "optitrack_session"  # TMUX session name for OptiTrack
        self.packageComboBox = self.comboBox
        self.scriptComboBox = self.comboBox_2
        self.roscoreCheckBox = self.checkBox_8  # Assuming this is the checkbox for Roscore
        self.roscoreCheckBox.toggled.connect(self.toggleRoscore)
        self.rosbridgeCheckBox = self.checkBox_9  # Assuming this is the checkbox for Rosbridge
        self.rosbridgeCheckBox.toggled.connect(self.toggleRosbridge)
        self.rviz2CheckBox = self.checkBox_10  # Assign the Rviz2 checkbox
        self.rviz2CheckBox.stateChanged.connect(self.toggleRviz2)
        self.gazeboCheckBox = self.checkBox_11  # New checkbox for Gazebo
        self.gazebo_process = None  # Process variable for Gazebo
        self.startButton=self.pushButton_9


        self.restartAllButton = self.pushButton_2
        self.restartAllButton.clicked.connect(self.restart_all_turtlebots)


        # Connect the Gazebo checkbox to its toggle function
        self.gazeboCheckBox.stateChanged.connect(self.toggleGazebo)

        # Setup settings dialog
        self.settings_dialog = SettingsDialog(self)

        # Add Settings menu action
        self.settingsAction = self.menuSetting.addAction("Settings")
        self.settingsAction.triggered.connect(self.open_settings)

        # Export environment variables on startup
        self.export_environment()  # Export environment variables when the GUI starts



        # Path to your ROS 2 workspace
        self.workspace_path = "/root/ros2_ws"

        # Connect buttons and dropdowns to their actions
        self.selectAllButton.clicked.connect(self.selectAllWasPressed)
        self.addScriptButton.clicked.connect(self.addNewScript)
        self.editScriptButton.clicked.connect(self.editSelectedScript)
        self.deleteScriptButton.clicked.connect(self.deleteSelectedScript)  # Connect Delete Script button
        self.runScriptButton.clicked.connect(self.runSelectedScript)
        self.packageComboBox.currentTextChanged.connect(self.updateScripts)


        # Populate packages from the src directory
        self.populatePackages()

        self.turtlebots = [
            {
                "checkbox": self.checkBox,
                "radio": self.radioButton,
                "ip_field": "lineEdit_3",  # Turtlebot1_IP
                "monitoring": False,
            },
            {
                "checkbox": self.checkBox_2,
                "radio": self.radioButton_2,
                "ip_field": "lineEdit_4",  # Turtlebot2_IP
                "monitoring": False,
            },
            {
                "checkbox": self.checkBox_3,
                "radio": self.radioButton_3,
                "ip_field": "lineEdit_5",  # Turtlebot3_IP
                "monitoring": False,
            },
            {
                "checkbox": self.checkBox_4,
                "radio": self.radioButton_4,
                "ip_field": "lineEdit_6",  # Turtlebot4_IP
                "monitoring": False,
            },
            {
                "checkbox": self.checkBox_5,
                "radio": self.radioButton_5,
                "ip_field": "lineEdit_7",  # Turtlebot5_IP
                "monitoring": False,
            },
            {
                "checkbox": self.checkBox_6,
                "radio": self.radioButton_6,
                "ip_field": "lineEdit_8",  # Turtlebot6_IP
                "monitoring": False,
            },
        ]


        self.startButton.clicked.connect(self.start_turtlebots)
        self.resetButtons = [
            self.pushButton_3,
            self.pushButton_4,
            self.pushButton_5,
            self.pushButton_6,
            self.pushButton_7,
            self.pushButton_8,
        ]

        for idx, reset_button in enumerate(self.resetButtons):
            reset_button.clicked.connect(lambda _, i=idx: self.reset_turtlebot(i))


        self.monitor_thread = MonitorThread(self.turtlebots)
        self.monitor_thread.status_signal.connect(self.update_turtlebot_status)

    def closeEvent(self, event):
        """Override closeEvent to stop monitoring and clean up before exit."""
        self.stop_monitoring()  # Stop the monitor thread cleanly
        event.accept()  # Allow the window to close


    ##################Optitrack Button Setup####################

    def show_mocap_instructions(self):
        msg = QtWidgets.QMessageBox(self)
        msg.setIcon(QtWidgets.QMessageBox.Icon.Information)
        msg.setWindowTitle("OptiTrack Mapping Guide")
        msg.setText(
            "OptiTrack sends mocap bodies → TurtleBots.\n\n"
            "Two mapping options:\n"
            "1. Rename rigid bodies in Motive as 'robot_2', 'robot_3', ...\n"
            "   → They map directly to those robots.\n\n"
            "2. If names don't match → fallback to first-come-first-seen order\n"
            "   (first body becomes robot_2, next robot_3, ...).\n\n"
            "Note: Mapping always starts at robot_2 since our lab has no robot_1."
        )
        msg.addButton("OK", QtWidgets.QMessageBox.ButtonRole.AcceptRole)
        msg.exec()



    def toggleOptiTrack(self, checked):
        """Toggle the OptiTrack setup using tmux."""
        session_name = "optitrack_session"
        import pathlib, subprocess, os
        TM_DIR = pathlib.Path(__file__).resolve().parent
        TURTLE_WS = TM_DIR / "turtle_ws"
        SETUP = TURTLE_WS / "install" / "setup.bash"

        if not SETUP.exists():
            QtWidgets.QMessageBox.warning(
                self, "OptiTrack workspace not built",
                "The embedded ROS 2 workspace (turtle_ws) isn’t built yet.\n"
                "Open a terminal and run:\n\n"
                "  source /opt/ros/foxy/setup.bash\n"
                "  cd ~/TurtleManager/turtle_ws\n"
                "  rosdep update && rosdep install --from-paths src --ignore-src -r -y\n"
                "  colcon build --symlink-install"
            )
            return

        ws = str(TURTLE_WS)  # ensure clean string paths for bash

        if checked:
            self.show_mocap_instructions()
            print("[INFO] Starting OptiTrack…")

            # Fresh tmux session
            subprocess.run(["tmux", "kill-session", "-t", session_name],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["tmux", "new-session", "-d", "-s", session_name, "-n", "optitrack"])

            # Pane 0: Launch the OptiTrack client (from the repo's launch dir)
            launch_command = (
                "source /opt/ros/foxy/setup.bash && "
                f"source {ws}/install/setup.bash && "
                f"cd {ws}/src/ros2-mocap_optitrack/launch && "
                "ros2 launch launch_z_up.py"
            )
            subprocess.run(["tmux", "send-keys", "-t", f"{session_name}:0.0", launch_command, "C-m"])

            # Pane 1: Run the new process_mocap node (name-based by default; change mode if you prefer)
            subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:0"])
            subprocess.run(["tmux", "select-layout", "-t", f"{session_name}:0", "tiled"])

            mocap_command = (
                "source /opt/ros/foxy/setup.bash && "
                f"source {ws}/install/setup.bash && "
                "ros2 run process_mocap process_mocap "
                "--ros-args "
                "-p mode:=name "
                "-p robot_namespaces:='[robot_2, robot_3, robot_4, robot_5]'"
            )
            # (If you want first-seen as default, change -p mode:=first_seen)

            subprocess.run(["tmux", "send-keys", "-t", f"{session_name}:0.1", mocap_command, "C-m"])

            # Attach in GNOME Terminal
            try:
                subprocess.Popen(["gnome-terminal", "--", "tmux", "attach-session", "-t", session_name])
            except FileNotFoundError:
                print("[ERROR] gnome-terminal is not installed.")
            except Exception as e:
                print(f"[ERROR] Failed to open gnome-terminal: {e}")

        else:
            print("[INFO] Stopping OptiTrack…")
            result = subprocess.run(["tmux", "kill-session", "-t", session_name],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if result.returncode == 0:
                print("[INFO] OptiTrack session terminated successfully.")
            else:
                print(f"[ERROR] Failed to terminate OptiTrack session: {result.stderr.decode().strip()}")


    #####################new radio buttons for scripts modes ###########################################     


        # Connect radio buttons
        self.radioButton_7.toggled.connect(lambda checked: self.switch_mode("run", checked))
        self.radioButton_8.toggled.connect(lambda checked: self.switch_mode("scripts", checked))
        self.radioButton_9.toggled.connect(lambda checked: self.switch_mode("launch", checked))

    def switch_mode(self, mode, checked):
        if checked:
            self.current_mode = mode
            print(f"Switched to mode: {self.current_mode}")
            self.updateScripts(self.packageComboBox.currentText())

        ##################################################################

    def getTargetFolder(self, package_name):
        """Determine the target folder based on the current mode."""
        package_folder = self.packages[package_name]

        if self.current_mode == "scripts":
            target_folder = os.path.join(package_folder, "scripts")
        elif self.current_mode == "launch":
            target_folder = os.path.join(package_folder, "launch")
        else:  # "run" mode assumes scripts are directly in the nested package folder
            target_folder = os.path.join(package_folder, package_name)

        # Ensure the target folder exists
        if not os.path.exists(target_folder):
            os.makedirs(target_folder, exist_ok=True)

        return target_folder    

    

#########################Turtlebots###########################################################################
########################Turtlebots###########################################################################

# Renaming buttons for ease of use



    def launch_turtlebots_with_gnome_tmux(self, master_uri, turtlebot_ips):
        """Launch selected Turtlebots using tmux in a new GNOME Terminal."""
        session_name = "turtlebots_session"
        pane_indexes = ["0.0"]  # First pane starts as "0.0"
        self.turtlebots_pane_mapping = {}  # Map each TurtleBot index to its tmux pane

        # Kill any existing session
        subprocess.run(["tmux", "kill-session", "-t", session_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Create a new tmux session
        subprocess.run(["tmux", "new-session", "-d", "-s", session_name, "-n", "turtlebots"])

        for idx, turtlebot in enumerate(self.turtlebots):
            if not turtlebot["checkbox"].isChecked():
                continue  # Skip if not selected

            ip = turtlebot_ips[idx]
            turtlebot_nbr = idx + 1

            # Split and create a new pane if it's not the first TurtleBot
            if idx > 0:
                subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:0"])
                subprocess.run(["tmux", "select-layout", "-t", f"{session_name}:0", "tiled"])
                time.sleep(1)  # Ensure tmux has time to initialize

                # Get the latest pane index dynamically
                pane_index = self.get_latest_pane_index(session_name)
                if pane_index is not None:
                    pane_assignment = f"0.{pane_index}"
                    pane_indexes.append(pane_assignment)
                else:
                    print(f"[ERROR] Could not retrieve a valid pane index for TurtleBot {turtlebot_nbr}. Skipping...")
                    continue

            # Store the mapping of the TurtleBot to its corresponding tmux pane
            self.turtlebots_pane_mapping[idx] = pane_indexes[-1]

            # Prepare and send the command
            command = (
                f"sshpass -p '{self.password}' ssh -o StrictHostKeyChecking=no -t ubuntu@{ip} "
                f"\"bash -i -c 'export ROS_MASTER_URI={master_uri} && "
                f"source /opt/ros/noetic/setup.bash && "
                f"source ~/catkin_ws/devel/setup.bash && "
                f"export TURTLEBOT_NBR={turtlebot_nbr} && "
                f"exec roslaunch turtlebot_bringup minimal.launch'\""
            )

            # Send the command to the appropriate tmux pane
            pane_to_use = pane_indexes[-1]
            print(f"[DEBUG] Sending command to TurtleBot {turtlebot_nbr} on tmux pane {pane_to_use}...")
            subprocess.run(["tmux", "send-keys", "-t", f"{session_name}:{pane_to_use}", command, "C-m"])

        # Open GNOME Terminal and attach to the tmux session
        try:
            subprocess.Popen(["gnome-terminal", "--", "tmux", "attach-session", "-t", session_name])
        except FileNotFoundError:
            print("[ERROR] gnome-terminal is not installed. Install it using:\n  sudo apt-get install gnome-terminal")
        except Exception as e:
            print(f"[ERROR] Failed to open gnome-terminal: {e}")


    def get_latest_pane_index(self, session_name):
        """Retrieve the latest tmux pane index dynamically."""
        result = subprocess.run(
            ["tmux", "list-panes", "-t", f"{session_name}:0", "-F", "#{pane_index}"],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            pane_indexes = result.stdout.strip().split('\n')
            return max(map(int, pane_indexes)) if pane_indexes else None
        return None


    def open_settings(self):
        """Show the Settings dialog."""
        if self.settings_dialog.exec() == QtWidgets.QDialog.DialogCode.Accepted:
            self.settings_dialog.apply_settings()

    def get_turtlebot_ips(self):
        """Retrieve Turtlebot IPs dynamically from the settings menu."""
        turtlebot_ips = []
        for turtlebot in self.turtlebots:
            ip_field_name = turtlebot["ip_field"]
            ip_field = getattr(self.settings_dialog, ip_field_name, None)
            if ip_field:
                turtlebot_ips.append(ip_field.text())
        return turtlebot_ips

    def start_turtlebots(self):
        master_uri, _ = self.export_environment()
        turtlebot_ips = self.get_turtlebot_ips()
        
        for turtlebot in self.turtlebots:
            self.update_radio_button(turtlebot["radio"], "red")

        self.launch_turtlebots_with_gnome_tmux(master_uri, turtlebot_ips)
        
        # Start monitoring
        self.start_monitoring()



    def disconnect_turtlebot(self, idx):
        """Disconnect from a Turtlebot."""
        radio_button = self.turtlebots[idx]["radio"]
        radio_button.setChecked(False)  # Turn off the radio button


    def reset_turtlebot(self, idx):
        """Reset the specified TurtleBot by stopping its tmux process and rebooting the robot."""
        turtlebot_ips = self.get_turtlebot_ips()
        ip = turtlebot_ips[idx]

        def reset_task():
            try:
                # Step 1: Find the corresponding tmux pane and stop the process using CTRL+C
                pane_index = self.turtlebots_pane_mapping.get(idx)  # Get the correct pane for this TurtleBot
                tmux_session_name = "turtlebots_session"

                if pane_index is not None:
                    print(f"[INFO] Stopping TurtleBot {idx + 1} on tmux pane {pane_index}...")
                    subprocess.run(["tmux", "send-keys", "-t", f"{tmux_session_name}:{pane_index}", "C-c"])
                    time.sleep(2)  # Allow processes to terminate

                # Step 2: Send the reboot command and handle potential errors
                command = f"sshpass -p '{self.password}' ssh -o StrictHostKeyChecking=no ubuntu@{ip} 'sudo reboot'"
                print(f"[INFO] Sending reboot command to TurtleBot at {ip}...")
                result = subprocess.run(command, shell=True, capture_output=True, text=True)

                # Step 3: Determine whether the reboot was successful and notify the user on the main thread
                if result.returncode != 0:
                    self.invoke_messagebox(
                        "Reset Failed",
                        f"Failed to reset TurtleBot at {ip}.\n{result.stderr.strip()}",
                        critical=True,
                    )
                else:
                    self.invoke_messagebox(
                        "Reboot Sent",
                        f"TurtleBot at {ip} is rebooting successfully."
                    )

            except Exception as e:
                print(f"[ERROR] Unexpected error resetting TurtleBot at {ip}: {e}")
                self.invoke_messagebox("Error", f"Unexpected error resetting TurtleBot at {ip}.\n{e}", critical=True)

        # Run the reset task in a separate thread
        thread = Thread(target=reset_task)
        thread.start()

    def invoke_messagebox(self, title, message, critical=False):
        """Safely display QMessageBox using the main thread."""
        QMetaObject.invokeMethod(
            self,
            "show_messagebox",
            Qt.ConnectionType.BlockingQueuedConnection,
            title,
            message,
            critical,
        )

    def show_messagebox(self, title, message, critical=False):
        """Display QMessageBox from the main thread."""
        if critical:
            QtWidgets.QMessageBox.critical(None, title, message)
        else:
            QtWidgets.QMessageBox.information(None, title, message)


    def update_radio_button(self, radio_button, color):
        """Update the radio button text color correctly using style sheets."""
        if color == "green":
            radio_button.setChecked(True)
            # Apply green style for checked state
            radio_button.setStyleSheet("""
                QRadioButton {
                    color: green;
                }
            """)
        elif color == "red":
            radio_button.setChecked(True)
            # Apply red style for unchecked state
            radio_button.setStyleSheet("""
                QRadioButton {
                    color: red;
                }
            """)
            
        radio_button.update()  # Ensure immediate UI refresh



    def start_monitoring(self):
        self.monitor_thread.start()

    def stop_monitoring(self):
        self.monitor_thread.stop()
        self.monitor_thread.wait()  # Ensure the thread exits cleanly

    def update_turtlebot_status(self, idx, color):
        self.update_radio_button(self.turtlebots[idx]["radio"], color)


    def check_turtlebot_node(self, node_filter):
        """Check if any node contains the specified filter (e.g., 'robot_')."""
        try:
            # Ensure environment variables are set
            ros_master_uri, ros_ip = self.export_environment()

            # Run rosnode list to find active nodes
            command = (
                f"export ROS_MASTER_URI={ros_master_uri} && "
                f"export ROS_IP={ros_ip} && "
                "source /opt/ros/noetic/setup.bash && rosnode list"
            )
            result = subprocess.run(command, capture_output=True, text=True, shell=True, executable='/bin/bash')

            if result.returncode != 0:
                print(f"[ERROR] Error checking nodes: {result.stderr}")
                return False

            nodes = result.stdout.splitlines()

            # Check if any node contains the node filter
            return any(node_filter in node for node in nodes)
        except Exception as e:
            print(f"[ERROR] Error checking nodes with filter '{node_filter}': {e}")
            return False


    def check_turtlebot_topic(self, topic_filter):
        """Check if any topic contains the specified filter (e.g., 'robot_')."""
        try:
            # Ensure environment variables are set
            ros_master_uri, ros_ip = self.export_environment()

            # Command to list topics using ROS 1 (rostopic list)
            command_list = (
                f"export ROS_MASTER_URI={ros_master_uri} && "
                f"export ROS_IP={ros_ip} && "
                "source /opt/ros/noetic/setup.bash && rostopic list"
            )
            result_list = subprocess.run(command_list, capture_output=True, text=True, shell=True, executable='/bin/bash')

            if result_list.returncode != 0:
                print(f"[ERROR] Error listing topics: {result_list.stderr}")
                return False

            topics = result_list.stdout.splitlines()

            # Check if any topic contains the topic filter
            return any(topic_filter in topic for topic in topics)
        except Exception as e:
            print(f"[ERROR] Error checking topics with filter '{topic_filter}': {e}")
            return False

    def restart_all_turtlebots(self):
        """Restart all selected TurtleBots."""
        selected_bots = [idx for idx, turtlebot in enumerate(self.turtlebots) if turtlebot["checkbox"].isChecked()]

        if not selected_bots:
            QtWidgets.QMessageBox.warning(self, "No Selection", "Please select at least one TurtleBot to restart.")
            return

        for idx in selected_bots:
            print(f"[INFO] Restarting TurtleBot {idx + 1}...")
            self.reset_turtlebot(idx)

        QtWidgets.QMessageBox.information(self, "Restarting", "Restart commands have been sent to the selected TurtleBots.")








########################################################################################################

#####################Setting Menus########################################################################3            
    
    def open_settings(self):
        """Show the Settings dialog and apply changes globally if accepted."""
        if self.settings_dialog.exec() == QtWidgets.QDialog.DialogCode.Accepted:
            self.settings_dialog.apply_settings()

            # Re-export environment variables globally after settings change
            os.environ["ROS_MASTER_URI"] = self.settings_dialog.lineEdit.text()
            os.environ["ROS_IP"] = self.settings_dialog.lineEdit_2.text()

            print(f"Updated: ROS_MASTER_URI={os.environ['ROS_MASTER_URI']}, ROS_IP={os.environ['ROS_IP']}")


    def run_command_in_terminal(self,command):
        """Run a command in a new terminal with exported environment variables."""
        try:
            subprocess.Popen(
                ["gnome-terminal", "--", "bash", "-c", command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except FileNotFoundError:
            QtWidgets.QMessageBox.critical(
                None,
                "Error",
                (
                    "The terminal emulator 'gnome-terminal' is not installed.\n\n"
                    "Please install it by running the following command in your terminal:\n\n"
                    "sudo apt-get install gnome-terminal"
                ),
            )
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error", f"Failed to execute command: {e}")

    def export_environment(self):
        """Get and export the environment variables for ROS_MASTER_URI and ROS_IP."""
        try:
            # Get the correct host IP using hostname -I
            host_ip = subprocess.check_output(["hostname", "-I"], text=True).strip().split()[0]
        except Exception as e:
            print(f"Error getting host IP: {e}")
            host_ip = "127.0.0.1"  # Fallback to localhost

        ros_master_uri = f"http://{host_ip}:11311"

        # Export environment variables
        os.environ["ROS_MASTER_URI"] = ros_master_uri
        os.environ["ROS_IP"] = host_ip
        return ros_master_uri, host_ip


    def terminate_process_by_name(self,process_name):
        """Terminate a process by its name or command-line arguments."""
        import psutil
        try:
            found = False
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                if process_name in " ".join(proc.info['cmdline']):
                    proc.terminate()
                    proc.wait()
                    print(f"Process '{process_name}' with PID {proc.info['pid']} terminated.")
                    found = True
            return found
        except Exception as e:
            print(f"Error stopping {process_name}: {e}")
            return False

#####################################################################################################################33



    def selectAllWasPressed(self):
        """Check all TurtleBot checkboxes."""
        self.turtleBot1CheckBox.setChecked(True)
        self.turtleBot2CheckBox.setChecked(True)
        self.turtleBot3CheckBox.setChecked(True)
        self.turtleBot4CheckBox.setChecked(True)
        self.turtleBot5CheckBox.setChecked(True)
        self.turtleBot6CheckBox.setChecked(True)

    def findPackages(self):
        """Recursively scan the src directory for valid ROS 2 packages."""
        packages = {}
        src_path = os.path.join(self.workspace_path, "src")
        if not os.path.exists(src_path):
            QtWidgets.QMessageBox.critical(self, "Error", f"Source directory not found: {src_path}")
            return packages

        # Recursively walk through all directories in src
        for root, dirs, files in os.walk(src_path):
            if "package.xml" in files:  # A valid ROS 2 package must have a package.xml
                package_name = os.path.basename(root)  # Get the folder name
                packages[package_name] = root
        return packages

    def populatePackages(self):
        """Fetch and populate ROS 2 packages from the src directory."""
        try:
            self.packages = self.findPackages()
            if self.packages:
                print(f"Discovered packages: {list(self.packages.keys())}")  # Debug log
                self.packageComboBox.addItems(self.packages.keys())
            else:
                QtWidgets.QMessageBox.warning(self, "Warning", "No ROS 2 packages found in the src directory.")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load ROS 2 packages: {e}")

    def updateScripts(self, package_name):
        self.scriptComboBox.clear()
        if not package_name or package_name not in self.packages:
            return

        try:
            package_folder = self.packages[package_name]

            if self.current_mode == "run":
                nested_folder = os.path.join(package_folder, package_name)
                scripts = [
                    file for file in os.listdir(nested_folder)
                    if file.endswith(".py") and file != "__init__.py" and os.path.isfile(os.path.join(nested_folder, file))
                ]
            elif self.current_mode == "scripts":
                scripts_folder = os.path.join(package_folder, "scripts")
                scripts = [
                    file for file in os.listdir(scripts_folder)
                    if file.endswith(".py") and os.path.isfile(os.path.join(scripts_folder, file))
                ]
            elif self.current_mode == "launch":
                launch_folder = os.path.join(package_folder, "launch")
                scripts = [
                    file for file in os.listdir(launch_folder)
                    if file.endswith(".py") and os.path.isfile(os.path.join(launch_folder, file))
                ]

            if scripts:
                self.scriptComboBox.addItems(scripts)
            else:
                QtWidgets.QMessageBox.warning(self, "Warning", f"No scripts found for {package_name} in {self.current_mode} mode.")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load scripts for {package_name}: {e}")

    def isGeditAvailable(self):
        """Check if gedit is installed and available."""
        return os.system("which gedit > /dev/null 2>&1") == 0

    def addNewScript(self):
        """Prompt the user to name a new script and open a graphical text editor."""
        package_name = self.packageComboBox.currentText()
        if not package_name or package_name not in self.packages:
            QtWidgets.QMessageBox.warning(self, "Warning", "Please select a valid package to add a script.")
            return

        # Prompt the user for the script name
        script_name, ok = QtWidgets.QInputDialog.getText(
            self, "Add New Script", "Enter the name for the new script (e.g., my_script.py):"
        )
        if not ok or not script_name:
            return

        # Ensure the script name ends with .py
        if not script_name.endswith(".py"):
            script_name += ".py"

        target_folder = self.getTargetFolder(package_name)
        script_path = os.path.join(target_folder, script_name)

        # Check if gedit is available
        if not self.isGeditAvailable():
            QtWidgets.QMessageBox.critical(
                self,
                "Error",
                (
                    "The text editor 'gedit' is not installed.\n\n"
                    "Gedit is a graphical application that allows you to easily create and edit text files. "
                    "It is highly recommended for editing Python scripts.\n\n"
                    "To install gedit, please run the following command in your terminal:\n\n"
                    "sudo apt-get install gedit"
                )
            )
            return

        try:
            # Create the script file with a default template
            with open(script_path, "w") as file:
                file.write("def main():\n")
                file.write("    print('Hello from the new script!')\n")

            # Open the script in gedit
            os.system(f"gedit {script_path} &")

            if self.current_mode == "run":
                # Only add entry points if in "run" mode
                self.addToEntryPoint(package_name, script_name)

            QtWidgets.QMessageBox.information(self, "Success", f"Script {script_name} created successfully.")
            self.updateScripts(package_name)

        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to create script: {e}")

    def addToEntryPoint(self, package_name, script_name):
        """Add the script to the entry point in the setup.py of the package."""
        setup_path = os.path.join(self.packages[package_name], "setup.py")
        script_basename = os.path.splitext(script_name)[0]

        try:
            with open(setup_path, "r") as file:
                setup_content = file.readlines()

            # Check if the entry already exists
            new_entry = f"'{script_basename} = {package_name}.{script_basename}:main',"
            if any(new_entry in line for line in setup_content):
                print(f"Entry point for {script_basename} already exists in {setup_path}.")
                return

            # Locate the console_scripts section
            for idx, line in enumerate(setup_content):
                if "'console_scripts': [" in line:
                    console_scripts_idx = idx
                    break
            else:
                QtWidgets.QMessageBox.critical(
                    self,
                    "Error",
                    f"No console_scripts section found in {setup_path}. Unable to add entry point.",
                )
                return

            # Insert the new entry after the console_scripts opening line
            setup_content.insert(console_scripts_idx + 1, f"            {new_entry}\n")

            # Write the updated setup.py file
            with open(setup_path, "w") as file:
                file.writelines(setup_content)

            print(f"Successfully added entry point for {script_basename} to {setup_path}.")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to update setup.py: {e}")

    def editSelectedScript(self):
        """Open the selected script in gedit for editing."""
        package_name = self.packageComboBox.currentText()
        script_name = self.scriptComboBox.currentText()

        if not package_name or package_name not in self.packages:
            QtWidgets.QMessageBox.warning(self, "Warning", "Please select a valid package.")
            return
        if not script_name:
            QtWidgets.QMessageBox.warning(self, "Warning", "Please select a valid script to edit.")
            return

        target_folder = self.getTargetFolder(package_name)
        script_path = os.path.join(target_folder, script_name)

        if not os.path.exists(script_path):
            QtWidgets.QMessageBox.critical(self, "Error", f"Script not found: {script_path}")
            return

        if not self.isGeditAvailable():
            QtWidgets.QMessageBox.critical(
                self,
                "Error",
                (
                    "The text editor 'gedit' is not installed.\n\n"
                    "To install gedit, run the following command in your terminal:\n\n"
                    "sudo apt-get install gedit"
                ),
            )
            return

        # Open the script in gedit
        os.system(f"gedit {script_path} &")

    def deleteSelectedScript(self):
        """Delete the selected script and its entry point in setup.py."""
        package_name = self.packageComboBox.currentText()
        script_name = self.scriptComboBox.currentText()

        if not package_name or package_name not in self.packages:
            QtWidgets.QMessageBox.warning(self, "Warning", "Please select a valid package.")
            return
        if not script_name:
            QtWidgets.QMessageBox.warning(self, "Warning", "Please select a valid script to delete.")
            return

        # Ask the user for confirmation
        reply = QtWidgets.QMessageBox.question(
            self,
            "Confirm Deletion",
            f"Are you sure you want to delete the script '{script_name}' in the package '{package_name}'?",
            QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
        )

        if reply != QtWidgets.QMessageBox.StandardButton.Yes:
            return

        target_folder = self.getTargetFolder(package_name)
        script_path = os.path.join(target_folder, script_name)

        try:
            # Remove the script file
            if os.path.exists(script_path):
                os.remove(script_path)
                print(f"Deleted script: {script_path}")
            else:
                QtWidgets.QMessageBox.warning(self, "Warning", f"Script not found: {script_path}")

            if self.current_mode == "run":
                # Only remove entry points if in "run" mode
                self.removeFromEntryPoint(package_name, script_name)

            # Refresh the script list
            self.updateScripts(package_name)

            QtWidgets.QMessageBox.information(self, "Success", f"Script '{script_name}' deleted successfully.")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to delete script: {e}")

    def removeFromEntryPoint(self, package_name, script_name):
        """Remove the script's entry point from the setup.py of the package."""
        setup_path = os.path.join(self.packages[package_name], "setup.py")
        script_basename = os.path.splitext(script_name)[0]

        try:
            with open(setup_path, "r") as file:
                setup_content = file.readlines()

            # Identify and remove the entry
            new_content = []
            entry_removed = False
            for line in setup_content:
                if f"'{script_basename} = {package_name}.{script_basename}:main'," in line:
                    entry_removed = True
                    continue
                new_content.append(line)

            if entry_removed:
                with open(setup_path, "w") as file:
                    file.writelines(new_content)
                print(f"Entry point for {script_basename} removed from {setup_path}.")
            else:
                print(f"No entry point found for {script_basename} in {setup_path}.")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to update setup.py: {e}")

    def runSelectedScript(self):
        """Run the selected script using the mode selected (ros2 run, scripts folder, or launch)."""
        package_name = self.packageComboBox.currentText()
        script_name = self.scriptComboBox.currentText()

        if not package_name or package_name not in self.packages:
            QtWidgets.QMessageBox.warning(self, "Warning", "Please select a valid package.")
            return
        if not script_name:
            QtWidgets.QMessageBox.warning(self, "Warning", "Please select a valid script to run.")
            return

        # Remove the ".py" extension to match the entry point in setup.py when in "run" mode
        executable_name = os.path.splitext(script_name)[0]

        try:
            if self.current_mode == "run":
                # Build the package
                build_command = f"colcon build --packages-select {package_name}"
                print(f"Building package: {package_name}")
                build_result = subprocess.run(
                    ["bash", "-c", build_command],
                    cwd=self.workspace_path,
                    capture_output=True,
                    text=True,
                )

                if build_result.returncode != 0:
                    print(f"Build failed: {build_result.stderr}")
                    QtWidgets.QMessageBox.critical(
                        self,
                        "Build Error",
                        f"Failed to build the package '{package_name}'.\n\nError:\n{build_result.stderr}",
                    )
                    return
                else:
                    print(f"Package '{package_name}' built successfully.")

                # Source the workspace after the build
                source_command = f"source /opt/ros/foxy/setup.bash && source {self.workspace_path}/install/setup.bash"

                # Construct the ros2 run command
                run_command = f"{source_command} && ros2 run {package_name} {executable_name}"

            elif self.current_mode == "scripts":
                # Run the selected script directly
                script_path = os.path.join(self.packages[package_name], "scripts", script_name)
                run_command = f"source /opt/ros/foxy/setup.bash && source {self.workspace_path}/install/setup.bash && python3 {script_path}"

            elif self.current_mode == "launch":
                # Run the launch file
                launch_file = os.path.join(self.packages[package_name], "launch", script_name)
                run_command = f"source /opt/ros/foxy/setup.bash && ros2 launch {package_name} {script_name}"

            print(f"Running command using tmux: {run_command}")

            # Use tmux to run the command
            session_name = f"script_{executable_name}_session"

            # Check if tmux session already exists
            result = subprocess.run(["tmux", "has-session", "-t", session_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if result.returncode == 0:
                # Kill session if it already exists to avoid conflicts
                subprocess.run(["tmux", "kill-session", "-t", session_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # Create a new tmux session and run the command
            subprocess.run(["tmux", "new-session", "-d", "-s", session_name, "bash"])
            subprocess.run(["tmux", "send-keys", "-t", session_name, run_command, "C-m"])

            # Attach to the session in a new terminal window
            try:
                subprocess.Popen(["gnome-terminal", "--", "tmux", "attach-session", "-t", session_name])
                QtWidgets.QMessageBox.information(self, "Script Started", f"Script '{script_name}' has been started using tmux.")
            except FileNotFoundError:
                QtWidgets.QMessageBox.critical(
                    None,
                    "Error",
                    (
                        "The terminal emulator 'gnome-terminal' is not installed.\n\n"
                        "Please install it by running the following command in your terminal:\n\n"
                        "sudo apt-get install gnome-terminal"
                    ),
                )
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Error", f"Failed to start the terminal: {e}")

        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to run the script: {e}")

    def toggleRoscore(self, checked):
        """Toggle the roscore process."""
        if checked:
            ros_master_uri, host_ip = self.export_environment()
            command = (
                f"export ROS_MASTER_URI={ros_master_uri} && "
                f"export ROS_IP={host_ip} && "
                "source /opt/ros/noetic/setup.bash && roscore"
            )
            print(f"Starting roscore with command: {command}")
            self.run_command_in_terminal(command)
            QtWidgets.QMessageBox.information(self, "Roscore Started", "Roscore has been started successfully.")
        else:
            if self.terminate_process_by_name("roscore"):
                QtWidgets.QMessageBox.information(self, "Roscore Stopped", "Roscore has been stopped successfully.")
            else:
                QtWidgets.QMessageBox.warning(self, "Warning", "No active roscore process to stop.")

    def toggleRosbridge(self, checked):
        """Toggle the rosbridge process."""
        if checked:
            ros_master_uri, host_ip = self.export_environment()
            command = (
                f"export ROS_MASTER_URI={ros_master_uri} && "
                f"export ROS_IP={host_ip} && "
                "source /opt/ros/noetic/setup.bash && "
                "source /opt/ros/foxy/setup.bash && "
                "ros2 run ros1_bridge dynamic_bridge --bridge-all-topics"
            )
            print(f"Starting rosbridge with command: {command}")
            self.run_command_in_terminal(command)
            QtWidgets.QMessageBox.information(self, "Rosbridge Started", "Rosbridge has been started successfully.")
        else:
            if self.terminate_process_by_name("dynamic_bridge"):
                QtWidgets.QMessageBox.information(self, "Rosbridge Stopped", "Rosbridge has been stopped successfully.")
            else:
                QtWidgets.QMessageBox.warning(self, "Warning", "No active rosbridge process to stop.")

    def toggleRviz2(self, checked):
        """Toggle the Rviz2 process based on the checkbox state."""
        import psutil

        if checked:
            try:
                # Start Rviz2 in a new terminal
                command = "bash -c 'source /opt/ros/foxy/setup.bash && rviz2'"
                print(f"Starting Rviz2 with command: {command}")
                subprocess.Popen(
                    ["gnome-terminal", "--", "bash", "-c", command],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                QtWidgets.QMessageBox.information(self, "Rviz2 Started", "Rviz2 has been started successfully.")
            except FileNotFoundError:
                QtWidgets.QMessageBox.critical(
                    self,
                    "Error",
                    (
                        "The terminal emulator 'gnome-terminal' is not installed.\n\n"
                        "Please install it by running the following command in your terminal:\n\n"
                        "sudo apt-get install gnome-terminal"
                    ),
                )
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Error", f"Failed to start Rviz2: {e}")
        else:
            # Stop Rviz2 process by its name or command line
            try:
                rviz2_found = False
                for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                    # Match processes with "rviz2" in their command-line arguments
                    if "rviz2" in " ".join(proc.info['cmdline']):
                        proc.terminate()
                        proc.wait()
                        print(f"Rviz2 process with PID {proc.info['pid']} terminated.")
                        QtWidgets.QMessageBox.information(self, "Rviz2 Stopped", "Rviz2 has been stopped successfully.")
                        rviz2_found = True
                        break

                if not rviz2_found:
                    QtWidgets.QMessageBox.warning(self, "Warning", "No active Rviz2 process to stop.")
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Error", f"Failed to stop Rviz2: {e}")
    
    def toggleGazebo(self, checked):
        """Toggle the Gazebo process based on the checkbox state."""
        if checked:
            try:
                # Start Gazebo in a new terminal
                command = (
                    "source /opt/ros/foxy/setup.bash && "
                    "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models:/opt/ros/foxy/share/turtlebot3_gazebo/models && "
                    "export TURTLEBOT3_MODEL=burger && "
                    "ros2 launch turtlebot3_gazebo empty_world.launch.py"
                )
                print(f"Starting Gazebo with command: {command}")
                self.gazebo_process = subprocess.Popen(
                    ["gnome-terminal", "--", "bash", "-c", command],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setpgrp,  # Detach the process
                )
                QtWidgets.QMessageBox.information(self, "Gazebo Started", "Gazebo with TurtleBot3 has been started successfully.")
            except FileNotFoundError:
                QtWidgets.QMessageBox.critical(
                    self,
                    "Error",
                    (
                        "The terminal emulator 'gnome-terminal' is not installed.\n\n"
                        "Please install it by running the following command in your terminal:\n\n"
                        "sudo apt-get install gnome-terminal"
                    ),
                )
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Error", f"Failed to start Gazebo: {e}")
        else:
            try:
                # Check if the tracked process is running
                if hasattr(self, 'gazebo_process') and self.gazebo_process and self.gazebo_process.poll() is None:
                    os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGTERM)
                    self.gazebo_process = None
                    QtWidgets.QMessageBox.information(self, "Gazebo Stopped", "Gazebo has been stopped successfully.")
                else:
                    # Use psutil as a fallback to find and terminate Gazebo processes
                    gazebo_found = False
                    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                        if proc.info['name'] in ['gzserver', 'gzclient'] or 'gazebo' in " ".join(proc.info['cmdline']):
                            proc.terminate()
                            proc.wait()
                            gazebo_found = True
                            print(f"Terminated Gazebo process: {proc.info['pid']} ({proc.info['name']})")
                    
                    if gazebo_found:
                        QtWidgets.QMessageBox.information(self, "Gazebo Stopped", "All Gazebo processes have been stopped.")
                    else:
                        QtWidgets.QMessageBox.warning(self, "Gazebo", "No active Gazebo process to stop.")
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Error", f"Failed to stop Gazebo: {e}")


# Main Application
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec()
