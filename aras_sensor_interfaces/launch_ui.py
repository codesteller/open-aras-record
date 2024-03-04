#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QListWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt, QTimer
import subprocess
import threading
import os

class RecordThread(threading.Thread):
    def __init__(self, command):
        super().__init__()
        self.command = command
        self._stop_event = threading.Event()

    def run(self):
        subprocess.Popen(self.command)
        self._stop_event.wait()

    def stop(self):
        self._stop_event.set()



class ROSLauncherUI(QWidget):
    def __init__(self):
        super().__init__()

        self.rosbag_process = None  # To store the rosbag recording process
        self.recording_indicator = QLabel('Recording OFF')

        recording_dir = "./Recordings"
        if not os.path.exists(recording_dir):
            print("Creating Recording Directory")
            os.makedirs(self.recording_dir)

        self.recording_prefix = os.path.join(recording_dir, 'ARAS_Bike_')

        # Create & Initialize the UI
        self.init_ui()


    def init_ui(self):
         # Create main layout
        main_layout = QVBoxLayout()

        
        # Create horizontal layout for buttons
        start_buttons_layout = QHBoxLayout()
        stop_buttons_layout = QHBoxLayout()

        # Create launch buttons
        self.launch_camera_button = QPushButton('Launch Camera')
        self.launch_camera_button.clicked.connect(self.launch_camera)

        self.stop_camera_button = QPushButton('Stop Camera')
        self.stop_camera_button.clicked.connect(self.stop_camera)

        self.launch_radar_button = QPushButton('Launch Radar AWR1843')
        self.launch_radar_button.clicked.connect(self.launch_radar)

        self.stop_radar_button = QPushButton('Stop Radar AWR1843')
        self.stop_radar_button.clicked.connect(self.stop_radar)

        self.launch_vedyne_button = QPushButton('Launch Vedyne (Vehicle Dynamics Module)')
        self.launch_vedyne_button.clicked.connect(self.launch_vedyne)

        self.stop_vedyne_button = QPushButton('Stop Vedyne')
        self.stop_vedyne_button.clicked.connect(self.stop_vedyne)

        self.start_record_button = QPushButton('Start Recording')
        self.start_record_button.clicked.connect(self.start_record)

        self.stop_record_button = QPushButton('Stop Recording')
        self.stop_record_button.clicked.connect(self.stop_record)

        self.stop_all_nodes_button = QPushButton('Stop All Nodes')
        self.stop_all_nodes_button.clicked.connect(self.stop_all_nodes)

        # Add buttons to start buttons layout
        start_buttons_layout.addWidget(self.launch_camera_button)
        start_buttons_layout.addWidget(self.launch_radar_button)
        start_buttons_layout.addWidget(self.launch_vedyne_button)
        start_buttons_layout.addWidget(self.start_record_button)

        # Add buttons to stop buttons layout
        stop_buttons_layout.addWidget(self.stop_camera_button)
        stop_buttons_layout.addWidget(self.stop_radar_button)
        stop_buttons_layout.addWidget(self.stop_vedyne_button)
        stop_buttons_layout.addWidget(self.stop_record_button)
        stop_buttons_layout.addWidget(self.stop_all_nodes_button)


        # Create and add a QListWidget to display ROS topics
        self.topic_list_widget = QListWidget()
    
        
        # Create logo label and add to the top-right corner
        logo_label = QLabel()
        logo_pixmap = QPixmap('assets/GahanAI_Logo.png')  # Replace with the path to your logo file
        logo_pixmap = logo_pixmap.scaled(110, 110, Qt.KeepAspectRatio)  # Resize the logo
        logo_label.setPixmap(logo_pixmap)
        # logo_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        # Create rectangle layout for the logo
        rectangle_layout = QVBoxLayout()
        rectangle_layout.addWidget(logo_label)

        

        # Set layout for the main window
        self.setLayout(main_layout)

        main_layout.addWidget(logo_label)

        ## Arrange Layout
        # Add rectangle layout to main layout
        main_layout.addLayout(rectangle_layout)

        # Add start buttons layout to main layout
        main_layout.addLayout(start_buttons_layout)

        # Add stop buttons layout to main layout
        main_layout.addLayout(stop_buttons_layout)

        # Add topic list to main layout
        main_layout.addWidget(self.topic_list_widget)

        # Set layout for the main window
        self.setLayout(main_layout)

        # Set window properties
        self.setWindowTitle('GAHAN ARAS RECORDER DASHBOARD')
        self.setGeometry(100, 100, 700, 500)

        
        # Create a timer to update the topic list every 2 seconds
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.populate_topic_list)
        self.timer.start(5000)  # Set the timer interval (in milliseconds), e.g., 5000ms for every 5 seconds


    def populate_topic_list(self):

        # Check if ros is running
        try:
            # Get all topics and add them to the QListWidget
            all_topics = subprocess.check_output(['rostopic', 'list']).decode('utf-8').split('\n')
        except subprocess.CalledProcessError:
            all_topics = ["Empty List: No ROS Topics Found"]

        
        self.topic_list_widget.clear()  # Clear the existing list
        for topic in all_topics:
            self.topic_list_widget.addItem(topic)


    def launch_camera(self):
        # Camera ROS launch file
        launch_file = './src/usb_cam/launch/usb_cam.launch'

        # Launch ROS node using subprocess
        self.run_command(['roslaunch', launch_file])

    def stop_camera(self):
        # Stop ROS node using subprocess
        self.run_command(['rosnode', 'kill', '/usb_cam'])  # Replace '/your_node_name' with your actual node name

    def launch_radar(self):
        # RADAR ROS launch file
        # launch_file = './src/ti_mmwave_rospkg/launch/1843es1_long_range.launch'
        launch_file = './src/ti_mmwave_rospkg/launch/1843_2d_mrr_furacam.launch'
        # launch_file = './src/ti_mmwave_rospkg/launch/1843_3d_lrr_furacam.launch'

        # Launch ROS node using subprocess
        self.run_command(['roslaunch', launch_file])

    def stop_radar(self):
        # Stop ROS node using subprocess
        self.run_command(['rosnode', 'kill', '/ti_mmwave'])
        self.run_command(['rosnode', 'kill', '/ti_mmwave_0'])

    def launch_vedyne(self):
        # Launch VeDyne using specified launch file
        launch_file = './src/vedyne_serial/launch/vedyne_serial_rviz.launch'

        # Launch ROS node using subprocess
        self.run_command(['roslaunch', launch_file])

    def stop_vedyne(self):
        # Stop VeDyne ROS node using subprocess
        self.run_command(['rosnode', 'kill', '/vedyne_serial'])

    def start_record(self):
        # Get all topics
        all_topics = subprocess.check_output(['rostopic', 'list']).decode('utf-8').split('\n')

        print(all_topics)

        # Exclude topics with "compressedDepth"
        filtered_topics = [topic for topic in all_topics if 'compressedDepth' not in topic]

        # Start rosbag recording for filtered topics
        self.rosbag_process = self.run_command_in_thread(['rosbag', 'record', '-o', self.recording_prefix] + filtered_topics)
        # self.rosbag_process = self.run_command_in_thread(['rosbag', 'record'] + filtered_topics)
        self.recording_indicator.setText('Recording ON')

        self.start_record_button.setEnabled(False)
        self.stop_record_button.setEnabled(True)

    def stop_record(self):
        # Stop rosbag recording using subprocess
        if self.rosbag_process is not None:
            self.rosbag_process.stop()
            self.recording_indicator.setText('Recording OFF')

            self.start_record_button.setEnabled(True)
            self.stop_record_button.setEnabled(False)

    def stop_all_nodes(self):
        # Stop all ROS nodes using subprocess
        subprocess.Popen(['rosnode', 'kill', '-a'])

    def run_command(self, command):
        subprocess.Popen(command)


    def run_command_in_thread(self, command):
        thread = RecordThread(command)
        thread.start()
        return thread

    def closeEvent(self, event):
        # Override close event to kill all active roslaunch processes before exiting
        subprocess.Popen(['pkill', '-f', 'roslaunch'])
        subprocess.Popen(['pkill', '-f', 'rosmaster'])
        subprocess.Popen(['pkill', '-f', 'usb_cam_node'])
        subprocess.Popen(['pkill', '-f', 'ti_mmwave_rospk'])
        subprocess.Popen(['pkill', '-f', 'vedyne_serial*'])
        subprocess.Popen(['rosnode', 'kill', '-a'])
        event.accept()

def crashEvent():
        # Override close event to kill all active roslaunch processes before exiting
        
        subprocess.Popen(['rosnode', 'kill', '-a'])

        subprocess.Popen(['pkill', '-f', 'roslaunch'])
        subprocess.Popen(['pkill', '-f', 'rosmaster'])
        subprocess.Popen(['pkill', '-f', 'usb_cam_node'])
        subprocess.Popen(['pkill', '-f', 'ti_mmwave_rospk'])
        subprocess.Popen(['pkill', '-f', 'vedyne_serial*'])

        sys.exit()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    try:
        window = ROSLauncherUI()
        window.show()
        sys.exit(app.exec_())
    except:
        print("Exiting")
        # crashEvent()

