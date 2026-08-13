import sys
import rclpy
import numpy as np
from rclpy.node import Node

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QGridLayout,
    QPushButton, QVBoxLayout,
    QHBoxLayout,
    QGroupBox)
from PySide6.QtCore import QTimer


from sensor_msgs.msg import JointState


class FR3Gui(QWidget):

    def __init__(self):
        super().__init__()
        rclpy.init()

        self.ros_node = rclpy.create_node("fr3_gui")
        self.ros_node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)

        self.setWindowTitle("FR3 Control")

        self.resize(600, 500)
        self.create_joint_table()
        self.apply_style()

        self.move_button.clicked.connect(self.move_robot)

    def joint_state_callback(self, msg):

        joint_positions = dict(zip(msg.name, msg.position))

        for i in range(7):

            joint_name = f"joint{i + 1}"

            if joint_name in joint_positions:
                angle = joint_positions[joint_name]

                self.current_labels[i].setText(
                    f"{angle:.4f}"
                )

    def spin_ros(self):

        rclpy.spin_once(
            self.ros_node,
            timeout_sec=0
        )

    def create_joint_table(self):

        title = QLabel("FR3 Control")
        title.setObjectName("title")
        layout = QGridLayout()

        layout.addWidget(QLabel("Joint"), 0, 0)
        layout.addWidget(QLabel("Current"), 0, 1)
        layout.addWidget(QLabel("Target"), 0, 2)

        self.target_inputs = []
        self.current_labels = []

        for i in range(7):

            joint_label = QLabel(f"Joint {i+1}")
            current_angle = QLabel("0.0")
            target_angle = QLineEdit()
            target_angle.setPlaceholderText("rad")
            self.target_inputs.append(target_angle)

            self.current_labels.append(current_angle)

            layout.addWidget(joint_label, i+1, 0)
            layout.addWidget(current_angle, i+1, 1)
            layout.addWidget(target_angle, i+1, 2)

        layout.addWidget(QLabel("Total Time"), 8, 0)
        self.total_time_input = QLineEdit("10.0")
        layout.addWidget(self.total_time_input, 8, 1)

        layout.addWidget(QLabel("Time Step"), 9, 0)
        self.time_step = QLineEdit("0.001")
        layout.addWidget(self.time_step, 9, 1)

        self.move_button = QPushButton("MOVE")
        layout.addWidget(self.move_button, 10, 1)

        self.setLayout(layout)

    def move_robot(self):

        target_angles = []

        for input_value in self.target_inputs:
            angle = float(input_value.text())
            target_angles.append(angle)

        total_time = float(self.total_time_input.text())
        time_step = float(self.time_step.text())

        print("Target angles: ", target_angles)
        print("Total Time: ", total_time)
        print("Time Step: ", time_step)

    def apply_style(self):

        self.setStyleSheet("""
            QWidget {
                background-color: #1e1e1e;
                color: #e5e7eb;
                font-size: 14px;
            }

            QLineEdit {
                background-color: #2a2a2a;
                color: #ffffff;
                border: 1px solid #555555;
                border-radius: 5px;
                padding: 6px;
            }

            QLineEdit:focus {
                border: 1px solid #3b82f6;
            }

            QPushButton {
                background-color: #2563eb;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 8px 25px;
                font-weight: bold;
            }

            QPushButton:hover {
                background-color: #3b82f6;
            }

            QPushButton:pressed {
                background-color: #1d4ed8;
            }
        """)

    def closeEvent(self, event):

        self.ros_node.destroy_node()
        rclpy.shutdown()

        event.accept()


app = QApplication(sys.argv)

window = FR3Gui()
window.show()

sys.exit(app.exec())
