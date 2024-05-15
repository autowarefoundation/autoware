#!/bin/env python3

# Copyright 2024 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sys
import time

from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtWidgets import QSlider
from PyQt5.QtWidgets import QWidget
from autoware_auto_planning_msgs.msg import Trajectory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool
from std_msgs.msg import Int32


class CenterlineUpdaterWidget(QMainWindow):
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUI()

    def setupUI(self):
        self.setObjectName("MainWindow")
        self.resize(480, 120)
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)

        central_widget = QWidget(self)
        central_widget.setObjectName("central_widget")

        self.grid_layout = QGridLayout(central_widget)
        self.grid_layout.setContentsMargins(10, 10, 10, 10)
        self.grid_layout.setObjectName("grid_layout")

        # button to update the trajectory's start and end index
        self.update_button = QPushButton("update slider")
        self.update_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.update_button.clicked.connect(self.onUpdateButton)

        # button to reset the trajectory's start and end index
        self.reset_button = QPushButton("reset")
        self.reset_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.reset_button.clicked.connect(self.onResetButton)

        # button to save map
        self.save_map_button = QPushButton("save map")
        self.save_map_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # slide of the trajectory's start and end index
        self.traj_start_index_slider = QSlider(QtCore.Qt.Horizontal)
        self.traj_end_index_slider = QSlider(QtCore.Qt.Horizontal)
        self.min_traj_start_index = 0
        self.max_traj_end_index = None

        # set layout
        self.grid_layout.addWidget(self.update_button, 1, 0, 1, -1)
        self.grid_layout.addWidget(self.reset_button, 2, 0, 1, -1)
        self.grid_layout.addWidget(self.save_map_button, 3, 0, 1, -1)
        self.grid_layout.addWidget(self.traj_start_index_slider, 4, 0, 1, -1)
        self.grid_layout.addWidget(self.traj_end_index_slider, 5, 0, 1, -1)
        self.setCentralWidget(central_widget)

    def initWithEndIndex(self, max_traj_end_index):
        self.max_traj_end_index = max_traj_end_index

        # initialize slider
        self.displayed_min_traj_start_index = self.min_traj_start_index
        self.displayed_max_traj_end_index = self.max_traj_end_index
        self.initializeSlider()

    def initializeSlider(self, move_value_to_end=True):
        self.traj_start_index_slider.setMinimum(0)
        self.traj_end_index_slider.setMinimum(0)
        self.traj_start_index_slider.setMaximum(
            self.displayed_max_traj_end_index - self.displayed_min_traj_start_index
        )
        self.traj_end_index_slider.setMaximum(
            self.displayed_max_traj_end_index - self.displayed_min_traj_start_index
        )

        if move_value_to_end:
            self.traj_start_index_slider.setValue(0)
            self.traj_end_index_slider.setValue(self.traj_end_index_slider.maximum())

    def onResetButton(self, event):
        current_traj_start_index = self.displayed_min_traj_start_index
        current_traj_end_index = self.displayed_max_traj_end_index

        self.displayed_min_traj_start_index = self.min_traj_start_index
        self.displayed_max_traj_end_index = self.max_traj_end_index

        self.initializeSlider(False)
        self.traj_start_index_slider.setValue(current_traj_start_index)
        if (
            current_traj_start_index == self.min_traj_start_index
            and current_traj_end_index == self.max_traj_end_index
        ):
            self.traj_end_index_slider.setValue(self.displayed_max_traj_end_index)
        else:
            self.traj_end_index_slider.setValue(
                current_traj_start_index + self.traj_end_index_slider.value()
            )

    def onUpdateButton(self, event):
        current_traj_start_index = self.getCurrentStartIndex()
        current_traj_end_index = self.getCurrentEndIndex()

        self.displayed_min_traj_start_index = current_traj_start_index
        self.displayed_max_traj_end_index = current_traj_end_index

        self.initializeSlider()

    def getCurrentStartIndex(self):
        return self.displayed_min_traj_start_index + self.traj_start_index_slider.value()

    def getCurrentEndIndex(self):
        return self.displayed_min_traj_start_index + self.traj_end_index_slider.value()


class CenterlineUpdaterHelper(Node):
    def __init__(self):
        super().__init__("centerline_updater_helper")
        # Qt
        self.widget = CenterlineUpdaterWidget()
        self.widget.show()
        self.widget.save_map_button.clicked.connect(self.onSaveMapButtonPushed)
        self.widget.traj_start_index_slider.valueChanged.connect(self.onStartIndexChanged)
        self.widget.traj_end_index_slider.valueChanged.connect(self.onEndIndexChanged)

        # ROS
        self.pub_save_map = self.create_publisher(Bool, "~/save_map", 1)
        self.pub_traj_start_index = self.create_publisher(Int32, "~/traj_start_index", 1)
        self.pub_traj_end_index = self.create_publisher(Int32, "~/traj_end_index", 1)

        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_whole_centerline = self.create_subscription(
            Trajectory,
            "/autoware_static_centerline_generator/output_whole_centerline",
            self.onWholeCenterline,
            transient_local_qos,
        )

        while self.widget.max_traj_end_index is None:
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.1)

    def onWholeCenterline(self, whole_centerline):
        self.widget.initWithEndIndex(len(whole_centerline.points) - 1)

    def onSaveMapButtonPushed(self, event):
        msg = Bool()
        msg.data = True
        self.pub_save_map.publish(msg)

    def onStartIndexChanged(self, event):
        msg = Int32()
        msg.data = self.widget.getCurrentStartIndex()
        self.pub_traj_start_index.publish(msg)

    def onEndIndexChanged(self, event):
        msg = Int32()
        msg.data = self.widget.getCurrentEndIndex()
        self.pub_traj_end_index.publish(msg)


def main(args=None):
    app = QApplication(sys.argv)

    rclpy.init(args=args)
    node = CenterlineUpdaterHelper()

    while True:
        app.processEvents()
        rclpy.spin_once(node, timeout_sec=0.01)


if __name__ == "__main__":
    main()
