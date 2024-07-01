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
from PyQt5.QtWidgets import QGroupBox
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtWidgets import QSlider
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
from autoware_planning_msgs.msg import Trajectory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import Empty
from std_msgs.msg import Float32
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
        self.grid_layout = QVBoxLayout(central_widget)
        self.grid_layout.setContentsMargins(10, 10, 10, 10)

        # slide of the trajectory's start and end index
        self.traj_start_index_slider = QSlider(QtCore.Qt.Horizontal)
        self.traj_end_index_slider = QSlider(QtCore.Qt.Horizontal)
        self.min_traj_start_index = 0
        self.max_traj_end_index = None

        # Layout: Range of Centerline
        centerline_vertical_box = QVBoxLayout(self)
        centerline_vertical_box.addWidget(self.traj_start_index_slider)
        centerline_vertical_box.addWidget(self.traj_end_index_slider)
        centerline_group = QGroupBox("Centerline")
        centerline_group.setLayout(centerline_vertical_box)
        self.grid_layout.addWidget(centerline_group)

        """
        # 2. Road Boundary
        road_boundary_group = QGroupBox("Road Boundary")
        road_boundary_vertical_box = QVBoxLayout(self)
        road_boundary_group.setLayout(road_boundary_vertical_box)
        self.grid_layout.addWidget(road_boundary_group)

        # 2.1. Slider
        self.road_boundary_lateral_margin_slider = QSlider(QtCore.Qt.Horizontal)
        road_boundary_vertical_box.addWidget(self.road_boundary_lateral_margin_slider)
        self.road_boundary_lateral_margin_ratio = 10
        self.road_boundary_lateral_margin_slider.setMinimum(0)
        self.road_boundary_lateral_margin_slider.setMaximum(
            5 * self.road_boundary_lateral_margin_ratio
        )
        road_boundary_vertical_box.addWidget(QPushButton("reset"))
        """

        # 3. General
        general_group = QGroupBox("General")
        general_vertical_box = QVBoxLayout(self)
        general_group.setLayout(general_vertical_box)
        self.grid_layout.addWidget(general_group)

        # 3.1. Validate Centerline
        self.validate_button = QPushButton("validate")
        self.validate_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        general_vertical_box.addWidget(self.validate_button)

        # 3.2. Save Map
        self.save_map_button = QPushButton("save map")
        self.save_map_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        general_vertical_box.addWidget(self.save_map_button)

        self.setCentralWidget(central_widget)

    def initWithEndIndex(self, max_traj_end_index):
        self.max_traj_end_index = max_traj_end_index

        # initialize sliders
        self.traj_start_index_slider.setMinimum(self.min_traj_start_index)
        self.traj_start_index_slider.setMaximum(self.max_traj_end_index)
        self.traj_start_index_slider.setValue(self.min_traj_start_index)

        self.traj_end_index_slider.setMinimum(self.min_traj_start_index)
        self.traj_end_index_slider.setMaximum(self.max_traj_end_index)
        self.traj_end_index_slider.setValue(self.max_traj_end_index)


class CenterlineUpdaterHelper(Node):
    def __init__(self):
        super().__init__("centerline_updater_helper")
        # Qt
        self.widget = CenterlineUpdaterWidget()
        self.widget.show()
        self.widget.save_map_button.clicked.connect(self.onSaveMapButtonPushed)
        self.widget.validate_button.clicked.connect(self.onValidateButtonPushed)
        self.widget.traj_start_index_slider.valueChanged.connect(self.onStartIndexChanged)
        self.widget.traj_end_index_slider.valueChanged.connect(self.onEndIndexChanged)
        """
        self.widget.road_boundary_lateral_margin_slider.valueChanged.connect(
            self.onRoadBoundaryLateralMargin
        )
        """

        # ROS
        self.pub_save_map = self.create_publisher(Empty, "/static_centerline_generator/save_map", 1)
        self.pub_validate = self.create_publisher(Empty, "/static_centerline_generator/validate", 1)
        self.pub_traj_start_index = self.create_publisher(
            Int32, "/static_centerline_generator/traj_start_index", 1
        )
        self.pub_traj_end_index = self.create_publisher(
            Int32, "/static_centerline_generator/traj_end_index", 1
        )
        self.pub_road_boundary_lateral_margin = self.create_publisher(
            Float32, "/static_centerline_generator/road_boundary_lateral_margin", 1
        )

        transient_local_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_whole_centerline = self.create_subscription(
            Trajectory,
            "/static_centerline_generator/output/whole_centerline",
            self.onWholeCenterline,
            transient_local_qos,
        )

        while self.widget.max_traj_end_index is None:
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.1)

    def onWholeCenterline(self, whole_centerline):
        self.widget.initWithEndIndex(len(whole_centerline.points) - 1)

    def onSaveMapButtonPushed(self, event):
        msg = Empty()
        self.pub_save_map.publish(msg)

        # NOTE: After saving the map, the generated centerline is written
        # in original_map_ptr_ in static_centerline_generator_node.
        # When saving the map again, another centerline is written without
        # removing the previous centerline.
        # Therefore, saving the map can be called only once.
        self.widget.save_map_button.setEnabled(False)

    def onValidateButtonPushed(self, event):
        msg = Empty()
        self.pub_validate.publish(msg)

    def onStartIndexChanged(self, event):
        msg = Int32()
        msg.data = self.widget.traj_start_index_slider.value()
        self.pub_traj_start_index.publish(msg)

    def onEndIndexChanged(self, event):
        msg = Int32()
        msg.data = self.widget.traj_end_index_slider.value()
        self.pub_traj_end_index.publish(msg)

    def onRoadBoundaryLateralMargin(self, event):
        msg = Float32()
        msg.data = (
            self.widget.road_boundary_lateral_margin_slider.value()
            / self.widget.road_boundary_lateral_margin_ratio
        )
        self.pub_road_boundary_lateral_margin.publish(msg)


def main(args=None):
    app = QApplication(sys.argv)

    rclpy.init(args=args)
    node = CenterlineUpdaterHelper()

    while True:
        app.processEvents()
        rclpy.spin_once(node, timeout_sec=0.01)


if __name__ == "__main__":
    main()
