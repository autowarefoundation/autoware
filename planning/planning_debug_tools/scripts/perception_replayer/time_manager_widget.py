#!/usr/bin/env python3

# Copyright 2023 TIER IV, Inc.
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

from PyQt5 import QtCore
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtWidgets import QSlider
from PyQt5.QtWidgets import QWidget


# With QSlider, the slider's handle cannot be captured if the mouse cursor is not the handle position when pressing the mouse.
class QJumpSlider(QSlider):
    def __init__(self, slider_direction, max_value):
        super(self.__class__, self).__init__(slider_direction)

        self.max_value = max_value
        self.is_mouse_pressed = False

    def mouse_to_value(self, event):
        x = event.pos().x()
        return int(self.max_value * x / self.width())

    def mousePressEvent(self, event):
        super(self.__class__, self).mousePressEvent(event)

        if event.button() == QtCore.Qt.LeftButton:
            self.setValue(self.mouse_to_value(event))
            self.is_mouse_pressed = True

    def mouseMoveEvent(self, event):
        super(self.__class__, self).mouseMoveEvent(event)
        if self.is_mouse_pressed:
            self.setValue(self.mouse_to_value(event))

    def mouseReleaseEvent(self, event):
        super(self.__class__, self).mouseReleaseEvent(event)

        if event.button() == QtCore.Qt.LeftButton:
            self.is_mouse_pressed = False


class TimeManagerWidget(QMainWindow):
    def __init__(self, start_timestamp, end_timestamp):
        super(self.__class__, self).__init__()

        self.start_timestamp = start_timestamp
        self.end_timestamp = end_timestamp
        self.max_value = 1000000

        self.setupUI()

    def setupUI(self):
        self.setObjectName("PerceptionReplayer")
        self.resize(480, 120)
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)

        self.central_widget = QWidget(self)
        self.central_widget.setObjectName("central_widget")

        self.grid_layout = QGridLayout(self.central_widget)
        self.grid_layout.setContentsMargins(10, 10, 10, 10)
        self.grid_layout.setObjectName("grid_layout")

        # rate button
        self.rate_button = []
        for i, rate in enumerate([0.1, 0.5, 1.0, 2.0, 5.0, 10.0]):
            button = QPushButton(str(rate))
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.rate_button.append(button)
            self.grid_layout.addWidget(self.rate_button[-1], 0, i, 1, 1)

        # pause button
        self.button = QPushButton("pause")
        self.button.setCheckable(True)
        self.button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.grid_layout.addWidget(self.button, 1, 0, 1, -1)
        self.pub_recorded_ego_pose_button = QPushButton("publish recorded ego pose")
        self.grid_layout.addWidget(self.pub_recorded_ego_pose_button, 2, 0, 1, -1)

        # slider
        self.slider = QJumpSlider(QtCore.Qt.Horizontal, self.max_value)
        self.slider.setMinimum(0)
        self.slider.setMaximum(self.max_value)
        self.slider.setValue(0)
        self.grid_layout.addWidget(self.slider, 3, 0, 1, -1)

        self.setCentralWidget(self.central_widget)

    def timestamp_to_value(self, timestamp):
        return int(
            (timestamp - self.start_timestamp)
            / (self.end_timestamp - self.start_timestamp)
            * self.max_value
        )

    def value_to_timestamp(self, value):
        return self.start_timestamp + self.slider.value() / self.max_value * (
            self.end_timestamp - self.start_timestamp
        )
