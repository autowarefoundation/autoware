# Copyright 2024 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from diagnostic_msgs.msg import DiagnosticStatus
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

from .graph import BaseUnit
from .graph import UnitLink


class MonitorIcons:
    def __init__(self):
        self.disable = QtGui.QIcon.fromTheme("dialog-question")
        self.unknown = QtGui.QIcon.fromTheme("system-search")
        self.ok = QtGui.QIcon.fromTheme("emblem-default")
        self.warn = QtGui.QIcon.fromTheme("emblem-important")
        self.error = QtGui.QIcon.fromTheme("dialog-error")
        self.stale = QtGui.QIcon.fromTheme("appointment-missed")

        self._levels = {}
        self._levels[DiagnosticStatus.OK] = self.ok
        self._levels[DiagnosticStatus.WARN] = self.warn
        self._levels[DiagnosticStatus.ERROR] = self.error
        self._levels[DiagnosticStatus.STALE] = self.stale

    def get(self, level):
        return self._levels.get(level, self.unknown)


class MonitorItem:
    icons = MonitorIcons()

    def __init__(self, link: UnitLink, unit: BaseUnit):
        item_text = f"{unit.path} ({unit.kind})" if unit.path else f"({unit.kind})"
        self.item = QtWidgets.QTreeWidgetItem([item_text])
        self.link = link
        self.unit = unit
        self.item.setIcon(0, self.icons.stale)

    def update(self):
        self.item.setIcon(0, self.icons.get(self.unit.level))
