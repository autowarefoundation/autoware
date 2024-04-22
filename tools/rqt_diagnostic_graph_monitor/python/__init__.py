# Copyright 2023 The Autoware Contributors
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

from rqt_gui_py.plugin import Plugin

from .module import MonitorModule
from .widget import MonitorWidget


class MonitorPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.widget = MonitorWidget()
        self.module = MonitorModule(context.node)
        self.module.append_struct_callback(self.widget.on_graph)
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        self.module.shutdown()
        self.widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
