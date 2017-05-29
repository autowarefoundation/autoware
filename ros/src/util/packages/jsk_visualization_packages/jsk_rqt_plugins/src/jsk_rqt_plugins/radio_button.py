from rqt_gui_py.plugin import Plugin
from jsk_rqt_plugins.button_general import ServiceButtonGeneralWidget

class ServiceRadioButtons(Plugin):
    """
    rqt class to provide multiple buttons
    """
    def __init__(self, context):
        super(ServiceRadioButtons, self).__init__(context)
        self.setObjectName("ServiceRadioButtons")
        self._widget = ServiceRadioButtonWidget()
        context.add_widget(self._widget)
    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)
    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)
    def trigger_configuration(self):
        self._widget.trigger_configuration()

class ServiceRadioButtonWidget(ServiceButtonGeneralWidget):
    """
    Qt widget to visualize multiple radio buttons
    """
    def __init__(self):
        super(ServiceRadioButtonWidget, self).__init__(button_type = "radio")
