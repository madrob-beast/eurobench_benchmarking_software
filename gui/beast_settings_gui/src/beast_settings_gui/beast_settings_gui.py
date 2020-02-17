import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox

from std_msgs.msg import UInt16
from eurobench_bms_msgs_and_srvs.srv import *


class beast_settings_gui(Plugin):
    def __init__(self, context):
        super(beast_settings_gui, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('BEAST Settings')

        # Create QWidget
        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'beast_settings_gui.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('BEAST Settings')
        self._widget.setWindowTitle('BEAST Settings')

        self.trolley_stiffness_combo = self._widget.findChild(QComboBox, 'trolley_stiffness_combo')
        self.trolley_stiffness_combo.addItems(['0', '1', '2'])

        context.add_widget(self._widget)

        self.run_rospy_node()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def run_rospy_node(self):
        self.benchmark_params_service = rospy.Service(
            'beast/gui/benchmark_params', BeastBenchmarkParams, self.benchmark_params_callback)

    def benchmark_params_callback(self, request):
        benchmark_params_response = BeastBenchmarkParamsResponse()
        benchmark_params_response.stiffness = int(self.trolley_stiffness_combo.currentText())

        return benchmark_params_response