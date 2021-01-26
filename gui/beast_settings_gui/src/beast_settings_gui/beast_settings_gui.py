import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox

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

        self.disturbance_type_combo = self._widget.findChild(QComboBox, 'disturbance_type_combo')
        self.disturbance_type_combo.addItems(['No Force', 'Small Pet'])
        self.load_combo = self._widget.findChild(QComboBox, 'load_combo')
        self.load_combo.addItems(['1500'])
        self.start_already_gripping_combo = self._widget.findChild(QComboBox, 'start_already_gripping_combo')
        self.start_already_gripping_combo.addItems(['False', 'True'])

        context.add_widget(self._widget)

        self.benchmark_params_service = rospy.Service('beast/gui/benchmark_params', BeastBenchmarkParams, self.benchmark_params_callback)

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

    def benchmark_params_callback(self, _):
        benchmark_params_response = BeastBenchmarkParamsResponse()
        benchmark_params_response.disturbance_type = str(self.disturbance_type_combo.currentText())
        benchmark_params_response.load = float(self.load_combo.currentText())
        benchmark_params_response.start_already_gripping = self.start_already_gripping_combo.currentText() == 'True'

        return benchmark_params_response
