import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QComboBox, QGroupBox, QPushButton, QPlainTextEdit

from std_msgs.msg import String
from eurobench_benchmark_server.srv import *


class madrob_settings_gui(Plugin):
    def __init__(self, context):
        super(madrob_settings_gui, self).__init__(context)

        self.setObjectName('MADROB Settings')

        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'madrob_settings_gui.ui')

        loadUi(ui_file, self._widget)

        self.showing_calibration = False
        
        self._widget.setObjectName('MADROB Settings')
        self._widget.setWindowTitle('MADROB Settings')

        self.benchmark_type_combo = self._widget.findChild(QComboBox, 'benchmark_type_combo')

        self.calibration_button = self._widget.findChild(QPushButton, 'show_hide_calib')
        self.calibration_button.clicked.connect(self.toggle_calibration_menu)

        self.calibration_menu = self._widget.findChild(QGroupBox, 'calib_groupbox')
        self.calibration_menu.hide()

        self.calib_loadcell_cw_button = self._widget.findChild(QPushButton, 'calib_loadcell_cw_button')
        self.calib_loadcell_cw_button.clicked.connect(self.calibrate_loadcell_cw)

        self.calib_loadcell_ccw_button = self._widget.findChild(QPushButton, 'calib_loadcell_ccw_button')
        self.calib_loadcell_ccw_button.clicked.connect(self.calibrate_loadcell_ccw)

        self.loadcell_weight_textedit = self._widget.findChild(QPlainTextEdit, 'calib_weight_textedit')

        self.calib_motor_start_button = self._widget.findChild(QPushButton, 'calib_motor_start')
        self.calib_motor_start_button.clicked.connect(self.start_motor_calibration)

        self.calib_encoder_cw_button = self._widget.findChild(QPushButton, 'calib_encoder_cw')
        self.calib_encoder_cw_button.clicked.connect(self.calibrate_encoder_cw)

        self.calib_encoder_closed_button = self._widget.findChild(QPushButton, 'calib_encoder_closed')
        self.calib_encoder_closed_button.clicked.connect(self.calibrate_encoder_closed)

        self.calib_encoder_ccw_button = self._widget.findChild(QPushButton, 'calib_encoder_ccw')
        self.calib_encoder_ccw_button.clicked.connect(self.calibrate_encoder_ccw)

        get_madrob_settings = rospy.ServiceProxy('madrob/settings', MadrobSettings)
        madrob_settings = get_madrob_settings()

        self.benchmark_type_combo.addItems(madrob_settings.benchmark_types)

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

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget
        # title bar
        # Usually used to open a modal configuration dialog

    def toggle_calibration_menu(self):
        if self.showing_calibration:
            self.calibration_menu.hide()
            self.calibration_button.setText('Show calibration menu')
        else:
            self.calibration_menu.show()
            self.calibration_button.setText('Hide calibration menu')
        self.showing_calibration = not self.showing_calibration

    def calibrate_loadcell_cw(self):
        try:
            weight = -int(round(float(self.loadcell_weight_textedit.toPlainText())))
        except:
            print('Error parsing integer for "weight": no calibration done.')
            return

        # TODO call service 'rosservice call madrob/handle/calibrate sample 160 step 0 force -<weight>'

    def calibrate_loadcell_ccw(self):
        try:
            weight = int(round(float(self.loadcell_weight_textedit.toPlainText())))
        except:
            print('Error parsing integer for "weight": no calibration done.')
            return

        # TODO call service 'rosservice call madrob/handle/calibrate sample 160 step 1 force <weight>'

    def start_motor_calibration(self):
        # TODO start motor calibration procedure
        pass

    def calibrate_encoder_closed(self):
        # TODO rosservice call madrob/door/calibrate_position 0
        pass

    def calibrate_encoder_cw(self):
        # TODO rosservice call madrob/door/calibrate_position 1
        pass
    
    def calibrate_encoder_ccw(self):
        # TODO rosservice call madrob/door/calibrate_position 2
        pass

    def run_rospy_node(self):
        self.benchmark_type_service = rospy.Service(
            'madrob/gui/benchmark_type', MadrobBenchmarkType, self.benchmark_type_callback)

    def benchmark_type_callback(self, request):
        benchmark_type_response = MadrobBenchmarkTypeResponse()
        benchmark_type_response.benchmark_type = self.benchmark_type_combo.currentText()

        return benchmark_type_response