#!/usr/bin/env python

import os
import sys
import signal
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QPushButton, QLabel, QTextEdit, QSpinBox
from python_qt_binding.QtCore import Qt, pyqtSignal
from python_qt_binding.QtGui import QPalette

from eurobench_bms_msgs_and_srvs.srv import *
from eurobench_bms_msgs_and_srvs.msg import *
from std_srvs.srv import Empty


class BenchmarkGui(Plugin):

    set_timer_signal = pyqtSignal(int)
    set_benchmark_info_signal = pyqtSignal(str)

    def __init__(self, context):
        super(BenchmarkGui, self).__init__(context)

        self.setObjectName('EUROBENCH Benchmark Control')

        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'benchmark_gui.ui')

        loadUi(ui_file, self._widget)

        self._widget.setObjectName('EUROBENCH Benchmark Control')
        self._widget.setWindowTitle('EUROBENCH Benchmark Control')

        self.robot_name_label = self._widget.findChild(QLabel, 'robot_name_label')
        self.benchmark_status_label = self._widget.findChild(QLabel, 'status_label')

        self.benchmark_name_label = self._widget.findChild(QLabel, 'benchmark_name_label')
        self.benchmark_name_label.setText(rospy.get_param('benchmark_group'))

        self.greenPalette = QPalette()
        self.greenPalette.setColor(self.benchmark_status_label.foregroundRole(), Qt.darkGreen)

        self.yellowPalette = QPalette()
        self.yellowPalette.setColor(self.benchmark_status_label.foregroundRole(), Qt.darkYellow)

        self.blackPalette = QPalette()
        self.blackPalette.setColor(self.benchmark_status_label.foregroundRole(), Qt.black)

        self.clock_textedit = self._widget.findChild(QTextEdit, 'clock_textedit')
        self.set_timer_signal.connect(self.timer_slot)

        self.results_textedit = self._widget.findChild(QTextEdit, 'results_textedit')
        self.set_benchmark_info_signal.connect(self.benchmark_info_slot)

        self.results_detail_label = self._widget.findChild(QLabel, 'results_detail_label')

        self.run_spinbox = self._widget.findChild(QSpinBox, 'run_spinbox')

        # Disable start button, set its listener
        self.start_button = self._widget.findChild(QPushButton, 'start_button')
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self.on_startbutton_click)

        self.stop_button = self._widget.findChild(QPushButton, 'stop_button')
        self.stop_button.clicked.connect(self.on_stopbutton_click)

        # Get robot names from BMS to fill the combo box
        get_robot_names = rospy.ServiceProxy('bmserver/robot_names', BenchmarkServerRobotNames)
        robot_names = get_robot_names()

        self.robot_combo = self._widget.findChild(QComboBox, 'robot_combo')
        self.robot_combo.addItems([''] + robot_names.robot_names)

        # Combobox listeners
        self.robot_combo.currentTextChanged.connect(self.on_combobox_change)

        # Subscribe to benchmark state
        rospy.Subscriber('bmserver/state', BenchmarkServerState, self.state_callback)

        context.add_widget(self._widget)

    def on_combobox_change(self, value):
        self.robot_name_label.setText(self.robot_combo.currentText())

    def on_startbutton_click(self):
        robot_name = self.robot_combo.currentText()

        run_number = self.run_spinbox.value()

        start_benchmark = rospy.ServiceProxy('bmserver/start_benchmark', StartBenchmark)

        start_request = StartBenchmarkRequest()
        start_request.robot_name = robot_name
        start_request.run_number = run_number

        start_benchmark(start_request)

    def on_stopbutton_click(self):
        stop_benchmark = rospy.ServiceProxy('bmserver/stop_benchmark', StopBenchmark)
        stop_benchmark()

    def state_callback(self, state):
        self.server_status = state.status

        # Set start button enabled/disabled
        startbutton_enabled = True

        if(self.robot_combo.currentText() == ''
        or state.status == BenchmarkServerState.RUNNING_BENCHMARK):
            startbutton_enabled = False

        self.start_button.setEnabled(startbutton_enabled)

        # If benchmark is running: set label, disable comboboxes
        if(state.status == BenchmarkServerState.RUNNING_BENCHMARK):
            self.benchmark_status_label.setText('Running benchmark')
            self.benchmark_status_label.setPalette(self.yellowPalette)

            self.robot_combo.setEnabled(False)
        else:
            self.benchmark_status_label.setText('Ready')
            self.benchmark_status_label.setPalette(self.greenPalette)

            self.robot_combo.setEnabled(True)

        # Set clock timer
        if self.server_status == BenchmarkServerState.READY:
            self.set_timer_signal.emit(0)
        else:
            self.set_timer_signal.emit(state.current_benchmark_seconds_passed)

        # Set results screen and detail label
        if self.server_status == BenchmarkServerState.RUNNING_BENCHMARK:
            self.set_benchmark_info_signal.emit(state.current_benchmark_info)
            self.results_detail_label.setPalette(self.yellowPalette)
            self.results_detail_label.setText('(benchmark running)')
        else:
            if state.last_benchmark_info:
                self.set_benchmark_info_signal.emit(state.last_benchmark_info)
                self.results_detail_label.setPalette(self.greenPalette)
                self.results_detail_label.setText('(benchmark finished)')

    def timer_slot(self, seconds_passed):
        minutes_passed = seconds_passed // 60
        seconds_passed = seconds_passed % 60
        self.clock_textedit.setText(' %02d:%02d' % (minutes_passed, seconds_passed))

    def benchmark_info_slot(self, info):
        if info:
            self.results_textedit.setText(info.replace('"', ''))

    def shutdown_plugin(self):
        shutdown_server = rospy.ServiceProxy('bmserver/shutdown', Empty)
        shutdown_server()

        rospy.signal_shutdown('Shutting down')

    def save_settings(self, plugin_settings, instance_settings):
        # Save intrinsic configuration
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore intrinsic configuration
        pass
