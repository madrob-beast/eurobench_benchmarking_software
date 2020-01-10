#!/usr/bin/env python

from os import path, makedirs
import sys
import signal
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QPushButton, QLabel, QTextEdit, QSpinBox, QFileDialog, QLineEdit
from python_qt_binding.QtCore import Qt, pyqtSignal
from python_qt_binding.QtGui import QPalette

from eurobench_bms_msgs_and_srvs.srv import *
from eurobench_bms_msgs_and_srvs.msg import *
from std_srvs.srv import Empty


class BenchmarkGui(Plugin):

    set_timer_signal = pyqtSignal(int, bool)
    set_benchmark_info_signal = pyqtSignal(str)

    def __init__(self, context):
        super(BenchmarkGui, self).__init__(context)

        self.setObjectName('EUROBENCH Benchmark Control')

        self._widget = QWidget()

        ui_file = path.join(path.dirname(path.realpath(__file__)),
                               'benchmark_gui.ui')

        loadUi(ui_file, self._widget)

        self._widget.setObjectName('EUROBENCH Benchmark Control')
        self._widget.setWindowTitle('EUROBENCH Benchmark Control')

        self.robot_name_label = self._widget.findChild(QLabel, 'robot_name_label')
        self.benchmark_status_label = self._widget.findChild(QLabel, 'status_label')

        self.benchmark_name_label = self._widget.findChild(QLabel, 'benchmark_name_label')
        self.benchmark_name_label.setText(rospy.get_param('benchmark_group'))

        self.clock_label = self._widget.findChild(QLabel, 'clock_label')

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

        self.rosbag_path_edit = self._widget.findChild(QLineEdit, 'rosbag_path_edit')
        self.rosbag_path_browse = self._widget.findChild(QPushButton, 'rosbag_path_browse')
        self.rosbag_path_browse.clicked.connect(self.on_rosbag_path_browse_click)

        self.testbed_yaml_edit = self._widget.findChild(QLineEdit, 'testbed_yaml_edit')
        self.testbed_yaml_browse = self._widget.findChild(QPushButton, 'testbed_yaml_browse')
        self.testbed_yaml_browse.clicked.connect(self.on_testbed_yaml_browse_click)

        # Disable start buttons, set their listeners
        self.start_button = self._widget.findChild(QPushButton, 'start_button')
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self.on_startbutton_click)

        self.start_rosbag_button = self._widget.findChild(QPushButton, 'start_from_bag_button')
        self.start_rosbag_button.setEnabled(False)
        self.start_rosbag_button.clicked.connect(self.on_rosbagbutton_click)

        self.stop_button = self._widget.findChild(QPushButton, 'stop_button')
        self.stop_button.clicked.connect(self.on_stopbutton_click)

        # Get robot names from BMS to fill the combo box
        get_robot_names = rospy.ServiceProxy('bmcore/robot_names', BenchmarkCoreRobotNames)
        robot_names = get_robot_names()

        self.robot_combo = self._widget.findChild(QComboBox, 'robot_combo')
        self.robot_combo.addItems([''] + robot_names.robot_names)

        # Combobox listeners
        self.robot_combo.currentTextChanged.connect(self.on_combobox_change)

        # Subscribe to benchmark state
        rospy.Subscriber('bmcore/state', BenchmarkCoreState, self.state_callback)

        context.add_widget(self._widget)

    def on_combobox_change(self, value):
        self.robot_name_label.setText(self.robot_combo.currentText())

    def on_startbutton_click(self):
        robot_name = self.robot_combo.currentText()

        run_number = self.run_spinbox.value()

        start_benchmark = rospy.ServiceProxy('bmcore/start_benchmark', StartBenchmark)

        start_request = StartBenchmarkRequest()
        start_request.robot_name = robot_name
        start_request.run_number = run_number
        start_request.use_rosbag = False

        start_benchmark(start_request)

    def on_rosbagbutton_click(self):
        rosbag_path = self.rosbag_path_edit.text()
        testbed_conf_path = self.testbed_yaml_edit.text()

        if (not rosbag_path) or (not testbed_conf_path):
            rospy.logerr('Error: Missing rosbag or yaml path.')
            return

        robot_name = self.robot_combo.currentText()

        run_number = self.run_spinbox.value()

        start_benchmark = rospy.ServiceProxy('bmcore/start_benchmark', StartBenchmark)

        start_request = StartBenchmarkRequest()
        start_request.robot_name = robot_name
        start_request.run_number = run_number
        start_request.use_rosbag = True
        start_request.rosbag_path = rosbag_path
        start_request.testbed_conf_path = testbed_conf_path

        start_benchmark(start_request)

    def on_rosbag_path_browse_click(self):
         # Show file dialog to choose rosbag
        rosbag_dir = self.get_cached_dir('rosbag_dir') # Read cached rosbag directory
        rosbag_filepath, _ = QFileDialog.getOpenFileName(self._widget, 'Open rosbag file', rosbag_dir, 'Rosbag Files (*.bag)')
        if not rosbag_filepath:
            return

        self.set_cached_dir('rosbag_dir', path.dirname(rosbag_filepath)) # Cache rosbag directory for next time

        self.rosbag_path_edit.setText(rosbag_filepath)

    def on_testbed_yaml_browse_click(self):
        # Show file dialog to choose testbed configuration yaml
        yaml_dir = self.get_cached_dir('yaml_dir') # Read cached rosbag directory
        yaml_filepath, _ = QFileDialog.getOpenFileName(self._widget, 'Open yaml file', yaml_dir, 'YAML Files (*.yaml)')
        if not yaml_filepath:
            return

        self.set_cached_dir('yaml_dir', path.dirname(yaml_filepath)) # Cache rosbag directory for next time

        self.testbed_yaml_edit.setText(yaml_filepath)

    def on_stopbutton_click(self):
        stop_benchmark = rospy.ServiceProxy('bmcore/stop_benchmark', StopBenchmark)
        stop_benchmark()

    def state_callback(self, state):
        self.core_status = state.status

        # Set start button enabled/disabled
        startbutton_enabled = True

        if(self.robot_combo.currentText() == ''
        or state.status == BenchmarkCoreState.RUNNING_BENCHMARK):
            startbutton_enabled = False

        self.start_button.setEnabled(startbutton_enabled)
        self.start_rosbag_button.setEnabled(startbutton_enabled)

        # If benchmark is running: set label, disable comboboxes
        if(state.status == BenchmarkCoreState.RUNNING_BENCHMARK):
            self.benchmark_status_label.setText('Running benchmark')
            self.benchmark_status_label.setPalette(self.yellowPalette)

            self.robot_combo.setEnabled(False)
        else:
            self.benchmark_status_label.setText('Ready')
            self.benchmark_status_label.setPalette(self.greenPalette)

            self.robot_combo.setEnabled(True)

        # Set clock timer
        self.set_timer_signal.emit(state.current_benchmark_seconds_passed, self.core_status == BenchmarkCoreState.RUNNING_BENCHMARK)

        # Set results screen and detail label
        if self.core_status == BenchmarkCoreState.RUNNING_BENCHMARK:
            self.set_benchmark_info_signal.emit(state.current_benchmark_info)
            self.results_detail_label.setPalette(self.yellowPalette)
            self.results_detail_label.setText('(benchmark running)')
        else:
            if state.last_benchmark_info:
                self.set_benchmark_info_signal.emit(state.last_benchmark_info)
                self.results_detail_label.setPalette(self.greenPalette)
                self.results_detail_label.setText('(benchmark finished)')

    def timer_slot(self, seconds_passed, benchmark_running):
        if not benchmark_running:
            self.clock_label.setPalette(self.blackPalette)
            self.clock_label.setText('Benchmark Time')
            self.clock_textedit.setTextColor(Qt.gray)
            seconds_passed = abs(seconds_passed)
        else:
            if seconds_passed < 0:
                self.clock_label.setPalette(self.yellowPalette)
                self.clock_label.setText('Benchmark starting')
                self.clock_textedit.setTextColor(Qt.darkYellow)
                seconds_passed = -seconds_passed
            else:
                self.clock_label.setPalette(self.blackPalette)
                self.clock_label.setText('Benchmark running')
                self.clock_textedit.setTextColor(Qt.black)
        
        minutes_passed = seconds_passed // 60
        seconds_passed = seconds_passed % 60
        self.clock_textedit.setText(' %02d:%02d' % (minutes_passed, seconds_passed))

    def benchmark_info_slot(self, info):
        if info:
            self.results_textedit.setText(info.replace('"', ''))

    def shutdown_plugin(self):
        shutdown_core = rospy.ServiceProxy('bmcore/shutdown', Empty)
        shutdown_core()

        rospy.signal_shutdown('Shutting down')

    def save_settings(self, plugin_settings, instance_settings):
        # Save intrinsic configuration
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore intrinsic configuration
        pass

    def get_cached_dir(self, dir_type):
        output_dir = path.expanduser(rospy.get_param('benchmark_output_directory'))
        if path.exists(path.join(output_dir, 'app_config', dir_type)):
            with open(path.join(output_dir, 'app_config', dir_type)) as file:
                return path.expanduser(file.read().strip())
        else:
            return '/home/'

    def set_cached_dir(self, dir_type, dir_path):
        output_dir = path.expanduser(rospy.get_param('benchmark_output_directory'))
        app_config = path.join(output_dir, 'app_config')
        if not path.exists(app_config):
            makedirs(app_config)

        with open(path.join(app_config, dir_type), 'w+') as file:
            file.write(dir_path)