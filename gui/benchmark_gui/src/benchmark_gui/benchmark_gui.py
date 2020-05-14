#!/usr/bin/env python

from os import path, makedirs
import sys
import signal
import yaml
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

        self.benchmark_group = rospy.get_param('benchmark_group')

        self.benchmark_name_label = self._widget.findChild(QLabel, 'benchmark_name_label')
        self.benchmark_name_label.setText(self.benchmark_group)

        self.core_status_label = self._widget.findChild(QLabel, 'core_status_label')
        self.rosbag_controller_status_label = self._widget.findChild(QLabel, 'rosbag_controller_status_label')
        self.testbed_status_label = self._widget.findChild(QLabel, 'testbed_status_label')

        self.clock_label = self._widget.findChild(QLabel, 'clock_label')

        self.greenPalette = QPalette()
        self.greenPalette.setColor(self.benchmark_status_label.foregroundRole(), Qt.darkGreen)

        self.yellowPalette = QPalette()
        self.yellowPalette.setColor(self.benchmark_status_label.foregroundRole(), Qt.darkYellow)

        self.redPalette = QPalette()
        self.redPalette.setColor(self.benchmark_status_label.foregroundRole(), Qt.red)

        self.blackPalette = QPalette()
        self.blackPalette.setColor(self.benchmark_status_label.foregroundRole(), Qt.black)

        self.clock_textedit = self._widget.findChild(QTextEdit, 'clock_textedit')
        self.set_timer_signal.connect(self.timer_slot)

        self.results_textedit = self._widget.findChild(QTextEdit, 'results_textedit')
        self.set_benchmark_info_signal.connect(self.benchmark_info_slot)

        self.results_detail_label = self._widget.findChild(QLabel, 'results_detail_label')

        self.run_spinbox = self._widget.findChild(QSpinBox, 'run_spinbox')

        self.restart_core_and_rosbag_button = self._widget.findChild(QPushButton, 'restart_core_and_rosbag_button')
        self.restart_core_and_rosbag_button.clicked.connect(self.shutdown_core_and_rosbag_controller)

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

        self.robot_combo = self._widget.findChild(QComboBox, 'robot_combo')

        # Get robot names from BMS to fill the combo box
        self.robot_names_set = False
        rospy.Timer(rospy.Duration(1), self.check_robot_names)

        # Combobox listeners
        self.robot_combo.currentTextChanged.connect(self.on_combobox_change)

        # Subscribe to benchmark state
        rospy.Subscriber('bmcore/state', BenchmarkCoreState, self.state_callback)

        # Status of required nodes
        self.benchmark_core_available = False
        self.rosbag_controller_available = False
        self.testbed_node_available = False

        # Run heartbeat checks for required nodes (every second)
        rospy.Timer(rospy.Duration(1), self.check_required_nodes)

        context.add_widget(self._widget)

    def check_robot_names(self, _):
        if not self.robot_names_set:
            try:
                get_robot_names = rospy.ServiceProxy('bmcore/robot_names', BenchmarkCoreRobotNames)
                robot_names = get_robot_names()
                self.robot_combo.addItems([''] + robot_names.robot_names)
                self.robot_names_set = True
            except rospy.ServiceException:
                pass # Core not available yet

    def on_combobox_change(self, value):
        self.robot_name_label.setText(self.robot_combo.currentText())

    def on_startbutton_click(self):
        robot_name = self.robot_combo.currentText()

        run_number = self.run_spinbox.value()

        try:
            start_benchmark = rospy.ServiceProxy('bmcore/start_benchmark', StartBenchmark)

            start_request = StartBenchmarkRequest()
            start_request.live_benchmark = True
            start_request.robot_name = robot_name
            start_request.run_number = run_number

            start_benchmark(start_request)
        except rospy.ServiceException as e:
            rospy.logerr("bmcore/start_benchmark couldn't be called: {ex_val}".format(ex_val=str(e)))

    def on_rosbagbutton_click(self):
        testbed_conf_path = self.testbed_yaml_edit.text()

        if not testbed_conf_path:
            rospy.logerr('Error: Missing yaml path.')
            return

        try:
            with open(testbed_conf_path, 'r') as testbed_conf_file:
                testbed_conf = yaml.load(testbed_conf_file, Loader=yaml.FullLoader)

            start_benchmark = rospy.ServiceProxy('bmcore/start_benchmark', StartBenchmark)

            start_request = StartBenchmarkRequest()
            start_request.live_benchmark = False
            start_request.testbed_conf = yaml.dump(testbed_conf)

            start_benchmark(start_request)
        except rospy.ServiceException as e:
            rospy.logerr("bmcore/start_benchmark couldn't be called: {ex_val}".format(ex_val=str(e)))

    def on_testbed_yaml_browse_click(self):
        # Show file dialog to choose testbed configuration yaml
        yaml_dir = self.get_cached_dir('yaml_dir') # Read cached rosbag directory
        yaml_filepath, _ = QFileDialog.getOpenFileName(self._widget, 'Open yaml file', yaml_dir, 'YAML Files (*.yaml)')
        if not yaml_filepath:
            return

        self.set_cached_dir('yaml_dir', path.dirname(yaml_filepath)) # Cache rosbag directory for next time

        self.testbed_yaml_edit.setText(yaml_filepath)

    def on_stopbutton_click(self):
        try:
            stop_benchmark = rospy.ServiceProxy('bmcore/stop_benchmark', StopBenchmark)
            stop_benchmark()
        except rospy.ServiceException as e:
            rospy.logerr("bmcore/stop_benchmark couldn't be called: {ex_val}".format(ex_val=str(e)))

    def state_callback(self, state):
        self.core_status = state.status

        # Live benchmark: requires 'robot name' combobox to be set, and testbed node to be running
        live_benchmark_can_start = True

        if(self.robot_combo.currentText() == ''
        or state.status == BenchmarkCoreState.RUNNING_BENCHMARK
        or not self.benchmark_core_available
        or not self.rosbag_controller_available
        or not self.testbed_node_available):
            live_benchmark_can_start = False

        self.start_button.setEnabled(live_benchmark_can_start)

        # Offline benchmark: only requires benchmark core and rosbag controller
        offline_benchmark_can_start = True

        if(state.status == BenchmarkCoreState.RUNNING_BENCHMARK
        or not self.benchmark_core_available
        or not self.rosbag_controller_available):
            offline_benchmark_can_start = False

        self.start_rosbag_button.setEnabled(offline_benchmark_can_start)

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
            else:
                self.results_detail_label.setText('')

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
        rospy.signal_shutdown('Shutting down')

    def save_settings(self, plugin_settings, instance_settings):
        # Save intrinsic configuration
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore intrinsic configuration
        pass

    def check_required_nodes(self, _):
        # TODO These checks may not be enough: especially for the testbed nodes, where we are only checking for a service from the door node, to assume that all of the testbed's topics are alive
        
        # Benchmark core
        try:
            rospy.wait_for_service('bmcore/start_benchmark', timeout=0.5)
            self.benchmark_core_available = True
        except rospy.ROSException:
            self.benchmark_core_available = False
        
        # Rosbag controller
        try:
            rospy.wait_for_service('/eurobench_rosbag_controller/start_recording', timeout=0.5)
            self.rosbag_controller_available = True
        except rospy.ROSException:
            self.rosbag_controller_available = False

        # Testbed
        if(self.benchmark_group == 'MADROB'):
            door_node_name = rospy.get_param('testbed_nodes')['door']
            set_mode_service_name = '/' + door_node_name + '/set_mode'

            try:
                rospy.wait_for_service(set_mode_service_name, timeout=0.5)
                self.testbed_node_available = True
            except rospy.ROSException:
                self.testbed_node_available = False

        if(self.benchmark_group == 'BEAST'):
            trolley_node_name = rospy.get_param('testbed_nodes')['trolley']
            set_stiffness_service_name = '/' + trolley_node_name + '/set_stiffness'

            try:
                rospy.wait_for_service(set_stiffness_service_name, timeout=0.5)
                self.testbed_node_available = True
            except rospy.ROSException:
                self.testbed_node_available = False

        self.update_required_node_labels()

    def update_required_node_labels(self):
        if self.benchmark_core_available:
            self.core_status_label.setText('OK')
            self.core_status_label.setPalette(self.greenPalette)
        else:
            self.core_status_label.setText('Off')
            self.core_status_label.setPalette(self.redPalette)

        if self.rosbag_controller_available:
            self.rosbag_controller_status_label.setText('OK')
            self.rosbag_controller_status_label.setPalette(self.greenPalette)
        else:
            self.rosbag_controller_status_label.setText('Off')
            self.rosbag_controller_status_label.setPalette(self.redPalette)

        if self.testbed_node_available:
            self.testbed_status_label.setText('OK')
            self.testbed_status_label.setPalette(self.greenPalette)
        else:
            self.testbed_status_label.setText('Off')
            self.testbed_status_label.setPalette(self.redPalette)

    def shutdown_core_and_rosbag_controller(self):
        if self.benchmark_core_available:
            try:
                shutdown_core = rospy.ServiceProxy('bmcore/shutdown', Empty)
                shutdown_core()
            except rospy.ServiceException:
                rospy.logerr('bmcore/shutdown service not available')

        if self.rosbag_controller_available:
            try:
                shutdown_rosbag_controller = rospy.ServiceProxy('/eurobench_rosbag_controller/shutdown', Empty)
                shutdown_rosbag_controller()
            except rospy.ServiceException:
                rospy.logerr('/eurobench_rosbag_controller/shutdown service not available')

    def get_cached_dir(self, dir_type):
        config_dir = path.expanduser('~/.config/eurobench/madrob_beast')

        if path.exists(path.join(config_dir, dir_type)):
            with open(path.join(config_dir, dir_type)) as file:
                return path.expanduser(file.read().strip())
        else:
            return '/home/'

    def set_cached_dir(self, dir_type, dir_path):
        config_dir = path.expanduser('~/.config/eurobench/madrob_beast')

        if not path.exists(config_dir):
            makedirs(config_dir)

        with open(path.join(config_dir, dir_type), 'w+') as file:
            file.write(dir_path)
