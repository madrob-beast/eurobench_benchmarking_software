#!/usr/bin/env python

from os import path
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QComboBox, QPushButton, QLabel, QTextEdit, QSpinBox, QFileDialog, QLineEdit
from python_qt_binding.QtCore import Qt, pyqtSignal

from eurobench_bms_msgs_and_srvs.srv import *
from eurobench_bms_msgs_and_srvs.msg import *


class BenchmarkGui(Plugin):

    set_timer_signal = pyqtSignal(int, bool)
    set_benchmark_info_signal = pyqtSignal(str)

    def __init__(self, context):
        super(BenchmarkGui, self).__init__(context)

        self.setObjectName('EUROBENCH Benchmark Control')

        self._widget = QWidget()

        ui_file = path.join(path.dirname(path.realpath(__file__)), 'benchmark_gui.ui')

        loadUi(ui_file, self._widget)

        self._widget.setObjectName('EUROBENCH Benchmark Control')
        self._widget.setWindowTitle('EUROBENCH Benchmark Control')

        self.benchmark_group = rospy.get_param('/benchmark_group')

        # UI elements
        self.robot_spinbox = self._widget.findChild(QSpinBox, 'robot_spinbox')
        self.run_spinbox = self._widget.findChild(QSpinBox, 'run_spinbox')
        self.clock_value = self._widget.findChild(QLabel, 'clock_value')
        self.benchmark_status_value = self._widget.findChild(QLabel, 'benchmark_status_value')
        self.core_status_value = self._widget.findChild(QLabel, 'core_status_value')
        self.testbed_status_value = self._widget.findChild(QLabel, 'testbed_status_value')
        self.rosbag_controller_status_value = self._widget.findChild(QLabel, 'rosbag_controller_status_value')
        self.start_button = self._widget.findChild(QPushButton, 'start_button')
        self.stop_button = self._widget.findChild(QPushButton, 'stop_button')

        # UI callbacks
        self.start_button.clicked.connect(self.on_startbutton_click)
        self.stop_button.clicked.connect(self.on_stopbutton_click)

        # Disable start buttons, set their listeners
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)

        # Status of required nodes
        self.benchmark_core_available = False
        self.rosbag_controller_available = False
        self.testbed_node_available = False
        self.core_status = None

        # Subscribe to benchmark state
        rospy.Subscriber('bmcore/state', BenchmarkCoreState, self.state_callback, queue_size=1)

        # Run heartbeat checks for required nodes (every second)
        rospy.Timer(rospy.Duration(1), self.check_required_nodes)

        # Update status labels
        rospy.Timer(rospy.Duration(0.1), self.update_status_labels)

        context.add_widget(self._widget)

    def on_startbutton_click(self):
        robot_name = self.robot_spinbox.value()
        run_number = self.run_spinbox.value()
        try:
            start_benchmark = rospy.ServiceProxy('bmcore/start_benchmark', StartBenchmark)
            start_request = StartBenchmarkRequest()
            start_request.robot_name = robot_name
            start_request.run_number = run_number
            start_benchmark(start_request)
        except rospy.ServiceException as e:
            rospy.logerr("bmcore/start_benchmark couldn't be called: {ex_val}".format(ex_val=str(e)))

    @staticmethod
    def on_stopbutton_click():
        try:
            stop_benchmark = rospy.ServiceProxy('bmcore/stop_benchmark', StopBenchmark)
            stop_benchmark()
        except rospy.ServiceException as e:
            rospy.logerr("bmcore/stop_benchmark couldn't be called: {ex_val}".format(ex_val=str(e)))

    def state_callback(self, core_status_msg):
        self.core_status = core_status_msg

    def update_status_labels(self, _):
        all_required_nodes_available = self.benchmark_core_available and self.rosbag_controller_available and self.testbed_node_available
        benchmark_running = self.core_status is not None and self.core_status.status == BenchmarkCoreState.RUNNING_BENCHMARK

        # ready_to_start
        if all_required_nodes_available:
            if not benchmark_running:
                self.benchmark_status_value.setText('Ready to start')
                self.run_spinbox.setEnabled(True)
                self.robot_spinbox.setEnabled(True)
                self.start_button.setEnabled(True)
                self.stop_button.setEnabled(False)
            if benchmark_running:
                self.benchmark_status_value.setText('Running benchmark')
                self.run_spinbox.setEnabled(False)
                self.robot_spinbox.setEnabled(False)
                self.start_button.setEnabled(False)
                self.stop_button.setEnabled(True)
        else:
            self.benchmark_status_value.setText('Cannot start (required nodes not running)')
            self.run_spinbox.setEnabled(False)
            self.robot_spinbox.setEnabled(False)
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(False)

        # Set clock timer
        if benchmark_running:
            seconds_passed = self.core_status.current_benchmark_seconds_passed
            if seconds_passed >= 0:
                minutes_passed = seconds_passed // 60
                seconds_passed = seconds_passed % 60
                self.clock_value.setText('%02d:%02d' % (minutes_passed, seconds_passed))
            else:
                self.clock_value.setText('%02d' % seconds_passed)
        else:
            self.clock_value.setText('--:--')

    def shutdown_plugin(self):
        rospy.signal_shutdown('Shutting down')

    def save_settings(self, plugin_settings, instance_settings):
        # Save intrinsic configuration
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore intrinsic configuration
        pass

    def check_required_nodes(self, _):

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
        if self.benchmark_group == 'MADROB':
            set_mode_service_name = '/madrob/door/set_mode'

            try:
                rospy.wait_for_service(set_mode_service_name, timeout=0.5)
                self.testbed_node_available = True
            except rospy.ROSException:
                self.testbed_node_available = False

        if self.benchmark_group == 'BEAST':
            some_service_name = 'reset_encoders'

            try:
                rospy.wait_for_service(some_service_name, timeout=0.5)
                self.testbed_node_available = True
            except rospy.ROSException:
                self.testbed_node_available = False

        if self.benchmark_core_available:
            self.core_status_value.setText('OK')
        else:
            self.core_status_value.setText('Off')

        if self.rosbag_controller_available:
            self.rosbag_controller_status_value.setText('OK')
        else:
            self.rosbag_controller_status_value.setText('Off')

        if self.testbed_node_available:
            self.testbed_status_value.setText('OK')
        else:
            self.testbed_status_value.setText('Off')
