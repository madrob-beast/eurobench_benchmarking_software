#!/usr/bin/env python

import rospy
import yaml
import json
import time
import bms_utils
from datetime import datetime, timedelta
from collections import OrderedDict
from os import path, makedirs
from exceptions import NotImplementedError
from std_srvs.srv import Trigger
from eurobench_bms_msgs_and_srvs.srv import StartRecording, StartRecordingRequest
from benchmark_scripts.testbed_comm.madrob_testbed_comm import MadrobTestbedComm
from benchmark_scripts.preprocess.preprocess import Preprocess
import benchmark_scripts.performance.madrob
from benchmark_scripts.performance.madrob import *
import benchmark_scripts.performance.beast
from benchmark_scripts.performance.beast import *


class Benchmark(object):
    def __init__(self, benchmark_group, config):
        self.benchmark_group = benchmark_group
        self.config = config
        self.output_dir = bms_utils.get_output_dir()

        try:
            rospy.wait_for_service('/eurobench_rosbag_controller/start_recording', timeout=5.0)
        except rospy.ROSException:
            rospy.logfatal('eurobench_rosbag_controller: start_recording service unavailable.')
            rospy.signal_shutdown('Rosbag controller unavailable')

        self.start_recording_service = rospy.ServiceProxy('/eurobench_rosbag_controller/start_recording',
                                                          StartRecording)
        self.stop_recording_service = rospy.ServiceProxy('/eurobench_rosbag_controller/stop_recording', Trigger)

        self.preprocess = Preprocess(self.benchmark_group)

        if benchmark_group == 'MADROB':
            self.testbed_device = 'door'
            self.testbed_comm = MadrobTestbedComm(self.config['benchmarks'])

            self.performance_indicators = benchmark_scripts.performance.madrob.__all__

        if benchmark_group == 'BEAST':
            self.testbed_device = 'trolley'

            self.performance_indicators = benchmark_scripts.performance.beast.__all__

    def setup(self, robot_name, run_number, live_benchmark, testbed_conf):
        self.terminated = False
        self.robot_name = robot_name
        self.run_number = run_number
        self.live_benchmark = live_benchmark
        self.testbed_conf = testbed_conf
        self.start_time = datetime.now() + timedelta(seconds=rospy.get_param('benchmark_countdown'))
        self.start_time_ros = rospy.Time.now() + rospy.Duration(rospy.get_param('benchmark_countdown'))

        self.result = {}

        if not self.live_benchmark:
            self.robot_name = self.testbed_conf['Robot name']
            self.run_number = self.testbed_conf['Run number']

    def get_benchmark_info(self):
        benchmark_info = OrderedDict([
            ('Robot name', self.robot_name),
            ('Run number', self.run_number),
            ('Benchmark', self.benchmark_group),
            ('Start time', self.start_time.strftime('%Y-%m-%d_%H:%M:%S')),
            ('Result', self.result)
        ])

        return json.dumps(benchmark_info, indent=2)

    def save_result(self):
        # If it doesn't exist, create directory with the robot's name
        team_output_dir = path.join(self.output_dir, self.robot_name)
        if not path.exists(team_output_dir):
            makedirs(team_output_dir)

        result_filename = path.join(team_output_dir, '%s_%s.txt' % (
            self.benchmark_group, self.start_time.strftime('%Y-%m-%d_%H:%M:%S')))

        with open(result_filename, 'w') as outfile:
            outfile.write(self.get_benchmark_info())

        rospy.loginfo('\nBENCHMARK RESULT:')
        rospy.loginfo(self.get_benchmark_info() + '\n')

    def execute(self):
        if self.live_benchmark:
            # Setup testbed
            self.testbed_comm.setup_testbed()

            # File name and path of rosbag
            start_time_str = self.start_time.strftime('%Y%m%d_%H%M%S')
            rosbag_filepath = path.join(self.output_dir, 'subject_%s_%s_%03d_%s.bag' % 
                (self.robot_name, self.benchmark_group, self.run_number, start_time_str))

            # Save testbed config yaml file, including rosbag filepath
            self.testbed_conf_path = path.join(self.output_dir, 'subject_%s_%s_%03d_%s.yaml' % 
                (self.robot_name, self.testbed_device, self.run_number, start_time_str))
            self.testbed_conf = self.testbed_comm.write_testbed_conf_file(self.testbed_conf_path, self.start_time_ros, self.robot_name, self.run_number, rosbag_filepath)

            # Start recording rosbag
            request = StartRecordingRequest()
            request.rosbag_filepath = rosbag_filepath

            if rospy.has_param('excluded_topics'):
                request.excluded_topics += rospy.get_param('excluded_topics')

            if 'excluded_topics' in self.config:
                request.excluded_topics += self.config['excluded_topics']

            response = self.start_recording_service(request)
            if not response.success:
                rospy.logerr('Could not start recording rosbag')
                return


        # Start preprocessing scripts
        self.preprocess.start(self.robot_name, self.run_number, self.start_time, self.testbed_conf, self.live_benchmark)

        # Loop while benchmark is running
        while not self.terminated:
            rospy.sleep(0.1)

        # Stop preprocessing
        preprocessed_filenames_dict = self.preprocess.finish()

        if self.live_benchmark:
            # Stop recording
            response = self.stop_recording_service()
            if not response.success:
                rospy.logerr('Could not stop recording rosbag')

        # Calculate PIs - Run all pre-processing scripts
        for performance_indicator_module in self.performance_indicators:
            pi = globals()[performance_indicator_module].performance_indicator

            try:
                pi(preprocessed_filenames_dict, self.testbed_conf, self.output_dir, self.start_time)
            except Exception as e:
                rospy.logerr("Error in performance indicator: {pi_name}, Type: {ex_type}, Value: {ex_val}".format(pi_name=performance_indicator_module, ex_type=str(type(e)), ex_val=str(e)))
                continue
