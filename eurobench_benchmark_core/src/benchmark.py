#!/usr/bin/env python

import rospy
import json
import bms_utils
from datetime import datetime, timedelta
from collections import OrderedDict
from os import path, makedirs

import yaml
from std_srvs.srv import Trigger
from eurobench_bms_msgs_and_srvs.srv import StartRecording, StartRecordingRequest
from benchmark_scripts.testbed_comm.madrob_testbed_comm import MadrobTestbedComm
from benchmark_scripts.testbed_comm.beast_testbed_comm import BeastTestbedComm
from benchmark_scripts.preprocess.preprocess import Preprocess


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

        self.start_recording_service = rospy.ServiceProxy('/eurobench_rosbag_controller/start_recording', StartRecording)
        self.stop_recording_service = rospy.ServiceProxy('/eurobench_rosbag_controller/stop_recording', Trigger)

        self.preprocess = Preprocess(self.benchmark_group)

        if benchmark_group == 'MADROB':
            self.testbed_comm = MadrobTestbedComm(self.config['benchmarks'])
        if benchmark_group == 'BEAST':
            self.testbed_comm = BeastTestbedComm()

        self.terminated = None
        self.live_benchmark = None
        self.testbed_conf = None
        self.original_testbed_conf_path = None
        self.start_time = None
        self.start_time_ros = None
        self.result = None
        self.robot_name = None
        self.run_number = None

    def setup(self, robot_name, run_number, live_benchmark, testbed_conf, testbed_conf_path):
        self.terminated = False
        self.robot_name = robot_name
        self.run_number = run_number
        self.live_benchmark = live_benchmark
        self.testbed_conf = testbed_conf  # self.testbed_conf is the path of the testbed config associated to a rosbag (live_benchmark is False), otherwise self.testbed_conf will be None
        self.original_testbed_conf_path = testbed_conf_path
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
        start_time_str = self.start_time.strftime('%Y%m%d_%H%M%S')
        benchmark_id = '%s_%s_%03d_%s' % (self.benchmark_group, self.robot_name, self.run_number, start_time_str)
        
        # Create a directory for this benchmark's files
        benchmark_results_dir = path.join(self.output_dir, benchmark_id)
        makedirs(benchmark_results_dir)

        testbed_conf_path = path.join(benchmark_results_dir, '%s_testbed.yaml' % str(self.benchmark_group).lower())

        if self.live_benchmark:
            # Setup testbed
            self.testbed_comm.setup_testbed()

            # File name and path of rosbag
            rosbag_filename = '%s.bag' % benchmark_id
            rosbag_filepath = path.join(benchmark_results_dir, rosbag_filename)

            # Save testbed config yaml file, including rosbag filepath
            self.testbed_conf = self.testbed_comm.write_testbed_conf_file(testbed_conf_path, self.start_time_ros, self.robot_name, self.run_number, rosbag_filename)

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
        else:
            # if this is a run from bag, save the testbed config as-is to the output dir
            with open(testbed_conf_path, 'w') as testbed_conf_file:
                yaml.dump(self.testbed_conf, testbed_conf_file, default_flow_style=False)

        # Start preprocessing scripts
        preprocess_ret = self.preprocess.start(self.robot_name, self.run_number, self.start_time, self.testbed_conf, self.original_testbed_conf_path, self.live_benchmark, benchmark_results_dir)

        if preprocess_ret:
            # Loop while benchmark is running
            while not self.terminated:
                rospy.sleep(0.1)
        else:
            rospy.logerr("could not start execution of the benchmark")

        # Stop preprocessing
        self.preprocess.finish()

        if self.live_benchmark:
            # Stop recording
            response = self.stop_recording_service()
            if not response.success:
                rospy.logerr('Could not stop recording rosbag')
