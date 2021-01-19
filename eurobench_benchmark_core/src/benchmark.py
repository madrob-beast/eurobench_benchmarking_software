#!/usr/bin/env python
import glob

import rospy
import json
import bms_utils
from datetime import datetime, timedelta
from collections import OrderedDict
from os import path, makedirs
from shutil import copyfile

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
        elif benchmark_group == 'BEAST':
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
        self.testbed_conf = testbed_conf  # self.testbed_conf is the testbed config associated to a rosbag (live_benchmark is False), otherwise self.testbed_conf will be None
        self.original_testbed_conf_path = testbed_conf_path
        self.start_time = datetime.now() + timedelta(seconds=rospy.get_param('benchmark_countdown'))
        self.start_time_ros = rospy.Time.now() + rospy.Duration(rospy.get_param('benchmark_countdown'))

        self.result = {}

        if not self.live_benchmark:
            self.robot_name = self.testbed_conf['robot_name']
            self.run_number = self.testbed_conf['run_number']

    def get_benchmark_info(self):
        benchmark_info = OrderedDict([
            ('Robot name', self.robot_name),
            ('Run number', self.run_number),
            ('Benchmark', self.benchmark_group),
            ('Start time', self.start_time.strftime('%Y-%m-%d_%H:%M:%S')),
            ('Result', self.result)
        ])

        return json.dumps(benchmark_info, indent=2)

    def execute(self):
        start_time_str = self.start_time.strftime('%Y%m%d_%H%M%S')

        madrob_conditions_path = rospy.get_param('~madrob_conditions_path')
        conditions_table = dict()
        conditions_path_table = dict()
        for condition_path in glob.glob(path.join(madrob_conditions_path, 'condition_*.yaml')):
            condition_number = int(path.basename(condition_path).replace("condition_", '').replace(".yaml", ''))
            with open(condition_path) as condition_file:
                condition = yaml.safe_load(condition_file)
                conditions_table[(condition['benchmark_type'], condition['door_opening_side'], condition['robot_approach_side'])] = condition_number
                conditions_path_table[(condition['benchmark_type'], condition['door_opening_side'], condition['robot_approach_side'])] = condition_path

        if self.live_benchmark:
            # Setup testbed
            self.testbed_comm.setup_testbed()
            self.testbed_conf = self.testbed_comm.get_testbed_conf_file(self.start_time_ros, self.robot_name, self.run_number)

            condition_number = conditions_table[(self.testbed_conf['benchmark_type'], self.testbed_conf['door_opening_side'], self.testbed_conf['robot_approach_side'])]
            self.testbed_conf['condition_number'] = condition_number
            condition_path = conditions_path_table[(self.testbed_conf['benchmark_type'], self.testbed_conf['door_opening_side'], self.testbed_conf['robot_approach_side'])]

            # make paths for directory and each file
            benchmark_id = "subject_{subject_number:03d}_cond_{condition_number:03d}_run_{run_number:03d}_{t}".format(
                subject_number=int(self.robot_name),
                condition_number=condition_number,
                run_number=self.run_number,
                t=start_time_str)
            benchmark_results_dir = path.join(self.output_dir, benchmark_id)
            makedirs(benchmark_results_dir)

            rosbag_filename = "subject_{subject_number:03d}_cond_{condition_number:03d}_run_{run_number:03d}_{t}.bag".format(
                subject_number=int(self.robot_name),
                condition_number=condition_number,
                run_number=self.run_number,
                t=start_time_str)
            rosbag_filepath = path.join(benchmark_results_dir, rosbag_filename)
            self.testbed_conf['rosbag_path'] = rosbag_filename

            testbed_conf_path = path.join(benchmark_results_dir, 'run_info_subject_{subject_number:03d}_cond_{condition_number:03d}_run_{run_number:03d}.yaml'.format(
                subject_number=int(self.robot_name),
                condition_number=condition_number,
                run_number=self.run_number,
            ))

            with open(testbed_conf_path, 'w') as testbed_conf_file:
                yaml.dump(self.testbed_conf, testbed_conf_file, default_flow_style=False)

            copyfile(condition_path, path.join(benchmark_results_dir, path.basename(condition_path)))

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
            # if this is a run from bag the testbed configuration (self.testbed_conf) is the input to compute every other value

            condition_number = conditions_table[(self.testbed_conf['benchmark_type'], self.testbed_conf['door_opening_side'], self.testbed_conf['robot_approach_side'])]
            self.testbed_conf['condition_number'] = condition_number
            condition_path = conditions_path_table[(self.testbed_conf['benchmark_type'], self.testbed_conf['door_opening_side'], self.testbed_conf['robot_approach_side'])]

            # make paths for directory and each file
            benchmark_id = "subject_{subject_number:03d}_cond_{condition_number:03d}_run_{run_number:03d}_{t}".format(
                subject_number=int(self.robot_name),
                condition_number=condition_number,
                run_number=self.run_number,
                t=start_time_str)
            benchmark_results_dir = path.join(self.output_dir, benchmark_id)
            makedirs(benchmark_results_dir)

            testbed_conf_path = path.join(benchmark_results_dir, 'run_info_subject_{subject_number:03d}_cond_{condition_number:03d}_run_{run_number:03d}.yaml'.format(
                subject_number=int(self.robot_name),
                condition_number=condition_number,
                run_number=self.run_number,
            ))

            with open(testbed_conf_path, 'w') as testbed_conf_file:
                yaml.dump(self.testbed_conf, testbed_conf_file, default_flow_style=False)

            copyfile(condition_path, path.join(benchmark_results_dir, path.basename(condition_path)))

        # Start preprocessing scripts
        preprocess_ret = self.preprocess.start(self.robot_name, condition_number, self.run_number, self.start_time, self.testbed_conf, self.original_testbed_conf_path, self.live_benchmark, benchmark_results_dir)

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
