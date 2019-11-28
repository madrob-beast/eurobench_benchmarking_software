#!/usr/bin/env python

import rospy
import yaml
import json
import time
import bms_utils
from datetime import datetime
from collections import OrderedDict
from os import path, makedirs
from exceptions import NotImplementedError
from benchmark_scripts.testbed_comm.madrob_testbed_comm import MadrobTestbedComm
from benchmark_scripts.preprocess.madrob_preprocess import MadrobPreprocess


class Benchmark(object):
    def __init__(self, benchmark_group, config):
        self.benchmark_group = benchmark_group
        self.config = config
        self.output_dir = bms_utils.get_output_dir()

        if benchmark_group == 'MADROB':
            self.testbed_device = 'door'
            self.testbed_comm = MadrobTestbedComm(self.config['benchmarks'])
            self.preprocess = MadrobPreprocess()


        if benchmark_group == 'BEAST':
            self.testbed_device = 'trolley'
    

    def setup(self, robot_name, run_number):
        self.terminated = False
        self.robot_name = robot_name
        self.run_number = run_number
        self.start_time = datetime.now()
        self.result = {}

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
        # Setup testbed
        self.testbed_comm.setup_testbed()

        # Save testbed config yaml file
        start_time_str = self.start_time.strftime('%Y%m%d_%H%M%S')
        filepath = path.join(self.output_dir, 'subject_%s_%s_%03d_%s.yaml' % 
            (self.robot_name, self.testbed_device, self.run_number, start_time_str))
        self.testbed_comm.write_testbed_conf_file(filepath)

        # Start recording rosbag

        # Start preprocessing script