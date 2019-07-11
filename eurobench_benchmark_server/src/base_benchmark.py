#!/usr/bin/env python

import rospy
import yaml
import json
import time
from datetime import datetime
from collections import OrderedDict
from os import path, makedirs
from exceptions import NotImplementedError


class BaseBenchmark(object):

    def setup(self, robot_name):
        self.terminated = False
        self.robot_name = robot_name
        self.start_time = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
        self.result = {}

        # Load config from yaml file
        config_dir = rospy.get_param('benchmark_config_directory')
        config_filepath = path.join(config_dir, '%s.yaml' % (self.benchmark_code))

        with open(config_filepath) as config_file:
            self.config = yaml.load(config_file)

        self.has_timed_out = False
        self.timeout = None
        if 'benchmark_timeout' in self.config:
            self.timeout = time.time() + self.config['benchmark_timeout']

    def get_benchmark_info(self):
        benchmark_info = OrderedDict([
            ('Robot name', self.robot_name),
            ('Benchmark ID', self.benchmark_code),
            ('Start time', self.start_time),
            ('Timed out', self.has_timed_out),
            ('Result', self.result)
        ])

        return json.dumps(benchmark_info, indent=2)

    def save_result(self):
        output_dir = path.expanduser(rospy.get_param('benchmark_output_directory'))

        # If it doesn't exist, create directory with the robot's name
        team_output_dir = path.join(output_dir, self.robot_name)
        if not path.exists(team_output_dir):
            makedirs(team_output_dir)

        result_filename = path.join(team_output_dir, '%s_%s.txt' % (
            self.benchmark_code, self.start_time))

        with open(result_filename, 'w') as outfile:
            outfile.write(self.get_benchmark_info())

        rospy.loginfo('\nBENCHMARK RESULT:')
        rospy.loginfo(self.get_benchmark_info() + '\n')

    def execute(self):
        raise NotImplementedError()
