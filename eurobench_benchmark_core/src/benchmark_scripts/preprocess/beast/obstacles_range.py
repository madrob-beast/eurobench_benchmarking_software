#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

from benchmark_scripts.preprocess.base_preprocess import BasePreprocess


class PreprocessObject(BasePreprocess):
    def __init__(self):
        super(PreprocessObject, self).__init__(data_format_name="range")
        self.scan_sub = None
        self.range_list = None

    def start(self, benchmark_group, robot_name, condition_number, run_number, start_time, testbed_conf, preprocess_dir):
        self.robot_name = robot_name
        self.condition_number = condition_number
        self.run_number = run_number
        self.preprocess_dir = preprocess_dir

        self.range_list = list()
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def finish(self):
        self.scan_sub.unregister()

        preprocess_file_path = self.preprocessed_csv_file_path()
        df = pd.DataFrame(columns=['time', 'range'], data=self.range_list)
        df[['time', 'range']].to_csv(preprocess_file_path, index=False)

        return self.data_format_name, preprocess_file_path

    def scan_callback(self, scan_msg):
        self.range_list.append((scan_msg.header.stamp.to_sec(), np.min(scan_msg.ranges)))
