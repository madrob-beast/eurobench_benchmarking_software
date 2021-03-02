#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

from benchmark_scripts.preprocess.base_preprocess import BasePreprocess


class PreprocessObject(BasePreprocess):
    def __init__(self):
        super(PreprocessObject, self).__init__(data_format_name="distance")
        self.scan_sub = None
        self.distance_list = None

    def start(self, benchmark_group, robot_name, condition_number, run_number, start_time, testbed_conf, preprocess_dir, live_benchmark):
        self.robot_name = robot_name
        self.condition_number = condition_number
        self.run_number = run_number
        self.preprocess_dir = preprocess_dir

        self.distance_list = list()

        if live_benchmark:
            input_topic_name = '/beast_cart/scan_filtered'
        else:
            input_topic_name = '/rosbag_replay/beast_cart/scan_filtered'

        self.scan_sub = rospy.Subscriber(input_topic_name, LaserScan, self.scan_callback)

    def finish(self):
        self.scan_sub.unregister()

        preprocess_file_path = self.preprocessed_csv_file_path()
        df = pd.DataFrame(columns=['time', 'distance'], data=self.distance_list)
        df[['time', 'distance']].to_csv(preprocess_file_path, index=False)

        return self.data_format_name, preprocess_file_path

    def scan_callback(self, scan_msg):
        self.distance_list.append((scan_msg.header.stamp.to_sec(), np.nanmin(scan_msg.ranges)))
