#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import rospy
from benchmark_scripts.preprocess.base_preprocess import BasePreprocess
from beast_msgs.msg import Handle


class PreprocessObject(BasePreprocess):
    def __init__(self):
        super(PreprocessObject, self).__init__(data_format_name="wrench")
        self.handle_sub = None
        self.handle_force_list = None

    def start(self, benchmark_group, robot_name, condition_number, run_number, start_time, testbed_conf, preprocess_dir):
        self.robot_name = robot_name
        self.condition_number = condition_number
        self.run_number = run_number
        self.preprocess_dir = preprocess_dir

        self.handle_force_list = list()
        self.handle_sub = rospy.Subscriber("/handle", Handle, self.handle_state_callback)

    def finish(self):
        self.handle_sub.unregister()

        preprocess_file_path = self.preprocessed_csv_file_path()
        df = pd.DataFrame(columns=['time', 'force_x'], data=self.handle_force_list)
        df[['time', 'force_x']].to_csv(preprocess_file_path, index=False)

        return self.data_format_name, preprocess_file_path

    def handle_state_callback(self, handle):
        force_newtons = handle.force * 0.0098  # convert handle.force from grams-force to Newtons
        self.handle_force_list.append((handle.header.stamp.to_sec(), force_newtons))
