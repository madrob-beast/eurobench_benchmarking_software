#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

from benchmark_scripts.preprocess.base_preprocess import BasePreprocess


class PreprocessObject(BasePreprocess):
    def __init__(self):
        super(PreprocessObject, self).__init__(data_format_name="landmarkTrajectories")
        self.pose_sub = None
        self.position_list = None

    def start(self, benchmark_group, robot_name, condition_number, run_number, start_time, testbed_conf, preprocess_dir):
        self.robot_name = robot_name
        self.condition_number = condition_number
        self.run_number = run_number
        self.preprocess_dir = preprocess_dir

        self.position_list = list()

        input_topic_name = '/amcl_pose'

        self.pose_sub = rospy.Subscriber(input_topic_name, PoseWithCovarianceStamped, self.pose_callback)

    def finish(self):
        self.pose_sub.unregister()

        preprocess_file_path = self.preprocessed_csv_file_path()
        df = pd.DataFrame(columns=['time', 'cart_x', 'cart_y'], data=self.position_list)
        df[['time', 'cart_x', 'cart_y']].to_csv(preprocess_file_path, index=False)

        return self.data_format_name, preprocess_file_path

    def pose_callback(self, msg):
        self.position_list.append((msg.header.stamp.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y))
