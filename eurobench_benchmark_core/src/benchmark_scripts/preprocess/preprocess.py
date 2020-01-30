#!/usr/bin/env python

import rospy
import message_filters
from os import path
from std_srvs.srv import Trigger, TriggerResponse
from benchmark_scripts.preprocess.base_preprocess import BasePreprocess
from eurobench_bms_msgs_and_srvs.srv import PlayRosbag, PlayRosbagRequest
from madrob_msgs.msg import Door, Handle, Passage
import benchmark_scripts.preprocess.madrob
from benchmark_scripts.preprocess.madrob import *
import benchmark_scripts.preprocess.beast
from benchmark_scripts.preprocess.beast import *

class Preprocess(object):
    def __init__(self, benchmark_group):
        self.benchmark_group = benchmark_group

        if benchmark_group == 'MADROB':
            self.preprocess_scripts = benchmark_scripts.preprocess.madrob.__all__
        
        if benchmark_group == 'BEAST':
            self.preprocess_scripts = benchmark_scripts.preprocess.beast.__all__

        self.saving_data = False
        self.playing_rosbag  = False

        self.running_preprocess_scripts = []
        self.preprocessed_files = {}

    def start(self, robot_name, run_number, start_time, testbed_conf, live_benchmark):
        if not live_benchmark:
            # Play rosbag
            play_rosbag_service = rospy.ServiceProxy('/eurobench_rosbag_controller/play_rosbag', PlayRosbag)

            rosbag_path = testbed_conf['Rosbag path']
            play_rosbag_service(rosbag_path)
            self.playing_rosbag  = True
            
        for preprocess_module in self.preprocess_scripts:
            preprocess = globals()[preprocess_module].PreprocessObject(preprocess_module)
            self.running_preprocess_scripts.append(preprocess)

            preprocess.start(self.benchmark_group, robot_name, run_number, start_time, testbed_conf)

        self.saving_data = True
    
    def finish(self):
        for preprocess in self.running_preprocess_scripts:
            try:
                file_type, file_name = preprocess.finish()
                self.preprocessed_files[file_type] = file_name
            except Exception as e:
                rospy.logerr('Error in preprocess script "' + preprocess.data_type + '": ' + str(e))
                continue

        self.saving_data = False

        self.running_preprocess_scripts = []

        if self.playing_rosbag:
            stop_rosbag_service = rospy.ServiceProxy('/eurobench_rosbag_controller/stop_rosbag', Trigger)
            stop_rosbag_service()
            self.playing_rosbag  = False

        return self.preprocessed_files