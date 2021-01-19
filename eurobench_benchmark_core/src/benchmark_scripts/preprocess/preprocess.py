#!/usr/bin/env python
import traceback
from os import path

import rospy
from std_srvs.srv import Trigger
# from benchmark_scripts.preprocess.base_preprocess import BasePreprocess
from eurobench_bms_msgs_and_srvs.srv import PlayRosbag, PlayRosbagRequest
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
        self.playing_rosbag = False

        self.running_preprocess_scripts = []
        self.preprocessed_files = {}

    def start(self, robot_name, condition_number, run_number, start_time, testbed_conf, testbed_conf_path, live_benchmark, preprocess_dir):
        if not live_benchmark:
            # Not live, rosbag needs to be played.

            # Remap node names, which will add a '_bag' suffix
            testbed_nodes_dict = rospy.get_param('testbed_nodes')
            topic_remappings = testbed_nodes_dict.values()

            # Temporarily change node names to the ones from the bag, so that live ones are ignored
            for node, name in testbed_nodes_dict.items():
                testbed_nodes_dict[node] = name + '_bag'
            rospy.set_param('testbed_nodes', testbed_nodes_dict)

            # Play rosbag
            play_rosbag_service = rospy.ServiceProxy('/eurobench_rosbag_controller/play_rosbag', PlayRosbag)

            rosbag_relative_path = testbed_conf['rosbag_path']
            rosbag_path = path.abspath(path.join(path.dirname(testbed_conf_path), rosbag_relative_path))

            play_rosbag_request = PlayRosbagRequest()
            play_rosbag_request.rosbag_filepath = rosbag_path
            play_rosbag_request.topic_remappings = topic_remappings

            try:
                play_rosbag_service_response = play_rosbag_service(play_rosbag_request)
            except rospy.ServiceException as e:
                rospy.logfatal(traceback.format_exc())
                return False

            if not play_rosbag_service_response.success:
                return False

            self.playing_rosbag = True

        for preprocess_module in self.preprocess_scripts:
            # noinspection PyBroadException
            try:
                preprocess_module_obj = globals()[preprocess_module].PreprocessObject()
                self.running_preprocess_scripts.append(preprocess_module_obj)
                rospy.loginfo("preprocess script initialised: {name}".format(name=preprocess_module))

                # noinspection PyBroadException
                try:
                    preprocess_module_obj.start(self.benchmark_group, robot_name, condition_number, run_number, start_time, testbed_conf, preprocess_dir)
                    rospy.loginfo("preprocess script started: {name}".format(name=preprocess_module))
                except Exception:
                    rospy.logerr("failed to start preprocess script {name} due to exception: {exception}".format(name=preprocess_module, exception=traceback.format_exc()))

            except Exception:
                rospy.logerr("failed to initialise preprocess script {name} due to exception: {exception}".format(name=preprocess_module, exception=traceback.format_exc()))

        self.saving_data = True
        return True
    
    def finish(self):
        for preprocess in self.running_preprocess_scripts:
            # noinspection PyBroadException
            try:
                file_type, file_name = preprocess.finish()
                self.preprocessed_files[file_type] = file_name
                rospy.loginfo("preprocess script finished: {name}".format(name=preprocess.data_format_name))
            except Exception:
                rospy.logerr("error finishing preprocess script {name} due to exception: {exception}".format(name=preprocess.data_format_name, exception=traceback.format_exc()))
                continue

        self.saving_data = False

        self.running_preprocess_scripts = []

        if self.playing_rosbag:
            # Remove '_bag' from node names
            testbed_nodes_dict = rospy.get_param('testbed_nodes')
            for node, name in testbed_nodes_dict.items():
                testbed_nodes_dict[node] = name.replace('_bag', '')
            rospy.set_param('testbed_nodes', testbed_nodes_dict)

            stop_rosbag_service = rospy.ServiceProxy('/eurobench_rosbag_controller/stop_rosbag', Trigger)
            stop_rosbag_service()
            self.playing_rosbag = False

        return self.preprocessed_files
