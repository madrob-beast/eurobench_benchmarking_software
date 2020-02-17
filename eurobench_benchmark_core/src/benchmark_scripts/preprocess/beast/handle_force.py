#!/usr/bin/env python

import rospy
from benchmark_scripts.preprocess import preprocess_utils
from benchmark_scripts.preprocess.base_preprocess import BasePreprocess
from beast_msgs.msg import Handle

class PreprocessObject(BasePreprocess):
    def __init__(self, data_type):
        self.data_type = data_type

    def start(self, benchmark_group, robot_name, run_number, start_time, testbed_conf, preprocess_dir):
        self.handle_force_file = preprocess_utils.open_preprocessed_csv(preprocess_dir, benchmark_group, robot_name, run_number, start_time, self.data_type)
        
        handle_node_name = rospy.get_param('handle_node_name')
        self.handle_sub = rospy.Subscriber('/' + handle_node_name + '/force', Handle, self.handle_state_callback)

    def finish(self):
        self.handle_sub.unregister()
        self.handle_force_file.close()
        return self.data_type, self.handle_force_file.name

    def handle_state_callback(self, handle):
        self.handle_force_file.write('%d.%d, %.1f\n' % (handle.header.stamp.secs, handle.header.stamp.nsecs, handle.force))