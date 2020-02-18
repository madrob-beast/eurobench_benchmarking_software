#!/usr/bin/env python

import rospy
from benchmark_scripts.preprocess import preprocess_utils
from benchmark_scripts.preprocess.base_preprocess import BasePreprocess
from madrob_msgs.msg import Door

class PreprocessObject(BasePreprocess):
    def __init__(self, data_type):
        self.data_type = data_type

    def start(self, benchmark_group, robot_name, run_number, start_time, testbed_conf, preprocess_dir):
        self.door_angle_file = preprocess_utils.open_preprocessed_csv(preprocess_dir, benchmark_group, robot_name, run_number, start_time, self.data_type)
        
        door_node_name = rospy.get_param('testbed_nodes')['door']
        self.door_sub = rospy.Subscriber('/' + door_node_name + '/state', Door, self.door_state_callback)

    def finish(self):
        self.door_sub.unregister()
        self.door_angle_file.close()
        return self.data_type, self.door_angle_file.name

    def door_state_callback(self, door):
        self.door_angle_file.write('%d.%d, %.1f\n' % (door.header.stamp.secs, door.header.stamp.nsecs, door.angle))
