#!/usr/bin/env python

import rospy
from os import path
from benchmark_scripts.preprocess.base_preprocess import BasePreprocess
from madrob_msgs.msg import Door

class MadrobPreprocess(BasePreprocess):
    def __init__(self, output_dir):
        self.output_dir = output_dir
        
        self.recording = False
        self.door_node_name = rospy.get_param('door_node_name')

        self.door_sub = None

    def start(self, robot_name, run_number, start_time, rosbag_path=None):
        if rosbag_path:
            # TODO Start playing rosbag
            rospy.logfatal('Pre-process: rosbag playing not implemented yet.')
            rospy.signal_shutdown('Pre-process: rosbag playing not implemented yet.')

        # Create CSV files and write their headers
        door_angle_filename = 'subject_%s_MADROB_door_angle_%03d_%s.csv' % (robot_name, run_number, start_time.strftime('%Y%m%d_%H%M%S'))
        self.door_angle_file = open(path.join(self.output_dir, door_angle_filename), 'w+')
        self.door_angle_file.write('timestamp, angle\n')

        door_velocity_filename = 'subject_%s_MADROB_door_velocity_%03d_%s.csv' % (robot_name, run_number, start_time.strftime('%Y%m%d_%H%M%S'))
        self.door_velocity_file = open(path.join(self.output_dir, door_velocity_filename), 'w+')
        self.door_velocity_file.write('timestamp, velocity\n')

        # Register subscribers
        self.door_sub = rospy.Subscriber('/' + self.door_node_name + '/state', Door, self.door_state_callback)

        self.recording = True
    
    def finish(self):
        self.door_sub.unregister()

        self.recording = False

        self.door_angle_file.close()
        self.door_velocity_file.close()

    def door_state_callback(self, door):
        self.door_angle_file.write('%d.%d, %.1f\n' % (door.header.stamp.secs, door.header.stamp.nsecs, door.angle))
        self.door_velocity_file.write('%d.%d, %.1f\n' % (door.header.stamp.secs, door.header.stamp.nsecs, door.velocity))