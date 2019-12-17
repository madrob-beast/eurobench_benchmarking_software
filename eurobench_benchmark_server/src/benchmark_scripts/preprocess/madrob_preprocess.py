#!/usr/bin/env python

import rospy
from os import path
from benchmark_scripts.preprocess.base_preprocess import BasePreprocess
from madrob_msgs.msg import Door, Handle

class MadrobPreprocess(BasePreprocess):
    def __init__(self, output_dir):
        self.output_dir = output_dir
        
        self.recording = False
        self.door_node_name = rospy.get_param('door_node_name')
        self.handle_node_name = rospy.get_param('handle_node_name')

        self.preprocessed_files = {}

        self.door_sub = None
        self.handle_sub = None

    def start(self, robot_name, run_number, start_time, rosbag_path=None):
        self.robot_name = robot_name
        self.run_number = run_number
        self.start_time = start_time

        if rosbag_path:
            # TODO Start playing rosbag
            rospy.logfatal('Pre-process: rosbag playing not implemented yet.')
            rospy.signal_shutdown('Pre-process: rosbag playing not implemented yet.')

        # Create CSV files and set their headers
        self.open_file('door_angle', 'timestamp,angle')
        self.open_file('door_velocity', 'timestamp,velocity')
        self.open_file('handle_force', 'timestamp,force')

        # Register subscribers
        self.door_sub = rospy.Subscriber('/' + self.door_node_name + '/state', Door, self.door_state_callback)
        self.handle_sub = rospy.Subscriber('/' + self.handle_node_name + '/state', Handle, self.handle_state_callback)

        self.recording = True
    
    def finish(self):
        self.door_sub.unregister()
        self.handle_sub.unregister()

        self.recording = False

        # Build a dict with filenames to return, and close the files
        preprocessed_filenames_dict = {}
        for file_type, file in self.preprocessed_files.items():
            preprocessed_filenames_dict[file_type] = file.name
            file.close()

        self.preprocessed_files = {}

        return preprocessed_filenames_dict

    def open_file(self, data_type, header):
        filename = 'subject_%s_MADROB_%s_%03d_%s.csv' % (self.robot_name, data_type, self.run_number, self.start_time.strftime('%Y%m%d_%H%M%S'))
        self.preprocessed_files[data_type] = open(path.join(self.output_dir, filename), 'w+')
        self.preprocessed_files[data_type].write(header + '\n')

    def door_state_callback(self, door):
        self.preprocessed_files['door_angle'].write('%d.%d, %.1f\n' % (door.header.stamp.secs, door.header.stamp.nsecs, door.angle))
        self.preprocessed_files['door_velocity'].write('%d.%d, %.1f\n' % (door.header.stamp.secs, door.header.stamp.nsecs, door.velocity))

    def handle_state_callback(self, handle):
        self.preprocessed_files['handle_force'].write('%d.%d, %.1f\n' % (handle.header.stamp.secs, handle.header.stamp.nsecs, handle.force))