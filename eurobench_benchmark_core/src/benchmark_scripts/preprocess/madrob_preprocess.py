#!/usr/bin/env python

import rospy
import message_filters
from os import path
from std_srvs.srv import Trigger, TriggerResponse
from benchmark_scripts.preprocess.base_preprocess import BasePreprocess
from eurobench_bms_msgs_and_srvs.srv import PlayRosbag, PlayRosbagRequest
from madrob_msgs.msg import Door, Handle, Passage

class MadrobPreprocess(BasePreprocess):
    def __init__(self, output_dir):
        self.output_dir = output_dir
        
        self.recording = False
        self.playing_rosbag  = False
        self.door_node_name = rospy.get_param('door_node_name')
        self.handle_node_name = rospy.get_param('handle_node_name')
        self.passage_node_name = rospy.get_param('passage_node_name')

        self.preprocessed_files = {}

        self.door_sub = None
        self.handle_sub = None

        self.cw_right_sub = None
        self.cw_left_sub = None
        self.ccw_right_sub = None
        self.ccw_left_sub = None

    def start(self, robot_name, run_number, start_time, rosbag_path=None):
        self.robot_name = robot_name
        self.run_number = run_number
        self.start_time = start_time

        if rosbag_path:
            # Play rosbag
            play_rosbag_service = rospy.ServiceProxy('/eurobench_rosbag_controller/play_rosbag', PlayRosbag)
            play_rosbag_service(rosbag_path)
            self.playing_rosbag  = True
            

        # Create CSV files and set their headers
        self.open_file('door_angle', 'timestamp,angle')
        self.open_file('door_velocity', 'timestamp,velocity')
        self.open_file('handle_force', 'timestamp,force')

        # TODO below files just for graph analysis. in the future: one file passage_status -> timestamp,passage_status. same subs
        self.open_file('cw_right', 'timestamp,range')
        self.open_file('cw_left', 'timestamp,range')
        self.open_file('ccw_right', 'timestamp,range')
        self.open_file('ccw_left', 'timestamp,range')

        # Register subscribers
        self.door_sub = rospy.Subscriber('/' + self.door_node_name + '/state', Door, self.door_state_callback)
        self.handle_sub = rospy.Subscriber('/' + self.handle_node_name + '/state', Handle, self.handle_state_callback)

        # Door passage subscribers: grouped using ApproximateTimeSynchronizer, so that all topics are processed by the same callback: http://docs.ros.org/melodic/api/message_filters/html/python/index.html#message_filters.ApproximateTimeSynchronizer
        self.cw_right_sub = message_filters.Subscriber('/' + self.passage_node_name + '/cw_right', Passage)
        self.cw_left_sub = message_filters.Subscriber('/' + self.passage_node_name + '/cw_left', Passage)
        self.ccw_right_sub = message_filters.Subscriber('/' + self.passage_node_name + '/ccw_right', Passage)
        self.ccw_left_sub = message_filters.Subscriber('/' + self.passage_node_name + '/ccw_left', Passage)

        self.door_proximity_timesync = message_filters.ApproximateTimeSynchronizer([self.cw_right_sub, self.cw_left_sub, self.ccw_right_sub, self.ccw_left_sub], 10, 0.1)
        self.door_proximity_timesync.registerCallback(self.door_proximity_callback)

        self.recording = True
    
    def finish(self):
        self.door_sub.unregister()
        self.handle_sub.unregister()

        self.cw_right_sub.unregister()
        self.cw_left_sub.unregister()
        self.ccw_right_sub.unregister()
        self.ccw_left_sub.unregister()

        self.recording = False

        # Build a dict with filenames to return, and close the files
        preprocessed_filenames_dict = {}
        for file_type, file in self.preprocessed_files.items():
            preprocessed_filenames_dict[file_type] = file.name
            file.close()

        self.preprocessed_files = {}

        if self.playing_rosbag:
            stop_rosbag_service = rospy.ServiceProxy('/eurobench_rosbag_controller/stop_rosbag', Trigger)
            stop_rosbag_service()
            self.playing_rosbag  = False

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

    def door_proximity_callback(self, cw_right, cw_left, ccw_right, ccw_left):
        # TODO: there are 4 ranges per Passage message. We are only writing the second one (ranges[1]). Check what is relevant to write here.
        self.preprocessed_files['cw_right'].write('%d.%d, %.1f\n' % (cw_right.header.stamp.secs, cw_right.header.stamp.nsecs, cw_right.ranges[1].range))
        self.preprocessed_files['cw_left'].write('%d.%d, %.1f\n' % (cw_left.header.stamp.secs, cw_left.header.stamp.nsecs, cw_left.ranges[1].range))
        self.preprocessed_files['ccw_right'].write('%d.%d, %.1f\n' % (ccw_right.header.stamp.secs, ccw_right.header.stamp.nsecs, ccw_right.ranges[1].range))
        self.preprocessed_files['ccw_left'].write('%d.%d, %.1f\n' % (ccw_left.header.stamp.secs, ccw_left.header.stamp.nsecs, ccw_left.ranges[1].range))