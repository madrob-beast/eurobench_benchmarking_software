#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import message_filters

from benchmark_scripts.preprocess.base_preprocess import BasePreprocess
from benchmark_scripts.preprocess import preprocess_utils

from madrob_msgs.msg import Door, Passage, Handle
from std_msgs.msg import Float64

import numpy as np


class PreprocessObject(BasePreprocess):
    def __init__(self, data_type):
        self.data_type = data_type

        # Passage sensors values (a subset of the total number of sensors available from the testbed)
        self.cw_num_sensors = 5
        self.ccw_num_sensors = 5
        self.cw = np.zeros(self.cw_num_sensors)
        self.ccw = np.zeros(self.ccw_num_sensors)

        # Door angular position
        self.door_angle = 0.0

        # Handle force signal
        self.handle_force = 0.0

        # Passage is detected by checking when the height of the passage sensors becomes higher than a threshold
        # The first occurrence of a raising through the threshold is indicates the humanoid has approached the door
        # The last occurrence of a falling through the threshold indicates the humanoid has left the door
        self.first_rising_edge_side = None
        self.first_rising_edge_time = None
        self.last_falling_edge_side = None
        self.last_falling_edge_time = None

        # This list contains the events and their timestamps as tuples (timestamp, event_name)
        self.events = list()

        # Threshold of height in mm from ground used in detection of humanoid in passage sensors
        self.th = 500.0

        # When the door is ajar (almost close) the passage sensors would detect the door.
        # To avoid this, the sensors are ignored when the door is between the closed angle and this angle (rad).
        self.ajar_door_angle_th = 0.4

        # The door is considered closed when it's absolute angle is less than this threshold (rad).
        self.closed_door_angle_th = 0.03

        # The values from passage sensors is computed taking into account the height in mm of the door.
        self.door_height = 2000.0

        # An offset, which if subtracted from the handle force value, gives us the force signal
        # e.g. force 750, offset 750: force signal = 0
        # TODO: This should either be 0, or a door calibration constant
        self.handle_force_offset = 750

        # The door is being pushed / pulled when the absolute force signal is greater than this threshold
        self.handle_force_th = 200 # TODO need more sensor data

        # We only need the first handle_is_touched event, so when it happens, this flag is set
        self.handle_has_been_touched = False

        # Publishers
        self.cw_pub = None
        self.ccw_pub = None

        # Subscribers
        self.door_sub = None
        self.cw_right_sub = None
        self.cw_left_sub = None
        self.ccw_right_sub = None
        self.ccw_left_sub = None
        self.handle_sub = None
        self.door_proximity_timesync = None
        self.events_sequence_file = None

        # Enable visualisation of passage sensors and door closing/opening events in the terminal
        self.print_debug_info = False

    def start(self, benchmark_group, robot_name, run_number, start_time, testbed_conf, preprocess_dir):
        self.events_sequence_file = preprocess_utils.open_preprocessed_csv(preprocess_dir, benchmark_group, robot_name, run_number,
                                                                           start_time, self.data_type)

        # start_time event
        start_time_split = str(testbed_conf['Start time']).split('.')
        start_time_ros = rospy.Time(int(start_time_split[0]), int(start_time_split[1]))
        self.events.append((start_time_ros, 'benchmark_start'))

        # Params
        door_node_name = rospy.get_param('door_node_name')
        passage_node_name = rospy.get_param('passage_node_name')
        handle_node_name = rospy.get_param('handle_node_name')
        output_passage_topic_name = rospy.get_param('output_passage_topic_name', 'madrob/preprocessed_data/passage')

        # Publishers
        self.cw_pub = [rospy.Publisher('/' + output_passage_topic_name + '/cw%i' % i, Float64, queue_size=1) for i in range(self.cw_num_sensors)]
        self.ccw_pub = [rospy.Publisher('ccw%i' % i, Float64, queue_size=1) for i in range(self.ccw_num_sensors)]

        # Subscribers (instantiated last to avoid registering the callbacks before the attributes of self are completely instantiated)
        self.door_sub = rospy.Subscriber('/' + door_node_name + '/state', Door, self.door_state_callback)
        self.cw_right_sub = message_filters.Subscriber('/' + passage_node_name + '/cw_right', Passage)
        self.cw_left_sub = message_filters.Subscriber('/' + passage_node_name + '/cw_left', Passage)
        self.ccw_right_sub = message_filters.Subscriber('/' + passage_node_name + '/ccw_right', Passage)
        self.ccw_left_sub = message_filters.Subscriber('/' + passage_node_name + '/ccw_left', Passage)
        self.handle_sub = rospy.Subscriber('/' + handle_node_name + '/state', Handle, self.handle_state_callback)
        self.door_proximity_timesync = message_filters.ApproximateTimeSynchronizer([self.cw_right_sub, self.cw_left_sub, self.ccw_right_sub, self.ccw_left_sub], 10, 0.1)
        self.door_proximity_timesync.registerCallback(self.door_proximity_callback)

        rospy.loginfo("Events sequence preprocess script started")

        if self.print_debug_info:
            print "\tcw0\tcw1\tcw2\tcw3\tcw4\t|\tccw0\tccw1\tccw2\tccw3\tccw4"

    def finish(self):

        try:
            self.door_sub.unregister()
            self.cw_right_sub.unregister()
            self.cw_left_sub.unregister()
            self.ccw_right_sub.unregister()
            self.ccw_left_sub.unregister()
            self.handle_sub.unregister()
            self.door_proximity_timesync = None

        except rospy.exceptions.ROSException:
            rospy.logwarn("Could not unregister from subscribers in PreprocessObject.finish in events_sequence preprocess script.")

        if self.first_rising_edge_side is not None:
            self.events.append((self.first_rising_edge_time,
                                'humanoid_approaches_the_door_on_{}_side'.format(self.first_rising_edge_side)))

        if self.last_falling_edge_side is not None:
            self.events.append(
                (self.last_falling_edge_time, 'humanoid_moves_to_{}_side'.format(self.last_falling_edge_side)))

        # sort by time (first element of the tuple)
        self.events.sort(key=lambda time_event: time_event[0])

        for time, event in self.events:
            if self.print_debug_info:
                print time, '\t', event
            self.events_sequence_file.write('%d.%d, %s\n' % (time.secs, time.nsecs, event))

        self.events_sequence_file.close()

        return self.data_type, self.events_sequence_file.name

    def door_state_callback(self, door):

        was_door_closed = -self.closed_door_angle_th < self.door_angle < self.closed_door_angle_th
        is_door_closed = -self.closed_door_angle_th < door.angle < self.closed_door_angle_th

        was_door_ajar_cw = -self.ajar_door_angle_th < self.door_angle < -self.closed_door_angle_th
        is_door_ajar_cw = -self.ajar_door_angle_th < door.angle < -self.closed_door_angle_th

        was_door_ajar_ccw = self.closed_door_angle_th < self.door_angle < self.ajar_door_angle_th
        is_door_ajar_ccw = self.closed_door_angle_th < door.angle < self.ajar_door_angle_th

        if was_door_ajar_cw and not is_door_ajar_cw:
            if self.print_debug_info:
                print "%.3f\to\to\to\to\to\t|\t" % door.angle
        if is_door_ajar_cw and not was_door_ajar_cw:
            if self.print_debug_info:
                print "%.3f\t×\t×\t×\t×\t×\t|\t" % door.angle

        if was_door_ajar_ccw and not is_door_ajar_ccw:
            if self.print_debug_info:
                print "%.3f\t\t\t\t\t\t|\to\to\to\to\to" % door.angle
        if is_door_ajar_ccw and not was_door_ajar_ccw:
            if self.print_debug_info:
                print "%.3f\t\t\t\t\t\t|\t×\t×\t×\t×\t×" % door.angle

        if was_door_closed and not is_door_closed:
            if self.print_debug_info:
                print "door opens (ajar)"
            self.events.append((door.header.stamp, 'door_opens'))
        if is_door_closed and not was_door_closed:
            if self.print_debug_info:
                print "door closes"
            self.events.append((door.header.stamp, 'door_closes'))

        self.door_angle = door.angle

    def door_proximity_callback(self, cw_right, cw_left, ccw_right, ccw_left):

        door_ajar_cw = -self.ajar_door_angle_th < self.door_angle < -self.closed_door_angle_th
        door_ajar_ccw = self.closed_door_angle_th < self.door_angle < self.ajar_door_angle_th

        new_cw = np.zeros(self.cw_num_sensors)
        cw_stamps = np.full(fill_value=None, dtype=rospy.Time, shape=(self.cw_num_sensors,))
        if not door_ajar_cw:
            new_cw[0] = self.door_height - cw_right.ranges[3].range
            new_cw[1] = self.door_height - cw_right.ranges[2].range
            new_cw[2] = self.door_height - cw_right.ranges[1].range
            new_cw[3] = self.door_height - cw_right.ranges[0].range
            new_cw[4] = self.door_height - cw_left.ranges[3].range
            cw_stamps[0:4] = cw_right.header.stamp
            cw_stamps[4] = cw_left.header.stamp

        new_ccw = np.zeros(self.ccw_num_sensors)
        ccw_stamps = np.full(fill_value=None, dtype=rospy.Time, shape=(self.ccw_num_sensors,))
        if not door_ajar_ccw:
            new_ccw[0] = self.door_height - ccw_left.ranges[3].range
            new_ccw[1] = self.door_height - ccw_left.ranges[2].range
            new_ccw[2] = self.door_height - ccw_left.ranges[1].range
            new_ccw[3] = 0.0  # new_ccw[3] = self.door_height - ccw_left.ranges[0].range  # broken sensor
            new_ccw[4] = self.door_height - ccw_right.ranges[3].range
            ccw_stamps[0:4] = ccw_left.header.stamp
            ccw_stamps[4] = ccw_right.header.stamp

        for i in range(self.cw_num_sensors):
            # rising edge on sensor cw i
            if self.cw[i] < self.th < new_cw[i]:
                if self.print_debug_info:
                    print "{angle:.3f}\t{cw_pre_tabs}↑{cw_post_tabs}\t|".format(angle=self.door_angle,
                                                                                cw_pre_tabs='\t' * i,
                                                                                cw_post_tabs='\t' * (self.cw_num_sensors - i - 1))
                if self.first_rising_edge_side is None:
                    self.first_rising_edge_side = 'cw'
                    self.first_rising_edge_time = cw_stamps[i]

            # falling edge on sensor cw i
            if self.cw[i] > self.th > new_cw[i]:
                if self.print_debug_info:
                    print "{angle:.3f}\t{cw_pre_tabs}↓{cw_post_tabs}\t|".format(angle=self.door_angle,
                                                                                cw_pre_tabs='\t' * i,
                                                                                cw_post_tabs='\t' * (self.cw_num_sensors-i-1))
                self.last_falling_edge_side = 'cw'
                self.last_falling_edge_time = cw_stamps[i]

        for i in range(self.ccw_num_sensors):
            # rising edge on sensor ccw i
            if self.ccw[i] < self.th < new_ccw[i]:
                if self.print_debug_info:
                    print "{angle:.3f}\t{cw_tabs}|{ccw_tabs}↑".format(angle=self.door_angle,
                                                                      cw_tabs='\t' * self.cw_num_sensors,
                                                                      ccw_tabs='\t' * (i + 1))
                if self.first_rising_edge_side is None:
                    self.first_rising_edge_side = 'ccw'
                    self.first_rising_edge_time = ccw_stamps[i]

            # falling edge on sensor ccw i
            if self.ccw[i] > self.th > new_ccw[i]:
                if self.print_debug_info:
                    print "{angle:.3f}\t{cw_tabs}|{ccw_tabs}↓".format(angle=self.door_angle,
                                                                      cw_tabs='\t'*self.cw_num_sensors,
                                                                      ccw_tabs='\t' * (i + 1))
                self.last_falling_edge_side = 'ccw'
                self.last_falling_edge_time = ccw_stamps[i]

        for i in range(self.cw_num_sensors):
            self.cw[i] = new_cw[i]
            self.cw_pub[i].publish(self.cw[i])

        for i in range(self.ccw_num_sensors):
            self.ccw[i] = new_ccw[i]
            self.ccw_pub[i].publish(self.ccw[i])

    def handle_state_callback(self, handle):
        force_signal = handle.force - self.handle_force_offset

        was_handle_untouched = -self.handle_force_th < self.handle_force < self.handle_force_th
        is_handle_untouched = -self.handle_force_th < force_signal < self.handle_force_th

        if was_handle_untouched and not is_handle_untouched and not self.handle_has_been_touched:
            self.events.append((handle.header.stamp, 'handle_is_touched'))
            self.handle_has_been_touched = True

        self.handle_force = force_signal