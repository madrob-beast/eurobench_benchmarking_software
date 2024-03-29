#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import rospy
import message_filters
from benchmark_scripts.preprocess.base_preprocess import BasePreprocess
from madrob_msgs.msg import Door, Passage, Handle
from std_msgs.msg import Float64

import numpy as np


class PreprocessObject(BasePreprocess):
    def __init__(self):
        super(PreprocessObject, self).__init__(data_format_name="event")

        # Passage sensors values (a subset of the total number of sensors available from the testbed).
        self.cw_num_sensors = 5
        self.ccw_num_sensors = 5
        self.previous_cw = np.zeros(self.cw_num_sensors)
        self.previous_ccw = np.zeros(self.ccw_num_sensors)

        # Door angular position.
        self.door_angle = 0.0

        # Handle force signal.
        self.handle_force = 0.0

        # Passage is detected by checking when the height of the passage sensors becomes higher than a threshold.
        # The first occurrence of the height raising through the threshold indicates the humanoid has approached the door.
        # The last occurrence of the height falling through the threshold indicates the humanoid has left the door.
        self.first_rising_edge_side = None
        self.first_rising_edge_time = None
        self.last_falling_edge_side = None
        self.last_falling_edge_time = None

        # This list contains the events and their timestamps as tuples (timestamp, event_name).
        self.events = list()

        # Threshold of height in mm from ground used for detection of humanoid in passage sensors.
        self.height_th = 1000.0

        # When the door is ajar (almost close) the passage sensors would detect the door. To avoid this, each sensor is
        # ignored when the door is between the min and max angles (rad, absolute value). These values are obtained by
        # looking into the test datasets and picking the minimum and maximum angle values for which the door would be
        # wrongly detected as humanoid in each sensor. This measurement is made with a height threshold of 500mm.
        # A margin is applied to the measured angles: 50% for the minimum, 112.5% for the maximum.
        margin_min = 0.5
        margin_max = 1.125
        self.passage_disable_zone_cw_min = np.array([0.087, 0.097, 0.097, 0.133, 0.191])*margin_min
        self.passage_disable_zone_cw_max = np.array([0.274, 0.324, 0.371, 0.461, 0.646])*margin_max
        self.passage_disable_zone_ccw_min = np.array([0.087, 0.097, 0.097, 0.133, 0.191])*margin_min
        self.passage_disable_zone_ccw_max = np.array([0.408, 0.370, 0.371, 0.461, 0.646])*margin_max

        # The door is considered closed when its absolute angle is less than this threshold (rad).
        self.closed_door_angle_th = 0.03

        # The values from passage sensors is computed taking into account the height in mm of the door.
        self.door_height = 2000.0

        # An offset, which if subtracted from the handle force value, gives us the force signal.
        # e.g. force 750, offset 750: force signal = 0
        self.handle_force_offset = 0

        # The door is being pushed / pulled when the absolute force signal (grams) is greater than this threshold.
        self.handle_force_th = 200

        # We only need the first handle_is_touched event, so when it happens, this flag is set.
        self.handle_has_been_touched = False

        # Publishers
        self.door_pub = None
        self.cw_pub = None
        self.ccw_pub = None
        self.cw_raw_pub = None
        self.ccw_raw_pub = None

        # Subscribers
        self.door_sub = None
        self.cw_right_sub = None
        self.cw_left_sub = None
        self.ccw_right_sub = None
        self.ccw_left_sub = None
        self.handle_sub = None
        self.door_proximity_timesync = None

        # Enable visualisation of passage sensors and door closing/opening events in the terminal.
        self.print_debug_info = False

    def start(self, benchmark_group, robot_name, condition_number, run_number, start_time, testbed_conf, preprocess_dir):
        self.robot_name = robot_name
        self.condition_number = condition_number
        self.run_number = run_number
        self.preprocess_dir = preprocess_dir

        # Start_time event
        start_time_ros = rospy.Time().from_sec(testbed_conf['start_time'])
        self.events.append((start_time_ros, 'benchmark_start'))

        # Params
        output_passage_topic_name = rospy.get_param('output_passage_topic_name', 'madrob/preprocessed_data/passage')

        # Publishers
        self.door_pub = rospy.Publisher('/' + output_passage_topic_name + '/door', Float64, queue_size=1)
        self.cw_pub = [rospy.Publisher('/' + output_passage_topic_name + '/cw%i' % i, Float64, queue_size=1) for i in range(self.cw_num_sensors)]
        self.ccw_pub = [rospy.Publisher('/' + output_passage_topic_name + '/ccw%i' % i, Float64, queue_size=1) for i in range(self.ccw_num_sensors)]
        self.cw_raw_pub = [rospy.Publisher('/' + output_passage_topic_name + '/dcw%i' % i, Float64, queue_size=1) for i in range(self.cw_num_sensors)]
        self.ccw_raw_pub = [rospy.Publisher('/' + output_passage_topic_name + '/dccw%i' % i, Float64, queue_size=1) for i in range(self.ccw_num_sensors)]

        # Subscribers (instantiated last to avoid registering the callbacks before the attributes of self are completely instantiated)
        self.door_sub = rospy.Subscriber('/madrob/door/state', Door, self.door_state_callback)
        self.cw_right_sub = message_filters.Subscriber('/madrob/passage/cw_right', Passage)
        self.cw_left_sub = message_filters.Subscriber('/madrob/passage/cw_left', Passage)
        self.ccw_right_sub = message_filters.Subscriber('/madrob/passage/ccw_right', Passage)
        self.ccw_left_sub = message_filters.Subscriber('/madrob/passage/ccw_left', Passage)
        self.handle_sub = rospy.Subscriber('/madrob/handle/state', Handle, self.handle_state_callback)
        self.door_proximity_timesync = message_filters.ApproximateTimeSynchronizer([self.cw_right_sub, self.cw_left_sub, self.ccw_right_sub, self.ccw_left_sub], 10, 0.1)
        self.door_proximity_timesync.registerCallback(self.door_proximity_callback)

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
            self.events.append((self.last_falling_edge_time,
                                'humanoid_moves_to_{}_side'.format(self.last_falling_edge_side)))

        # sort by time (first element of the tuple)
        self.events.sort(key=lambda time_event: time_event[0])

        preprocess_file_path = self.preprocessed_csv_file_path()
        df = pd.DataFrame(columns=['time', 'event'], data=self.events)
        df['time'] = map(lambda ros_time: ros_time.to_sec(), df['time'].iloc[:])

        df[['time', 'event']].to_csv(preprocess_file_path, index=False)

        if self.print_debug_info:
            print df

        return self.data_format_name, preprocess_file_path

    def door_state_callback(self, door):

        was_door_closed = -self.closed_door_angle_th < self.door_angle < self.closed_door_angle_th
        is_door_closed = -self.closed_door_angle_th < door.angle < self.closed_door_angle_th

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

        cw_stamps = np.full(fill_value=None, dtype=rospy.Time, shape=(self.cw_num_sensors,))
        cw_stamps[0:4] = cw_right.header.stamp
        cw_stamps[4] = cw_left.header.stamp

        raw_cw = np.zeros(self.cw_num_sensors)
        raw_cw[0] = self.door_height - cw_right.ranges[3].range
        raw_cw[1] = self.door_height - cw_right.ranges[2].range
        raw_cw[2] = self.door_height - cw_right.ranges[1].range
        raw_cw[3] = self.door_height - cw_right.ranges[0].range
        raw_cw[4] = self.door_height - cw_left.ranges[3].range

        latest_cw = np.zeros(self.cw_num_sensors)
        for i in range(self.cw_num_sensors):
            if not (-self.passage_disable_zone_cw_max[i] < self.door_angle < -self.passage_disable_zone_cw_min[i]):
                latest_cw[i] = raw_cw[i]

        ccw_stamps = np.full(fill_value=None, dtype=rospy.Time, shape=(self.ccw_num_sensors,))
        ccw_stamps[0:4] = ccw_left.header.stamp
        ccw_stamps[4] = ccw_right.header.stamp

        raw_ccw = np.zeros(self.ccw_num_sensors)
        raw_ccw[0] = self.door_height - ccw_left.ranges[3].range
        raw_ccw[1] = self.door_height - ccw_left.ranges[2].range
        raw_ccw[2] = self.door_height - ccw_left.ranges[1].range
        raw_ccw[3] = self.door_height - ccw_left.ranges[0].range
        raw_ccw[4] = self.door_height - ccw_right.ranges[0].range

        latest_ccw = np.zeros(self.ccw_num_sensors)
        for i in range(self.ccw_num_sensors):
            if not (self.passage_disable_zone_ccw_min[i] < self.door_angle < self.passage_disable_zone_ccw_max[i]):
                latest_ccw[i] = raw_ccw[i]

        for i in range(self.cw_num_sensors):
            # rising edge on sensor cw i
            if self.previous_cw[i] < self.height_th < latest_cw[i]:
                if self.print_debug_info:
                    print "{angle:.3f}\t{cw_pre_tabs}↑{cw_post_tabs}\t|".format(angle=self.door_angle,
                                                                                cw_pre_tabs='\t' * i,
                                                                                cw_post_tabs='\t' * (self.cw_num_sensors - i - 1))
                if self.first_rising_edge_side is None:
                    if cw_stamps[i] is not None:
                        self.first_rising_edge_side = 'cw'
                        self.first_rising_edge_time = cw_stamps[i]
                    else:
                        rospy.logwarn("[events_sequence preprocess data script] cw_stamps contains Nones")

            # falling edge on sensor cw i
            if self.previous_cw[i] > self.height_th > latest_cw[i]:
                if self.print_debug_info:
                    print "{angle:.3f}\t{cw_pre_tabs}↓{cw_post_tabs}\t|".format(angle=self.door_angle,
                                                                                cw_pre_tabs='\t' * i,
                                                                                cw_post_tabs='\t' * (self.cw_num_sensors-i-1))

                if cw_stamps[i] is not None:
                    self.last_falling_edge_side = 'cw'
                    self.last_falling_edge_time = cw_stamps[i]
                else:
                    rospy.logwarn("[events_sequence preprocess data script] cw_stamps contains Nones")

        for i in range(self.ccw_num_sensors):
            # rising edge on sensor ccw i
            if self.previous_ccw[i] < self.height_th < latest_ccw[i]:
                if self.print_debug_info:
                    print "{angle:.3f}\t{cw_tabs}|{ccw_tabs}↑".format(angle=self.door_angle,
                                                                      cw_tabs='\t' * self.cw_num_sensors,
                                                                      ccw_tabs='\t' * (i + 1))
                if self.first_rising_edge_side is None:
                    if ccw_stamps[i] is not None:
                        self.first_rising_edge_side = 'ccw'
                        self.first_rising_edge_time = ccw_stamps[i]
                    else:
                        rospy.logwarn("[events_sequence preprocess data script] ccw_stamps contains Nones")

            # falling edge on sensor ccw i
            if self.previous_ccw[i] > self.height_th > latest_ccw[i]:
                if self.print_debug_info:
                    print "{angle:.3f}\t{cw_tabs}|{ccw_tabs}↓".format(angle=self.door_angle,
                                                                      cw_tabs='\t'*self.cw_num_sensors,
                                                                      ccw_tabs='\t' * (i + 1))
                if ccw_stamps[i] is not None:
                    self.last_falling_edge_side = 'ccw'
                    self.last_falling_edge_time = ccw_stamps[i]
                else:
                    rospy.logwarn("[events_sequence preprocess data script] ccw_stamps contains Nones")

        self.door_pub.publish(np.abs(self.door_angle) * 100)

        for i in range(self.cw_num_sensors):
            self.previous_cw[i] = latest_cw[i]
            self.cw_pub[i].publish(self.previous_cw[i])
            self.cw_raw_pub[i].publish(raw_cw[i])

        for i in range(self.ccw_num_sensors):
            self.previous_ccw[i] = latest_ccw[i]
            self.ccw_pub[i].publish(self.previous_ccw[i])
            self.ccw_raw_pub[i].publish(raw_ccw[i])

    def handle_state_callback(self, handle):
        force_signal = handle.force - self.handle_force_offset

        was_handle_untouched = -self.handle_force_th < self.handle_force < self.handle_force_th
        is_handle_untouched = -self.handle_force_th < force_signal < self.handle_force_th

        if was_handle_untouched and not is_handle_untouched and not self.handle_has_been_touched:
            self.events.append((handle.header.stamp, 'handle_is_touched'))
            self.handle_has_been_touched = True

        self.handle_force = force_signal
