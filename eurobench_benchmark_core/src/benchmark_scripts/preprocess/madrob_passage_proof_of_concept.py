#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters

from madrob_msgs.msg import Door, Passage
from std_msgs.msg import Float64

from collections import deque
import numpy as np

class MadrobPreprocess:
    def __init__(self):
        rospy.init_node("passage_test_node")

        self.cw_num_sensors = 5
        self.ccw_num_sensors = 5
        self.cw = np.zeros(self.cw_num_sensors)
        self.ccw = np.zeros(self.ccw_num_sensors)

        self.previous_door_angle_msgs = deque()
        self.previous_angular_velocity = None
        self.previous_angular_velocity_stamp = None
        self.previous_angular_acceleration = None
        self.previous_angular_acceleration_stamp = None

        self.door_angle = 0.0

        self.last_falling_edge_side = None
        self.last_falling_edge_time = None

        self.first_rising_edge_side = None
        self.first_rising_edge_time = None

        self.events = list()

        self.th = 500.0
        self.ajar_door_angle_th = 0.4
        self.closed_door_angle_th = 0.03
        self.door_height = 2000.0

        # test moving average
        self.ccw1_clean = deque()
        self.cw1_clean = deque()
        self.moving_average_width = 6
        self.cw1_clean_pub = rospy.Publisher('cw1_clean', Float64, queue_size=1)
        self.ccw1_clean_pub = rospy.Publisher('ccw1_clean', Float64, queue_size=1)
        # END test moving average

        # Publishers
        self.angular_velocity_pub = rospy.Publisher('/madrob/door/velocity_test', Float64, queue_size=1)
        self.angular_acceleration_pub = rospy.Publisher('/madrob/door/acceleration_test', Float64, queue_size=1)
        self.cw_pub = [rospy.Publisher('cw%i' % i, Float64, queue_size=1) for i in range(self.cw_num_sensors)]
        self.ccw_pub = [rospy.Publisher('ccw%i' % i, Float64, queue_size=1) for i in range(self.ccw_num_sensors)]

        # Subscribers (last in __init__ to avoid registering the callbacks before the attributes of self are completely instantiated)
        self.door_sub = rospy.Subscriber('/madrob/door/state', Door, self.door_state_callback)
        self.cw_right_sub = message_filters.Subscriber('/madrob/passage/cw_right', Passage)
        self.cw_left_sub = message_filters.Subscriber('/madrob/passage/cw_left', Passage)
        self.ccw_right_sub = message_filters.Subscriber('/madrob/passage/ccw_right', Passage)
        self.ccw_left_sub = message_filters.Subscriber('/madrob/passage/ccw_left', Passage)
        self.door_proximity_timesync = message_filters.ApproximateTimeSynchronizer([self.cw_right_sub, self.cw_left_sub, self.ccw_right_sub, self.ccw_left_sub], 10, 0.1)
        self.door_proximity_timesync.registerCallback(self.door_proximity_callback)


        # cw  → negative angles
        # ccw → positive angles
        print "\tcw0\tcw1\tcw2\tcw3\tcw4\t|\tccw0\tccw1\tccw2\tccw3\tccw4"

    def door_state_callback(self, door):

        was_door_closed = -self.closed_door_angle_th < self.door_angle < self.closed_door_angle_th
        is_door_closed = -self.closed_door_angle_th < door.angle < self.closed_door_angle_th

        was_door_ajar_cw = -self.ajar_door_angle_th < self.door_angle < -self.closed_door_angle_th
        is_door_ajar_cw = -self.ajar_door_angle_th < door.angle < -self.closed_door_angle_th

        was_door_ajar_ccw = self.closed_door_angle_th < self.door_angle < self.ajar_door_angle_th
        is_door_ajar_ccw = self.closed_door_angle_th < door.angle < self.ajar_door_angle_th

        if was_door_ajar_cw and not is_door_ajar_cw:
            print "%.3f\to\to\to\to\to\t|\t" % door.angle
        if is_door_ajar_cw and not was_door_ajar_cw:
            print "%.3f\t×\t×\t×\t×\t×\t|\t" % door.angle

        if was_door_ajar_ccw and not is_door_ajar_ccw:
            print "%.3f\t\t\t\t\t\t|\to\to\to\to\to" % door.angle
        if is_door_ajar_ccw and not was_door_ajar_ccw:
            print "%.3f\t\t\t\t\t\t|\t×\t×\t×\t×\t×" % door.angle

        if was_door_closed and not is_door_closed:
            print "door opens (ajar)"
            self.events.append((door.header.stamp, 'door_opens'))
        if is_door_closed and not was_door_closed:
            print "door closes"
            self.events.append((door.header.stamp, 'door_closes'))

        self.door_angle = door.angle

        # test compute velocity and acceleration
        current_angular_velocity = None
        current_angular_velocity_stamp = None
        if len(self.previous_door_angle_msgs) >= 10:
            previous_angular_velocity_msg = self.previous_door_angle_msgs.popleft()

            current_angular_velocity = (previous_angular_velocity_msg.angle - door.angle) / (previous_angular_velocity_msg.header.stamp - door.header.stamp).to_sec()
            current_angular_velocity_stamp = door.header.stamp + (door.header.stamp - previous_angular_velocity_msg.header.stamp) * 0.5

            self.angular_velocity_pub.publish(Float64(data=current_angular_velocity))

        self.previous_door_angle_msgs.append(door)

        # acceleration
        if current_angular_velocity is not None and self.previous_angular_velocity is not None:
            current_angular_acceleration = (self.previous_angular_velocity - current_angular_velocity) / (self.previous_angular_velocity_stamp - current_angular_velocity_stamp).to_sec()
            current_angular_acceleration_stamp = current_angular_velocity_stamp + (current_angular_velocity_stamp - self.previous_angular_velocity_stamp) * 0.5  # (self.previous_angular_velocity.header.stamp + current_angular_velocity_stamp) / 2

            self.angular_acceleration_pub.publish(Float64(data=current_angular_acceleration))

        self.previous_angular_velocity = current_angular_velocity
        self.previous_angular_velocity_stamp = current_angular_velocity_stamp
        # END test compute velocity and acceleration

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

        # test moving average
        while len(self.cw1_clean) >= self.moving_average_width:
            self.cw1_clean.popleft()
        self.cw1_clean.append(new_cw[1])
        self.cw1_clean_pub.publish(sum(self.cw1_clean)/len(self.cw1_clean))

        while len(self.ccw1_clean) >= self.moving_average_width:
            self.ccw1_clean.popleft()
        self.ccw1_clean.append(new_ccw[1])
        self.ccw1_clean_pub.publish(sum(self.ccw1_clean)/len(self.ccw1_clean))
        # END test moving average

        for i in range(self.cw_num_sensors):
            # rising edge on sensor cw i
            if self.cw[i] < self.th < new_cw[i]:
                print "{angle:.3f}\t{cw_pre_tabs}↑{cw_post_tabs}\t|".format(angle=self.door_angle,
                                                                            cw_pre_tabs='\t' * i,
                                                                            cw_post_tabs='\t' * (self.cw_num_sensors - i - 1))
                if self.first_rising_edge_side is None:
                    self.first_rising_edge_side = 'cw'
                    self.first_rising_edge_time = cw_stamps[i]

            # falling edge on sensor cw i
            if self.cw[i] > self.th > new_cw[i]:
                print "{angle:.3f}\t{cw_pre_tabs}↓{cw_post_tabs}\t|".format(angle=self.door_angle,
                                                                            cw_pre_tabs='\t' * i,
                                                                            cw_post_tabs='\t' * (self.cw_num_sensors-i-1))
                self.last_falling_edge_side = 'cw'
                self.last_falling_edge_time = cw_stamps[i]

        for i in range(self.ccw_num_sensors):
            # rising edge on sensor ccw i
            if self.ccw[i] < self.th < new_ccw[i]:
                print "{angle:.3f}\t{cw_tabs}|{ccw_tabs}↑".format(angle=self.door_angle,
                                                                  cw_tabs='\t' * self.cw_num_sensors,
                                                                  ccw_tabs='\t' * (i + 1))
                if self.first_rising_edge_side is None:
                    self.first_rising_edge_side = 'ccw'
                    self.first_rising_edge_time = ccw_stamps[i]

            if self.ccw[i] > self.th > new_ccw[i]:
                print "{angle:.3f}\t{cw_tabs}|{ccw_tabs}↓".format(angle=self.door_angle,
                                                                  cw_tabs='\t'*self.cw_num_sensors,
                                                                  ccw_tabs='\t' * (i + 1))
                self.last_falling_edge_side = 'ccw'
                self.last_falling_edge_time = ccw_stamps[i]

        for i in range(self.cw_num_sensors):
            self.cw[i] = new_cw[i]

        for i in range(self.ccw_num_sensors):
            self.ccw[i] = new_ccw[i]

        for i in range(self.cw_num_sensors):
            self.cw_pub[i].publish(self.cw[i])

        for i in range(self.ccw_num_sensors):
            self.ccw_pub[i].publish(self.ccw[i])


    @staticmethod
    def run():
        try:
            r = rospy.Rate(10)
            while not rospy.is_shutdown():
                r.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass

    def end(self):
        if self.first_rising_edge_side is not None:
            self.events.append((self.first_rising_edge_time, 'humanoid_approaches_the_door_on_{}_side'.format(self.first_rising_edge_side)))

        if self.last_falling_edge_side is not None:
            self.events.append((self.last_falling_edge_time, 'humanoid_moves_to_{}_side'.format(self.last_falling_edge_side)))

        # sort by time (first element of the tuple)
        self.events.sort(key=lambda time_event: time_event[0])

        for time, event in self.events:
            print time, '\t', event


passage_preprocessor = MadrobPreprocess()
passage_preprocessor.run()
passage_preprocessor.end()