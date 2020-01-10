#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters

from madrob_msgs.msg import Door, Passage
from std_msgs.msg import Float64

class MadrobPreprocess:
    def __init__(self):
        rospy.init_node("passage_test_node")

        self.cw_right_sub = None
        self.cw_left_sub = None
        self.ccw_right_sub = None
        self.ccw_left_sub = None

        self.door_sub = None

        self.door_sub = rospy.Subscriber('/madrob/door/state', Door, self.door_state_callback)
        self.door_vel_pub = rospy.Publisher('/madrob/door/velocity_test', Float64, queue_size=1)

        self.cw_right_sub = message_filters.Subscriber('/madrob/passage/cw_right', Passage)
        self.cw_left_sub = message_filters.Subscriber('/madrob/passage/cw_left', Passage)
        self.ccw_right_sub = message_filters.Subscriber('/madrob/passage/ccw_right', Passage)
        self.ccw_left_sub = message_filters.Subscriber('/madrob/passage/ccw_left', Passage)

        self.door_proximity_timesync = message_filters.ApproximateTimeSynchronizer([self.cw_right_sub, self.cw_left_sub, self.ccw_right_sub, self.ccw_left_sub], 10, 0.1)
        self.door_proximity_timesync.registerCallback(self.door_proximity_callback)

        self.cw0_pub = rospy.Publisher('cw0', Float64, queue_size=1)
        self.cw1_pub = rospy.Publisher('cw1', Float64, queue_size=1)
        self.cw2_pub = rospy.Publisher('cw2', Float64, queue_size=1)
        self.cw3_pub = rospy.Publisher('cw3', Float64, queue_size=1)
        self.cw4_pub = rospy.Publisher('cw4', Float64, queue_size=1)

        self.ccw0_pub = rospy.Publisher('ccw0', Float64, queue_size=1)
        self.ccw1_pub = rospy.Publisher('ccw1', Float64, queue_size=1)
        self.ccw2_pub = rospy.Publisher('ccw2', Float64, queue_size=1)
        self.ccw3_pub = rospy.Publisher('ccw3', Float64, queue_size=1)
        self.ccw4_pub = rospy.Publisher('ccw4', Float64, queue_size=1)

        self.cw0 = 0.0
        self.cw1 = 0.0
        self.cw2 = 0.0
        self.cw3 = 0.0
        self.cw4 = 0.0
        self.cw5 = 0.0

        self.ccw0 = 0.0
        self.ccw1 = 0.0
        self.ccw2 = 0.0
        self.ccw3 = 0.0
        self.ccw4 = 0.0
        self.ccw5 = 0.0

        self.door_angle = 0.0

        self.last_falling_edge_side = None
        self.last_falling_edge_time = None

        self.events = list()

        self.th = 500.0
        self.ajar_door_angle_th = 0.4
        self.closed_door_angle_th = 0.03
        self.door_height = 2000.0

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
        if is_door_closed and not was_door_closed:
            print "door closes"
            self.events.append((door.header.stamp, 'door_closes'))

        self.door_angle = door.angle

    def door_proximity_callback(self, cw_right, cw_left, ccw_right, ccw_left):
        # door_closed = False
        # door_open = False
        door_ajar_cw = -self.ajar_door_angle_th < self.door_angle < -self.closed_door_angle_th
        door_ajar_ccw = self.closed_door_angle_th < self.door_angle < self.ajar_door_angle_th
        # last_falling_edge = None
        # last_falling_edge_time = None

        # if -self.closed_door_angle_th < self.door_angle < self.closed_door_angle_th:
        #     door_closed = True
        #
        # if self.closed_door_angle_th < self.door_angle < self.ajar_door_angle_th:
        #     door_ajar_ccw = True
        #
        # if -self.ajar_door_angle_th < self.door_angle < -self.closed_door_angle_th:
        #     door_ajar_cw = True
        #
        # if self.door_angle > self.ajar_door_angle_th or self.door_angle < -self.ajar_door_angle_th:
        #     door_open = True

        if door_ajar_cw:
            new_cw0 = 0.0
            new_cw1 = 0.0
            new_cw2 = 0.0
            new_cw3 = 0.0
            new_cw4 = 0.0
        else:
            new_cw0 = self.door_height - cw_right.ranges[3].range
            new_cw1 = self.door_height - cw_right.ranges[2].range
            new_cw2 = self.door_height - cw_right.ranges[1].range
            new_cw3 = self.door_height - cw_right.ranges[0].range
            new_cw4 = self.door_height - cw_left.ranges[3].range

        if door_ajar_ccw:
            new_ccw0 = 0.0
            new_ccw1 = 0.0
            new_ccw2 = 0.0
            new_ccw3 = 0.0
            new_ccw4 = 0.0
        else:
            new_ccw0 = self.door_height - ccw_left.ranges[3].range
            new_ccw1 = self.door_height - ccw_left.ranges[2].range
            new_ccw2 = self.door_height - ccw_left.ranges[1].range
            # new_ccw3 = self.door_height - ccw_left.ranges[0].range  # broken sensor
            new_ccw3 = 0.0
            new_ccw4 = self.door_height - ccw_right.ranges[3].range

        if self.cw0 < self.th < new_cw0:
            print "%.3f\t↑\t\t\t\t\t|" % self.door_angle  # rising edge cw
        if self.cw0 > self.th > new_cw0:
            print "%.3f\t↓\t\t\t\t\t|" % self.door_angle  # falling edge cw
            self.last_falling_edge_side = 'cw'
            self.last_falling_edge_time = cw_right.header.stamp

        if self.cw1 < self.th < new_cw1:
            print "%.3f\t\t↑\t\t\t\t|" % self.door_angle
        if self.cw1 > self.th > new_cw1:
            print "%.3f\t\t↓\t\t\t\t|" % self.door_angle
            self.last_falling_edge_side = 'cw'
            self.last_falling_edge_time = cw_right.header.stamp

        if self.cw2 < self.th < new_cw2:
            print "%.3f\t\t\t↑\t\t\t|" % self.door_angle
        if self.cw2 > self.th > new_cw2:
            print "%.3f\t\t\t↓\t\t\t|" % self.door_angle
            self.last_falling_edge_side = 'cw'
            self.last_falling_edge_time = cw_right.header.stamp

        if self.cw3 < self.th < new_cw3:
            print "%.3f\t\t\t\t↑\t\t|" % self.door_angle
        if self.cw3 > self.th > new_cw3:
            print "%.3f\t\t\t\t↓\t\t|" % self.door_angle
            self.last_falling_edge_side = 'cw'
            self.last_falling_edge_time = cw_right.header.stamp

        if self.cw4 < self.th < new_cw4:
            print "%.3f\t\t\t\t\t↑\t|" % self.door_angle
        if self.cw4 > self.th > new_cw4:
            print "%.3f\t\t\t\t\t↓\t|" % self.door_angle
            self.last_falling_edge_side = 'cw'
            self.last_falling_edge_time = cw_left.header.stamp


        if self.ccw0 < self.th < new_ccw0:
            print "%.3f\t\t\t\t\t\t|\t↑" % self.door_angle  # rising edge ccw
        if self.ccw0 > self.th > new_ccw0:
            print "%.3f\t\t\t\t\t\t|\t↓" % self.door_angle  # falling edge ccw
            self.last_falling_edge_side = 'ccw'
            self.last_falling_edge_time = ccw_left.header.stamp

        if self.ccw1 < self.th < new_ccw1:
            print "%.3f\t\t\t\t\t\t|\t\t↑" % self.door_angle
        if self.ccw1 > self.th > new_ccw1:
            print "%.3f\t\t\t\t\t\t|\t\t↓" % self.door_angle
            self.last_falling_edge_side = 'ccw'
            self.last_falling_edge_time = ccw_left.header.stamp

        if self.ccw2 < self.th < new_ccw2:
            print "%.3f\t\t\t\t\t\t|\t\t\t↑" % self.door_angle
        if self.ccw2 > self.th > new_ccw2:
            print "%.3f\t\t\t\t\t\t|\t\t\t↓" % self.door_angle
            self.last_falling_edge_side = 'ccw'
            self.last_falling_edge_time = ccw_left.header.stamp

        if self.ccw3 < self.th < new_ccw3:
            print "%.3f\t\t\t\t\t\t|\t\t\t\t↑" % self.door_angle
        if self.ccw3 > self.th > new_ccw3:
            print "%.3f\t\t\t\t\t\t|\t\t\t\t↓" % self.door_angle
            self.last_falling_edge_side = 'ccw'
            self.last_falling_edge_time = ccw_left.header.stamp

        if self.ccw4 < self.th < new_ccw4:
            print "%.3f\t\t\t\t\t\t|\t\t\t\t\t↑" % self.door_angle
        if self.ccw4 > self.th > new_ccw4:
            print "%.3f\t\t\t\t\t\t|\t\t\t\t\t↓" % self.door_angle
            self.last_falling_edge_side = 'ccw'
            self.last_falling_edge_time = ccw_right.header.stamp

        self.cw0 = new_cw0
        self.cw1 = new_cw1
        self.cw2 = new_cw2
        self.cw3 = new_cw3
        self.cw4 = new_cw4

        self.ccw0 = new_ccw0
        self.ccw1 = new_ccw1
        self.ccw2 = new_ccw2
        self.ccw3 = new_ccw3
        self.ccw4 = new_ccw4

        self.cw0_pub.publish(self.cw0)
        self.cw1_pub.publish(self.cw1)
        self.cw2_pub.publish(self.cw2)
        self.cw3_pub.publish(self.cw3)
        self.cw4_pub.publish(self.cw4)

        self.ccw0_pub.publish(self.ccw0)
        self.ccw1_pub.publish(self.ccw1)
        self.ccw2_pub.publish(self.ccw2)
        self.ccw3_pub.publish(self.ccw3)
        self.ccw4_pub.publish(self.ccw4)

    @staticmethod
    def run():
        try:
            r = rospy.Rate(10)
            while not rospy.is_shutdown():
                r.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass

    def end(self):
        self.events.append((self.last_falling_edge_time, 'humanoid_moves_to_{}_side'.format(self.last_falling_edge_side)))
        print self.events


passage_preprocessor = MadrobPreprocess()
passage_preprocessor.run()
passage_preprocessor.end()