#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

from copy import deepcopy

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from benchmark_scripts.preprocess.base_preprocess import BasePreprocess
from beast_msgs.msg import Handle, Wheel

import numpy as np
import pandas as pd


def counterclockwise(a, b, c):
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])


def do_intersect(s1, s2):
    p1, p2 = s1
    p3, p4 = s2
    return counterclockwise(p1, p3, p4) != counterclockwise(p2, p3, p4) and counterclockwise(p1, p2, p3) != counterclockwise(p1, p2, p4)


class PreprocessObject(BasePreprocess):
    def __init__(self):
        super(PreprocessObject, self).__init__(data_format_name="event")

        # This list contains the events and their timestamps as tuples (timestamp, event_name).
        self.events = list()
        self.position_list = list()

        # The threshold to detect when the humanoid has touched the handle. Compared to the value from beast_msgs.msg.Handle/force, in grams.
        self.handle_force_th = 1.0  # TODO get better value after calibration

        # We only need the first handle_is_touched event, so when it happens, this flag is set.
        self.handle_has_been_touched = False
        self.x1 = None
        self.y1 = None

        # start moving threshold. The cart is considered to have started moving when any wheel angle changes more than this threshold from the initial angle.
        self.start_moving_threshold = 0.1  # radians

        # start moving variables
        self.cart_starts_moving = False
        self.left_wheel_initial_angle = None
        self.right_wheel_initial_angle = None

        # Subscribers
        self.handle_sub = None
        self.pose_sub = None
        self.left_wheel_sub = None
        self.right_wheel_sub = None

        if not rospy.has_param('checkpoints'):
            rospy.logfatal("param 'checkpoints' not set")
            rospy.signal_shutdown("param 'checkpoints' not set")

        self.checkpoints = rospy.get_param('checkpoints')
        rospy.loginfo("checkpoints: " + str(self.checkpoints))
        self.next_checkpoint_index = 0

        if not isinstance(self.checkpoints, list):
            rospy.logfatal("param 'checkpoints' should be a list")
            rospy.signal_shutdown("param 'checkpoints' should be a list")

        lists_all_the_way_down = True
        for checkpoint_segment in self.checkpoints:
            if not isinstance(checkpoint_segment, list) or len(checkpoint_segment) != 2:
                rospy.logfatal("the checkpoint segments in param 'checkpoints' should be lists of 2 items")
                rospy.signal_shutdown("the checkpoint segments in param 'checkpoints' should be lists of 2 items")
                break
            for point in checkpoint_segment:
                if not isinstance(point, list) or len(point) != 2:
                    rospy.logfatal("the points in the checkpoint segments in param 'checkpoints' should be lists of 2 items")
                    rospy.signal_shutdown("the points in the checkpoint segments in param 'checkpoints' should be lists of 2 items")
                    lists_all_the_way_down = False
                    break
            if not lists_all_the_way_down:
                break

        # checkpoint lines visualization
        self.vis_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=100)
        self.publish_marker_timer = None

    def start(self, benchmark_group, robot_name, condition_number, run_number, start_time, testbed_conf, preprocess_dir):
        self.robot_name = robot_name
        self.condition_number = condition_number
        self.run_number = run_number
        self.preprocess_dir = preprocess_dir

        # Start_time event
        self.events.append((float(testbed_conf['start_time']), 'benchmark_start'))

        pose_topic_name = '/amcl_pose'
        left_wheel_topic_name = '/beast_cart/left/wheel_status'
        right_wheel_topic_name = '/beast_cart/right/wheel_status'
        handle_topic_name = '/beast_cart/handle'

        # Subscribers (instantiated last to avoid registering the callbacks before the attributes of self are completely instantiated)
        self.pose_sub = rospy.Subscriber(pose_topic_name, PoseWithCovarianceStamped, self.pose_callback)
        self.left_wheel_sub = rospy.Subscriber(left_wheel_topic_name, Wheel, self.left_wheel_callback)
        self.right_wheel_sub = rospy.Subscriber(right_wheel_topic_name, Wheel, self.right_wheel_callback)
        self.handle_sub = rospy.Subscriber(handle_topic_name, Handle, self.handle_state_callback)
        self.publish_marker_timer = rospy.Timer(rospy.Duration.from_sec(0.1), self.publish_marker_callback)

    def publish_marker_callback(self, _):
        # publish checkpoints lines marker

        checkpoints_marker = Marker()
        checkpoints_marker.header.frame_id = "map"
        checkpoints_marker.header.stamp = rospy.Time.now()
        checkpoints_marker.ns = "checkpoints"
        checkpoints_marker.type = Marker.LINE_LIST
        checkpoints_marker.action = Marker.ADD
        checkpoints_marker.pose.orientation.w = 1
        checkpoints_marker.scale.x = 0.05
        checkpoints_marker.color.r = 0.85
        checkpoints_marker.color.g = 0.85
        checkpoints_marker.color.b = 1.0
        checkpoints_marker.color.a = 1.0

        next_checkpoint_marker = deepcopy(checkpoints_marker)
        next_checkpoint_marker.id = 1
        next_checkpoint_marker.color.r = 0.0
        next_checkpoint_marker.color.g = 0.0
        next_checkpoint_marker.color.b = 1.0

        reached_checkpoints_marker = deepcopy(checkpoints_marker)
        reached_checkpoints_marker.id = 2
        reached_checkpoints_marker.color.r = 0.0
        reached_checkpoints_marker.color.g = 1.0
        reached_checkpoints_marker.color.b = 0.0

        marker_array = MarkerArray()
        for checkpoint_index, checkpoint_segment in enumerate(self.checkpoints):

            text_checkpoints_marker = Marker()
            text_checkpoints_marker.header.frame_id = "map"
            text_checkpoints_marker.header.stamp = rospy.Time.now()
            text_checkpoints_marker.ns = "checkpoints"
            text_checkpoints_marker.action = Marker.ADD
            text_checkpoints_marker.id = checkpoint_index + 3
            text_checkpoints_marker.type = Marker.TEXT_VIEW_FACING
            text_checkpoints_marker.color.r = 1.0
            text_checkpoints_marker.color.g = 1.0
            text_checkpoints_marker.color.b = 1.0
            text_checkpoints_marker.color.a = 1.0
            text_checkpoints_marker.text = str(checkpoint_index + 1)
            text_checkpoints_marker.scale.z = 0.3
            text_checkpoints_marker.pose.position.x = (checkpoint_segment[0][0] + checkpoint_segment[1][0])/2
            text_checkpoints_marker.pose.position.y = (checkpoint_segment[0][1] + checkpoint_segment[1][1])/2
            text_checkpoints_marker.pose.orientation.w = 1
            marker_array.markers.append(text_checkpoints_marker)

            if checkpoint_index == self.next_checkpoint_index:
                for point in checkpoint_segment:
                    next_checkpoint_marker.points.append(Point(x=point[0], y=point[1]))

            if checkpoint_index < self.next_checkpoint_index:
                for point in checkpoint_segment:
                    reached_checkpoints_marker.points.append(Point(x=point[0], y=point[1]))

            if checkpoint_index > self.next_checkpoint_index:
                for point in checkpoint_segment:
                    checkpoints_marker.points.append(Point(x=point[0], y=point[1]))

        if len(checkpoints_marker.points):
            marker_array.markers.append(checkpoints_marker)
        if len(next_checkpoint_marker.points):
            marker_array.markers.append(next_checkpoint_marker)
        if len(reached_checkpoints_marker.points):
            marker_array.markers.append(reached_checkpoints_marker)

        self.vis_pub.publish(marker_array)

    def pose_callback(self, msg):
        t, x2, y2 = msg.header.stamp.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y

        trajectory = [[self.x1, self.y1], [x2, y2]]

        # if at least one pose has been received and there are still checkpoints to reach,
        # check if the trajectory between the previous and current position (x1, y1 to x2, y2) crossed the next checkpoint
        if self.x1 is not None and self.next_checkpoint_index < len(self.checkpoints):
            if do_intersect(trajectory, self.checkpoints[self.next_checkpoint_index]):
                print("reached checkpoint ", self.next_checkpoint_index + 1)
                self.events.append((t, 'checkpoint_{}'.format(self.next_checkpoint_index + 1)))
                self.next_checkpoint_index += 1

        self.x1 = x2
        self.y1 = y2

    def left_wheel_callback(self, msg):
        if not self.cart_starts_moving:
            if self.left_wheel_initial_angle is None:
                # before the cart has started moving, we need to get the initial position of the wheels
                self.left_wheel_initial_angle = msg.angle

            # check when the difference between the initial and current position is higher than the threshold
            raw_diff = np.abs(self.left_wheel_initial_angle - msg.angle)
            if min(raw_diff, np.abs(raw_diff - 2*np.pi)) > self.start_moving_threshold:
                self.cart_starts_moving = True
                print("cart starts moving")
                self.events.append((msg.header.stamp.to_sec(), 'cart_starts_moving'))

    def right_wheel_callback(self, msg):
        if not self.cart_starts_moving:
            if self.right_wheel_initial_angle is None:
                # before the cart has started moving, we need to get the initial position of the wheels
                self.right_wheel_initial_angle = msg.angle

            # check when the difference between the initial and current position is higher than the threshold
            raw_diff = np.abs(self.right_wheel_initial_angle - msg.angle)
            if min(raw_diff, np.abs(raw_diff - 2*np.pi)) > self.start_moving_threshold:
                self.cart_starts_moving = True
                print("cart starts moving")
                self.events.append((msg.header.stamp.to_sec(), 'cart_starts_moving'))

    def handle_state_callback(self, handle):
        if not self.handle_has_been_touched and np.abs(handle.force) > self.handle_force_th:
            self.handle_has_been_touched = True
            self.events.append((handle.header.stamp.to_sec(), 'handle_is_touched'))

    def finish(self):
        try:
            self.pose_sub.unregister()
            self.left_wheel_sub.unregister()
            self.right_wheel_sub.unregister()
            self.handle_sub.unregister()
            self.publish_marker_timer.shutdown()
        except rospy.exceptions.ROSException:
            rospy.logwarn("Could not unregister subscribers or timers in PreprocessObject.finish in event preprocess script.")

        # sort by time (first element of the tuple)
        self.events.sort(key=lambda time_event: time_event[0])

        preprocess_file_path = self.preprocessed_csv_file_path()
        df = pd.DataFrame(columns=['time', 'event'], data=self.events)
        df[['time', 'event']].to_csv(preprocess_file_path, index=False)

        return self.data_format_name, preprocess_file_path
