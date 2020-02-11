#!/usr/bin/env python

import rospy

from benchmark_scripts.preprocess import preprocess_utils
from benchmark_scripts.preprocess.base_preprocess import BasePreprocess
from madrob_msgs.msg import Door
from std_msgs.msg import Float64

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter


class PreprocessObject(BasePreprocess):
    def __init__(self, data_type):
        self.data_type = data_type
        self.angle_list = None
        self.lp_angle_list = None
        self.timestamp_list = None
        self.door_sub = None
        self.moving_average_width = None
        self.angular_velocity_file = None
        self.benchmark_group = None
        self.robot_name = None
        self.run_number = None
        self.start_time = None
        self.print_debug_info = False

    def start(self, benchmark_group, robot_name, run_number, start_time, testbed_conf, preprocess_dir):
        self.benchmark_group = benchmark_group
        self.robot_name = robot_name
        self.run_number = run_number
        self.start_time = start_time
        self.preprocess_dir = preprocess_dir

        # Length of moving average and Savitzky-Golay filter window in seconds
        self.moving_average_width = rospy.get_param('door_angle_filter_window_length')

        self.angle_list = list()
        self.lp_angle_list = list()
        self.timestamp_list = list()

        door_node_name = rospy.get_param('door_node_name')
        self.door_sub = rospy.Subscriber('/' + door_node_name + '/state', Door, self.door_state_callback)

        rospy.loginfo("Preprocess script started: {name}".format(name=self.data_type))

    def door_state_callback(self, door):
        self.angle_list.append(door.angle)
        self.timestamp_list.append(door.header.stamp.to_sec())

    def finish(self):
        self.door_sub.unregister()

        # Average time delta between samples
        delta = (self.timestamp_list[-1] - self.timestamp_list[0]) / (len(self.timestamp_list) - 1)

        # Number of samples for moving average and Savitzky-Golay filter given average time delta
        # It is computed as 1 + 2 * ceil(n/2) because it needs to be an odd integer
        filtering_window_length = 1 + 2 * int(np.math.ceil(self.moving_average_width / delta / 2))

        df = pd.DataFrame({'time': self.timestamp_list, 'angle': self.angle_list})

        if self.print_debug_info:
            # Statistics about time delta
            deltas = np.array(self.timestamp_list)[1:-1] - np.array(self.timestamp_list)[0:-2]
            print "delta min", np.min(deltas)
            print "delta max", np.max(deltas)
            print "delta avg", np.average(deltas)
            print "delta std", np.std(deltas)

            # Plot delta histogram
            plt.hist(deltas)
            plt.show()

        # Apply moving average to angle
        df['angle'] = df['angle'].rolling(window=filtering_window_length, center=True).mean()

        # Compute velocity using the Savitzky-Golay filter
        df['angular_velocity'] = savgol_filter(df['angle'], window_length=11, polyorder=2, deriv=1, delta=delta)
        angular_velocity_list = list(df['angular_velocity'])

        if self.print_debug_info:
            # Plot angle and velocity
            df.plot(x='time')
            plt.show()

        self.angular_velocity_file = preprocess_utils.open_preprocessed_csv(self.preprocess_dir, self.benchmark_group, self.robot_name, self.run_number, self.start_time, self.data_type)

        for time, vel in zip(self.timestamp_list, angular_velocity_list):
            if not np.math.isnan(vel):
                self.angular_velocity_file.write('%.6f, %.10f\n' % (time, vel))

        self.angular_velocity_file.close()

        rospy.loginfo("Preprocess script finished: {name}".format(name=self.data_type))
        return self.data_type, self.angular_velocity_file.name
