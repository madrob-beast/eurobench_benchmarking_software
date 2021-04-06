#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from benchmark_scripts.preprocess.base_preprocess import BasePreprocess
from madrob_msgs.msg import Door
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter


class PreprocessObject(BasePreprocess):
    def __init__(self):
        super(PreprocessObject, self).__init__(data_format_name="jointState")
        self.angle_list = None
        self.lp_angle_list = None
        self.timestamp_list = None
        self.door_sub = None
        self.moving_average_width = None
        self.print_debug_info = False

    def start(self, benchmark_group, robot_name, condition_number, run_number, start_time, testbed_conf, preprocess_dir):
        self.robot_name = robot_name
        self.condition_number = condition_number
        self.run_number = run_number
        self.preprocess_dir = preprocess_dir

        # Length of moving average and Savitzky-Golay filter window in seconds
        self.moving_average_width = rospy.get_param('door_angle_filter_window_length')

        self.angle_list = list()
        self.lp_angle_list = list()
        self.timestamp_list = list()

        self.door_sub = rospy.Subscriber('/madrob/door/state', Door, self.door_state_callback)

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

        df = pd.DataFrame({'time': self.timestamp_list, 'position': self.angle_list})

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
        df['position_filtered'] = df['position'].rolling(window=filtering_window_length, center=True).mean()

        # Compute velocity using the Savitzky-Golay filter
        df['velocity'] = savgol_filter(df['position_filtered'], window_length=11, polyorder=2, deriv=1, delta=delta)

        # Compute acceleration using the Savitzky-Golay filter
        df['acceleration'] = savgol_filter(df['position_filtered'], window_length=11, polyorder=2, deriv=2, delta=delta)

        if self.print_debug_info:
            # Plot angle and acceleration
            df.plot(x='time')
            plt.show()

        preprocess_file_path = self.preprocessed_csv_file_path()

        df[['time', 'position', 'velocity', 'acceleration']].to_csv(preprocess_file_path, index=False)

        return self.data_format_name, preprocess_file_path
