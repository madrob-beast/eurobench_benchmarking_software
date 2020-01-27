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
        self.angular_acceleration_file = None
        self.benchmark_group = None
        self.robot_name = None
        self.run_number = None
        self.start_time = None

    def start(self, benchmark_group, robot_name, run_number, start_time, testbed_conf):
        self.benchmark_group = benchmark_group
        self.robot_name = robot_name
        self.run_number = run_number
        self.start_time = start_time

        door_node_name = rospy.get_param('door_node_name')
        self.door_sub = rospy.Subscriber('/' + door_node_name + '/state', Door, self.door_state_callback)

        self.angle_list = list()
        self.lp_angle_list = list()
        self.timestamp_list = list()

        rospy.loginfo("Preprocess script started: {name}".format(name=self.data_type))

    def door_state_callback(self, door):
        self.angle_list.append(door.angle)
        self.timestamp_list.append(door.header.stamp.to_sec())

    def finish(self):
        self.door_sub.unregister()

        moving_average_width = 0.2  # width of moving average window in seconds

        delta = (self.timestamp_list[-1] - self.timestamp_list[0]) / (len(self.timestamp_list) - 1)
        moving_average_size = int(np.math.ceil(moving_average_width / delta))
        df = pd.DataFrame({'time': self.timestamp_list, 'angle': self.angle_list})

        deltas = np.array(self.timestamp_list)[1:-1] - np.array(self.timestamp_list)[0:-2]
        print "delta min", np.min(deltas)
        print "delta max", np.max(deltas)
        print "delta avg", np.average(deltas)
        print "delta std", np.std(deltas)

        plt.hist(deltas)
        plt.show()

        # Apply moving average to angle
        df['angle_ma10'] = pd.rolling_mean(df['angle'], moving_average_size)

        # Compute acceleration using the Savitzky-Golay filter
        # TODO Compute values independently from topic rate for savgol filter
        # df['acc_ma10_w7_o2'] = savgol_filter(df['angle_ma10'], window_length=7, polyorder=2, deriv=2, delta=delta)
        df['vel_ma10_w11_o2'] = savgol_filter(df['angle_ma10'], window_length=11, polyorder=2, deriv=1, delta=delta)
        df['acc_ma10_w11_o2'] = savgol_filter(df['angle_ma10'], window_length=11, polyorder=2, deriv=2, delta=delta)
        # df['acc_ma10_w21_o2'] = savgol_filter(df['angle_ma10'], window_length=21, polyorder=2, deriv=2, delta=delta)
        # df['acc_ma10_w31_o2'] = savgol_filter(df['angle_ma10'], window_length=31, polyorder=2, deriv=2, delta=delta)

        angular_acceleration_list = list(df['acc_ma10_w11_o2'])

        df.plot(x='time', y=['angle', 'vel_ma10_w11_o2', 'acc_ma10_w11_o2'])
        plt.show()

        self.angular_acceleration_file = preprocess_utils.open_preprocessed_csv(self.benchmark_group, self.robot_name, self.run_number, self.start_time, self.data_type)

        for acc in angular_acceleration_list:
            self.angular_acceleration_file.write('%d.%d, %.10f\n' % (0, 0, acc))

        self.angular_acceleration_file.close()

        rospy.loginfo("Preprocess script finished: {name}".format(name=self.data_type))
        return self.data_type, self.angular_acceleration_file.name
