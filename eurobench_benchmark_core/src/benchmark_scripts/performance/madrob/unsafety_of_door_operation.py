#!/usr/bin/env python
# coding=utf-8

import yaml
from os import path
import pandas as pd
import numpy as np
from scipy.signal import iirfilter, lfilter

from benchmark_scripts.performance.base_performance import BasePerformance


class PerformanceIndicator(BasePerformance):

    def run(self, preprocessed_filenames_dict, testbed_conf, start_time):

        critical_frequency = 2.0  # Hz
        filter_order = 5
        filter_type = 'butter'

        # Load csv as pandas DataFrame
        df = pd.read_csv(preprocessed_filenames_dict['door_angular_acceleration'], skipinitialspace=True)

        # Handle force timeseries
        acceleration = df['door_angular_acceleration']
        delta = (df['timestamp'][df.index[-1]] - df['timestamp'][0]) / (len(df['timestamp']) - 1)

        # Low-pass filter
        nyquist_frequency = 1./delta/2
        normalised_critical_frequency = critical_frequency / nyquist_frequency
        print 'nyquist_frequency', nyquist_frequency
        print 'normalised_critical_frequency', normalised_critical_frequency
        b, a = iirfilter(5, 0.5, btype='lowpass', ftype=filter_type)

        # Low-pass filtered acceleration
        a = lfilter(b, a, acceleration)

        # Compute result (note: the values in a are broadcasted, see google.com/search?q=numpy+broadcasting)
        unsafety_of_door_operation = float(np.max(np.abs(a)))
        print 'unsafety_of_door_operation', unsafety_of_door_operation, 'rad/s²'

        # Write result yaml file
        filepath = path.join(self.output_dir, 'unsafety_of_door_operation_%s.yaml' % (start_time.strftime('%Y%m%d_%H%M%S')))
        with open(filepath, 'w+') as result_file:
            yaml.dump({'unsafety_of_door_operation': unsafety_of_door_operation}, result_file, default_flow_style=False)
