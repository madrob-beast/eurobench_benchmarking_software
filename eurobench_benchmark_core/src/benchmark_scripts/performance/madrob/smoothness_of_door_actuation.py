#!/usr/bin/env python
# coding=utf-8

import yaml
from os import path
import pandas as pd
import numpy as np
from benchmark_scripts.performance.base_performance import BasePerformance


class PerformanceIndicator(BasePerformance):

    def run(self, preprocessed_filenames_dict, testbed_conf, start_time):

        # Load csv as pandas DataFrame
        df = pd.read_csv(preprocessed_filenames_dict['door_angular_acceleration'], skipinitialspace=True)

        # Handle force timeseries
        a = df['door_angular_acceleration']

        # Compute result (note: the values in a are broadcasted, see google.com/search?q=numpy+broadcasting)
        smoothness_of_door_actuation = float(1. / np.sqrt(np.sum(a ** 2)))
        print 'smoothness_of_door_actuation', smoothness_of_door_actuation, 's²/rad'

        # Write result yaml file
        filepath = path.join(self.output_dir, 'smoothness_of_door_actuation_%s.yaml' % (start_time.strftime('%Y%m%d_%H%M%S')))
        with open(filepath, 'w+') as result_file:
            yaml.dump({'smoothness_of_door_actuation': smoothness_of_door_actuation}, result_file, default_flow_style=False)
