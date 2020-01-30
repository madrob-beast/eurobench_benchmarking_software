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
        df = pd.read_csv(preprocessed_filenames_dict['handle_force'], skipinitialspace=True)

        # Handle force timeseries
        f = df['handle_force']

        # Compute result (note: the values in f are broadcasted, see google.com/search?q=numpy+broadcasting)
        roughness_of_door_actuation = float(np.max(np.abs(f)))
        print 'roughness_of_door_actuation', roughness_of_door_actuation, 'grams'

        # Write result yaml file
        filepath = path.join(self.output_dir, 'roughness_of_door_actuation_%s.yaml' % (start_time.strftime('%Y%m%d_%H%M%S')))
        with open(filepath, 'w+') as result_file:
            yaml.dump({'roughness_of_door_actuation': roughness_of_door_actuation}, result_file, default_flow_style=False)
