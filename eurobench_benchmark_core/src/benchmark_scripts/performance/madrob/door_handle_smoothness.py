#!/usr/bin/env python

import yaml
from os import path
import pandas as pd
import numpy as np
from benchmark_scripts.performance.base_performance import BasePerformance

# Door Handle Smoothness
class PerformanceIndicator(BasePerformance):

    def run(self, preprocessed_filenames_dict, testbed_configuration_filepath, start_time):
        with open(testbed_configuration_filepath, 'r') as file:
            self.testbed_conf = yaml.load(file)

        force_df = pd.read_csv(preprocessed_filenames_dict['handle_force'])
        
        force_delta_sum = 0

        for index, row in force_df.iterrows():
            if index > 0:
                force_delta_sum += abs(row['force'] - force_df.loc[index-1, 'force']) / (row['timestamp'] - force_df.loc[index-1, 'timestamp'])

        smoothness = 10000 / (force_delta_sum/len(force_df)) # Higher smoothness = lower force deltas

        # Write result yaml file
        filepath = path.join(self.output_dir, 'door_handle_smoothness_%s.yaml' % (start_time.strftime('%Y%m%d_%H%M%S')))
        with open(filepath, 'w+') as result_file:
            yaml.dump({'Door Handle Smoothness': float(smoothness)}, result_file, default_flow_style=False)