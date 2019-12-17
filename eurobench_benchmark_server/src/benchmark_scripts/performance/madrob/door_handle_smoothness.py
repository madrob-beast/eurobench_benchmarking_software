#!/usr/bin/env python

import yaml
from os import path
import pandas as pd
import numpy as np
from benchmark_scripts.performance.base_performance import BasePerformance

# Door Handle Smoothness
class PerformanceIndicator(BasePerformance):

    def run(self, preprocessed_filenames_dict, start_time):
        force_df = pd.read_csv(preprocessed_filenames_dict['handle_force'])
        forces = force_df['force'].values

        sum_squared_forces = np.sum(forces**2) # Sum of squared forces
        smoothness = 100 / sum_squared_forces # Higher smoothness = lower squared forces

        # Write result yaml file
        filepath = path.join(self.output_dir, 'door_handle_smoothness_%s.yaml' % (start_time.strftime('%Y%m%d_%H%M%S')))
        with open(filepath, 'w+') as result_file:
            yaml.dump({'Door Handle Smoothness': float(smoothness)}, result_file, default_flow_style=False)