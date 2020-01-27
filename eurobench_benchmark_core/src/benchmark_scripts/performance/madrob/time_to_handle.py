#!/usr/bin/env python

import yaml
from os import path
import pandas as pd
from benchmark_scripts.performance.base_performance import BasePerformance

# Time to Handle
class PerformanceIndicator(BasePerformance):

    def run(self, preprocessed_filenames_dict, testbed_conf, start_time):
        time_to_handle = 'TIMEOUT'

        events_df = pd.read_csv(preprocessed_filenames_dict['events_sequence'], skipinitialspace=True)

        # Check if there's a 'handle_is_touched' event
        handle_is_touched_events = events_df.loc[events_df['events_sequence'] == 'handle_is_touched']
        if len(handle_is_touched_events) > 0:
            first_handle_touch = handle_is_touched_events.iloc[0]

            time_to_handle = float(first_handle_touch['timestamp']) - float(events_df.loc[events_df['events_sequence'] == 'benchmark_start']['timestamp'])

        # Write result yaml file
        filepath = path.join(self.output_dir, 'time_to_handle_%s.yaml' % (start_time.strftime('%Y%m%d_%H%M%S')))
        with open(filepath, 'w+') as result_file:
            yaml.dump({'Time to Handle': time_to_handle}, result_file, default_flow_style=False)