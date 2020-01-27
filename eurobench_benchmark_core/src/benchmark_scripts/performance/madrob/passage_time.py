#!/usr/bin/env python

import yaml
from os import path
import pandas as pd
from benchmark_scripts.performance.base_performance import BasePerformance

# Passage Time
class PerformanceIndicator(BasePerformance):

    def run(self, preprocessed_filenames_dict, testbed_conf, start_time):
        passage_time = 'TIMEOUT'

        events_df = pd.read_csv(preprocessed_filenames_dict['events_sequence'], skipinitialspace=True)

        if testbed_conf['Robot approach side'] == 'CW':
            destination_side = 'ccw'
        else:
            destination_side = 'cw'

        # Check if there's a 'door_close' event
        door_closes_events = events_df.loc[events_df['events_sequence'] == 'door_closes']
        if len(door_closes_events) > 0:
            last_door_close = door_closes_events.iloc[-1]

            # Check if the robot has moved to the destination side
            robot_moves_to_dest_events = events_df.loc[events_df['events_sequence'] == 'humanoid_moves_to_{}_side'.format(destination_side)]
            if len(robot_moves_to_dest_events) > 0:
                robot_moves_to_dest = robot_moves_to_dest_events.iloc[0]

                # Check if the last door closing event occurs after the robot moves to destination
                if last_door_close['timestamp'] > robot_moves_to_dest['timestamp']:
                    # Finally, check if the 'handle_is_touched' event exists
                    handle_is_touched_events = events_df.loc[events_df['events_sequence'] == 'handle_is_touched']
                    if len(handle_is_touched_events) > 0:
                        first_handle_touch = handle_is_touched_events.iloc[0]

                        passage_time = float(last_door_close['timestamp']) - float(first_handle_touch['timestamp'])

        # Write result yaml file
        filepath = path.join(self.output_dir, 'passage_time_%s.yaml' % (start_time.strftime('%Y%m%d_%H%M%S')))
        with open(filepath, 'w+') as result_file:
            yaml.dump({'Passage Time': passage_time}, result_file, default_flow_style=False)