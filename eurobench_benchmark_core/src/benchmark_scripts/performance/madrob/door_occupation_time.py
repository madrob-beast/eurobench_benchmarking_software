#!/usr/bin/env python

import yaml
from os import path
import pandas as pd
from benchmark_scripts.performance.base_performance import BasePerformance

# Door Occupation Time
class PerformanceIndicator(BasePerformance):

    def run(self, preprocessed_filenames_dict, testbed_conf, start_time):
        door_occupation_time = 'TIMEOUT'

        events_df = pd.read_csv(preprocessed_filenames_dict['events_sequence'], skipinitialspace=True)

        if testbed_conf['Robot approach side'] == 'CW':
            approach_side = 'cw'
            destination_side = 'ccw'
        else:
            approach_side = 'ccw'
            destination_side = 'cw'

        # Check if the robot has moved to the destination side
        robot_moves_to_dest_events = events_df.loc[events_df['events_sequence'] == 'humanoid_moves_to_{}_side'.format(destination_side)]
        if len(robot_moves_to_dest_events) > 0:
            robot_moves_to_dest = robot_moves_to_dest_events.iloc[0]

            # Check if the robot approach event exists
            robot_approach_events = events_df.loc[events_df['events_sequence'] == 'humanoid_approaches_the_door_on_{}_side'.format(approach_side)]
            if len(robot_approach_events) > 0:
                robot_approach = robot_approach_events.iloc[0]

                # Check if 'robot moves to destination' occurs after 'robot approaches the door'
                if robot_moves_to_dest['timestamp'] > robot_approach['timestamp']:
                    door_occupation_time = float(robot_moves_to_dest['timestamp']) - float(robot_approach['timestamp'])

        # Write result yaml file
        filepath = path.join(self.output_dir, 'door_occupation_time_%s.yaml' % (start_time.strftime('%Y%m%d_%H%M%S')))
        with open(filepath, 'w+') as result_file:
            yaml.dump({'Door Occupation Time': door_occupation_time}, result_file, default_flow_style=False)