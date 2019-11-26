#!/usr/bin/env python

import rospy
import time
import yaml
from madrob_door_control import MadrobDoorControl
from base_benchmark import BaseBenchmark
from std_msgs.msg import Bool
from os import path
from datetime import datetime

class BenchmarkObject(BaseBenchmark):

    benchmark_code = 'MADROB_TRAVERSAL'

    # This benchmark script checks if a robot is able to pass through the door. It's valid for any door configuration.

    def execute(self):
        self.door_obstructed = False
        self.passed_through_door = False

        # Setup door based on the currently selected benchmark type
        door_control = MadrobDoorControl()
        door_control.setup_door()

        # Save testbed setup
        door_params = door_control.get_door_parameters()
        self.write_testbed_conf_file(door_params)

        # TODO subscribe to IR sensors, OR use madrob_door_control function
        #rospy.Subscriber('/eurobench_sensors/door_laser',
        #                 Bool, self.door_laser_callback)

        self.result['passed_through_door'] = False
        self.result['score'] = 0
        while not self.terminated:
            if self.passed_through_door:
                self.result['passed_through_door'] = True

                # Add the door_traversal_score, set in benchmark_config/TEST_BENCHMARK.yaml
                self.result['score'] += self.config['door_traversal_score']

                # Finish benchmark
                return

    def door_laser_callback(self, msg):
        if msg.data: # Sensor returned true: door obstructed
            self.door_obstructed = True
        else: # Door not obstructed: if it was obstructed before, assume robot passed through
            if self.door_obstructed:
                self.passed_through_door = True
            self.door_obstructed = False
    
    def write_testbed_conf_file(self, testbed_conf):
        output_dir = path.expanduser(rospy.get_param('benchmark_output_directory'))

        subject_number = self.robot_name # TODO this should be a number/id
        run_number = self.run_number
        date = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = 'subject_%s_door_%d_%s.yaml' % (subject_number, run_number, date)

        with open(path.join(output_dir, filename), 'w') as outfile:
            yaml.dump(testbed_conf, outfile, default_flow_style=False)
