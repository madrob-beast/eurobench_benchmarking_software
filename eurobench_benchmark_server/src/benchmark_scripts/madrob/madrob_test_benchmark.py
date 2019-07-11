#!/usr/bin/env python

import rospy
import time
from base_benchmark import BaseBenchmark
from std_msgs.msg import Bool


class BenchmarkObject(BaseBenchmark):

    benchmark_code = 'TEST_BENCHMARK_MADROB'

    # This benchmark checks the state of a laser sensor (supposedly in a door frame): when it turns on and then off,
    # it is assumed the robot passed through the door. The result is updated, and the benchmark finishes
    def execute(self):
        self.door_obstructed = False
        self.passed_through_door = False

        rospy.Subscriber('/eurobench_sensors/door_laser',
                         Bool, self.door_laser_callback)

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
