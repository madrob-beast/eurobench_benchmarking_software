#!/usr/bin/env python

import rospy
import time
import madrob_door_control
from base_benchmark import BaseBenchmark
from std_msgs.msg import Bool


class BenchmarkObject(BaseBenchmark):

    benchmark_code = 'MADROB_TRAVERSAL'

    # In this benchmark, the door is not locked: the robot simply needs to push it open.
    # No torque or braking is applied to the door.
    def execute(self):
        self.door_obstructed = False
        self.passed_through_door = False

        # Setup door based on the currently selected benchmark type
        madrob_door_control.setup_door()

        # TODO: based on a combo box for door mode, setup door with corresponding brake_enabled and LUT.

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
