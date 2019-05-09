#!/usr/bin/env python

import rospy
import time
import threading
import benchmark_scripts
from benchmark_scripts import *
from os import path
from eurobench_benchmark_server.srv import *
from eurobench_benchmark_server.msg import *


class BenchmarkServer(object):

    def __init__(self):
        rospy.init_node('eurobench_benchmark_server')

        self.current_benchmark = None

        self.load_benchmark_scripts()

        # Check if output dir exists
        output_dir = path.expanduser(rospy.get_param('benchmark_output_directory'))
        if (not path.exists(output_dir)) or path.isfile(output_dir):
            raise Exception(
                'Output directory "%s" does not exist.' % (output_dir))

        rospy.loginfo('EUROBENCH benchmark server started.')

    def load_benchmark_scripts(self):
        self.scripts = {}

        for module_name in benchmark_scripts.__all__:
            module = globals()[module_name]
            benchmark_object = module.BenchmarkObject()
            benchmark_code = benchmark_object.benchmark_code

            self.scripts[benchmark_code] = benchmark_object

    def start_benchmark_callback(self, request):
        if self.current_benchmark:
            rospy.loginfo(
                'Cannot start benchmark: a benchmark is currently running.')
            return StartBenchmarkResponse(False)

        rospy.loginfo('\n---\n STARTING BENCHMARK: %s | Robot name: %s \n---' %
                      (request.benchmark_code, request.robot_name))

        self.current_benchmark = self.scripts[request.benchmark_code]
        self.current_benchmark.setup(request.robot_name)

        # Execute benchmark in a new thread
        threading.Thread(target=self.execute_benchmark).start()

        return StartBenchmarkResponse(True)

    def stop_benchmark_callback(self, request):
        if self.current_benchmark:
            rospy.loginfo('Sending terminate signal to benchmark: %s' %
                          self.current_benchmark.benchmark_code)
            self.current_benchmark.terminated = True
            # Setting current_benchmark to None not needed, as the current benchmark should finish execution soon.

        return StopBenchmarkResponse(True)

    def execute_benchmark(self):
        self.current_benchmark.execute()
        self.current_benchmark.save_result()

        rospy.loginfo('\n---\n BENCHMARK FINISHED: %s | Robot name: %s \n---' %
                      (self.current_benchmark.benchmark_code, self.current_benchmark.robot_name))

        self.current_benchmark = None

    def publish_server_state(self):
        server_state = BenchmarkServerState()
        server_state.timestamp = rospy.Time.now()

        if self.current_benchmark:
            server_state.status = server_state.RUNNING_BENCHMARK
            server_state.current_benchmark_info = self.current_benchmark.get_benchmark_info()
            if self.current_benchmark.timeout and not self.current_benchmark.has_timed_out:
                server_state.current_benchmark_seconds_left = self.current_benchmark.timeout - time.time()
        else:
            server_state.status = server_state.READY

        self.benchmark_server_state_publisher.publish(server_state)

    def run(self):
        self.start_benchmark_service = rospy.Service(
            'bmserver/start_benchmark', StartBenchmark, self.start_benchmark_callback)
        self.stop_benchmark_service = rospy.Service(
            'bmserver/stop_benchmark', StopBenchmark, self.stop_benchmark_callback)
        self.benchmark_server_state_publisher = rospy.Publisher(
            'bmserver/state', BenchmarkServerState, queue_size=10, latch=True)

        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():

            # If a benchmark is running, check its timeout
            if self.current_benchmark and self.current_benchmark.timeout:
                if time.time() > self.current_benchmark.timeout:
                    self.current_benchmark.has_timed_out = True

            self.publish_server_state()

            rate.sleep()

        # rospy has shut down: if there's an active benchmark, save its current result
        if self.current_benchmark:
            self.current_benchmark.save_result()


if __name__ == '__main__':
    try:
        server = BenchmarkServer()
        server.run()
    except rospy.ROSInterruptException:
        pass
