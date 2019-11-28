#!/usr/bin/env python

import sys
import rospy
import yaml
from datetime import datetime
import threading
from os import path, makedirs
from std_srvs.srv import Empty, EmptyResponse
from eurobench_bms_msgs_and_srvs.srv import *
from eurobench_bms_msgs_and_srvs.msg import *
from benchmark import Benchmark


class BenchmarkServer(object):

    def __init__(self):
        rospy.init_node('eurobench_benchmark_server')

        self.current_benchmark = None
        self.last_benchmark_info = ''

        self.benchmark_group = rospy.get_param('~benchmark_group')

        # Load config from yaml file
        config_dir = rospy.get_param('benchmark_config_directory')
        config_filepath = path.join(config_dir, '%s.yaml' % (self.benchmark_group))

        with open(config_filepath) as config_file:
            self.config = yaml.load(config_file)

        rospy.loginfo('EUROBENCH benchmark server started.')

    def start_benchmark_callback(self, request):
        if self.current_benchmark:
            rospy.loginfo(
                'Cannot start benchmark: a benchmark is currently running.')
            return StartBenchmarkResponse(False)

        rospy.loginfo('\n---\n STARTING BENCHMARK: %s | Robot name: %s | Run %d\n---' %
                      (self.benchmark_group, request.robot_name, request.run_number))

        self.current_benchmark = Benchmark(self.benchmark_group, self.config)
        self.current_benchmark.setup(request.robot_name, request.run_number)

        # Execute benchmark in a new thread
        threading.Thread(target=self.execute_benchmark).start()

        return StartBenchmarkResponse(True)

    def stop_benchmark_callback(self, request):
        if self.current_benchmark:
            rospy.loginfo('Sending terminate signal to benchmark: %s' %
                          self.benchmark_group)
            self.current_benchmark.terminated = True
            # Setting current_benchmark to None not needed, as the current benchmark should finish execution soon.

        return StopBenchmarkResponse(True)

    def bmserver_robot_names_callback(self, request):
        settings_response = BenchmarkServerRobotNamesResponse()

        # Get robot names from params
        robot_names = rospy.get_param('robot_names')
        settings_response.robot_names = robot_names
        
        return settings_response

    def shutdown_callback(self, request):
        if self.current_benchmark:
            self.current_benchmark.save_result()

        response = EmptyResponse()
        return response

    def madrob_settings_callback(self, request):
        madrob_settings_response = MadrobSettingsResponse()

        # Get names of madrob benchmark types, ordered by id
        benchmarks = self.config['benchmarks']

        benchmark_types = sorted(benchmarks.items(), key = lambda kv:kv[1]['id'])
        benchmark_names = [benchmark_type[0] for benchmark_type in benchmark_types]

        madrob_settings_response.benchmark_types = benchmark_names
        return madrob_settings_response

    def execute_benchmark(self):
        self.current_benchmark.execute()
        self.current_benchmark.save_result()

        rospy.loginfo('\n---\n BENCHMARK FINISHED: %s | Robot name: %s \n---' %
                      (self.benchmark_group, self.current_benchmark.robot_name))

        self.last_benchmark_info = self.current_benchmark.get_benchmark_info()

        self.current_benchmark = None

    def publish_server_state(self):
        server_state = BenchmarkServerState()
        server_state.timestamp = rospy.Time.now()

        if self.current_benchmark:
            server_state.status = server_state.RUNNING_BENCHMARK
            server_state.current_benchmark_seconds_passed = (datetime.now() - self.current_benchmark.start_time).seconds
            server_state.current_benchmark_info = self.current_benchmark.get_benchmark_info()

        else:
            server_state.status = server_state.READY
            if self.last_benchmark_info:
                server_state.current_benchmark_seconds_passed = 0
                server_state.last_benchmark_info = self.last_benchmark_info

        self.benchmark_server_state_publisher.publish(server_state)

    def run(self):
        self.start_benchmark_service = rospy.Service(
            'bmserver/start_benchmark', StartBenchmark, self.start_benchmark_callback)

        self.stop_benchmark_service = rospy.Service(
            'bmserver/stop_benchmark', StopBenchmark, self.stop_benchmark_callback)

        self.benchmark_settings_service = rospy.Service(
            'bmserver/robot_names', BenchmarkServerRobotNames, self.bmserver_robot_names_callback)

        self.shutdown_service = rospy.Service(
            'bmserver/shutdown', Empty, self.shutdown_callback)

        if self.benchmark_group == 'MADROB':
            self.madrob_settings_service = rospy.Service(
                'madrob/settings', MadrobSettings, self.madrob_settings_callback)

        self.benchmark_server_state_publisher = rospy.Publisher(
            'bmserver/state', BenchmarkServerState, queue_size=1)

        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
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
