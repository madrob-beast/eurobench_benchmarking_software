#!/usr/bin/env python

import sys
import rospy
import yaml
import time
import threading
import benchmark_scripts.madrob
from benchmark_scripts.madrob import *
import benchmark_scripts.beast
from benchmark_scripts.beast import *
from os import path
from std_srvs.srv import Empty, EmptyResponse
from eurobench_benchmark_server.srv import *
from eurobench_benchmark_server.msg import *
import madrob_door_control


class BenchmarkServer(object):

    def __init__(self):
        rospy.init_node('eurobench_benchmark_server')

        self.current_benchmark = None
        self.last_benchmark_info = ''

        self.load_benchmark_scripts()

        # Check if output dir exists
        output_dir = path.expanduser(rospy.get_param('benchmark_output_directory'))
        if (not path.exists(output_dir)) or path.isfile(output_dir):
            raise Exception(
                'Output directory "%s" does not exist.' % (output_dir))

        rospy.loginfo('EUROBENCH benchmark server started.')

    def load_benchmark_scripts(self):
        self.scripts = {}

        if rospy.get_param('~benchmark_group') == 'MADROB':
            modules = benchmark_scripts.madrob.__all__
        else:
            modules = benchmark_scripts.beast.__all__

        for module_name in modules:
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

        self.scripts[request.benchmark_code].setup(request.robot_name)
        self.current_benchmark = self.scripts[request.benchmark_code]

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

    def bmserver_settings_callback(self, request):
        settings_response = BenchmarkServerSettingsResponse()

        # Get robot names from params
        robot_names = rospy.get_param('robot_names')
        settings_response.robot_names = robot_names

        # Get list of benchmark descriptions
        # Go through self.scripts, for each one add it to benchmark_ids[], go to its yaml file and add its description to benchmark_descriptions[]
        benchmark_ids = list(self.scripts.keys())
        settings_response.benchmark_ids = benchmark_ids

        config_dir = rospy.get_param('benchmark_config_directory')
        for benchmark_id in benchmark_ids:
            # Load config from yaml file
            config_filepath = path.join(config_dir, '%s.yaml' % (benchmark_id))
            with open(config_filepath) as config_file:
                config = yaml.load(config_file)
                description = config['benchmark_description']
                timeout = config['benchmark_timeout']
                settings_response.benchmark_descriptions.append(description)
                settings_response.benchmark_timeouts.append(timeout)
        
        return settings_response

    def shutdown_callback(self, request):
        if self.current_benchmark:
            self.current_benchmark.save_result()

        response = EmptyResponse()
        return response

    def madrob_settings_callback(self, request):
        madrob_settings_response = MadrobSettingsResponse()

        # Get names of madrob benchmark types
        benchmark_names = madrob_door_control.benchmark_types.keys()
        madrob_settings_response.benchmark_types = benchmark_names
        return madrob_settings_response

    def execute_benchmark(self):
        self.current_benchmark.execute()
        self.current_benchmark.save_result()

        rospy.loginfo('\n---\n BENCHMARK FINISHED: %s | Robot name: %s \n---' %
                      (self.current_benchmark.benchmark_code, self.current_benchmark.robot_name))

        self.last_benchmark_info = self.current_benchmark.get_benchmark_info()

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
            if self.last_benchmark_info:
                server_state.last_benchmark_info = self.last_benchmark_info

        self.benchmark_server_state_publisher.publish(server_state)

    def run(self):
        self.start_benchmark_service = rospy.Service(
            'bmserver/start_benchmark', StartBenchmark, self.start_benchmark_callback)
        self.stop_benchmark_service = rospy.Service(
            'bmserver/stop_benchmark', StopBenchmark, self.stop_benchmark_callback)
        self.benchmark_settings_service = rospy.Service(
            'bmserver/settings', BenchmarkServerSettings, self.bmserver_settings_callback)
        self.shutdown_service = rospy.Service(
            'bmserver/shutdown', Empty, self.shutdown_callback)
        self.madrob_settings_service = rospy.Service(
            'madrob/settings', MadrobSettings, self.madrob_settings_callback)

        self.benchmark_server_state_publisher = rospy.Publisher(
            'bmserver/state', BenchmarkServerState, queue_size=1)

        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():

            # If a benchmark is running, check its timeout
            if self.current_benchmark and hasattr(self.current_benchmark, 'timeout'):
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
