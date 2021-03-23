#!/usr/bin/env python
import glob

import rospy
import yaml
from datetime import datetime
import threading
from os import path
from std_srvs.srv import Empty, EmptyResponse
from eurobench_bms_msgs_and_srvs.srv import *
from eurobench_bms_msgs_and_srvs.msg import *
from benchmark import Benchmark


class BenchmarkCore(object):

    def __init__(self):
        rospy.init_node('eurobench_benchmark_core')

        self.current_benchmark = None
        self.last_benchmark_info = ''

        self.benchmark_group = rospy.get_param('~benchmark_group')
        self.benchmark_countdown = int(rospy.get_param('benchmark_countdown'))

        # Load config from yaml file
        config_dir = rospy.get_param('benchmark_config_directory')
        config_filepath = path.join(config_dir, '%s.yaml' % self.benchmark_group)

        with open(config_filepath) as config_file:
            self.config = yaml.load(config_file, Loader=yaml.FullLoader)

        self.benchmark_core_state_publisher = None

        rospy.loginfo('EUROBENCH benchmark core started.')

    def start_benchmark_callback(self, request):
        if self.current_benchmark:
            rospy.loginfo(
                'Cannot start benchmark: a benchmark is currently running.')
            return StartBenchmarkResponse(False)

        self.current_benchmark = Benchmark(self.benchmark_group, self.config, request.robot_name, request.run_number)

        rospy.loginfo('\n---\n STARTING BENCHMARK: %s | Robot name: %s | Run %d\n---' % (self.benchmark_group, self.current_benchmark.robot_name, self.current_benchmark.run_number))

        # Execute benchmark in a new thread
        threading.Thread(target=self.execute_benchmark).start()

        return StartBenchmarkResponse(True)

    def stop_benchmark_callback(self, _):
        if self.current_benchmark:
            rospy.loginfo('Sending terminate signal to benchmark: %s' % self.benchmark_group)
            self.current_benchmark.terminated = True
            # Setting current_benchmark to None not needed, as the current benchmark should finish execution soon.

        return StopBenchmarkResponse(True)

    def shutdown_callback(self, _):
        if self.current_benchmark:
            self.current_benchmark.terminated = True

        # Shutdown in one second
        rospy.Timer(rospy.Duration(1), self.shutdown, oneshot=True)        

        response = EmptyResponse()
        return response

    @staticmethod
    def shutdown(_):
        rospy.signal_shutdown('Shutting down')

    def madrob_settings_callback(self, _):
        madrob_settings_response = MadrobSettingsResponse()

        # Get names of madrob benchmark types, ordered by id
        benchmarks = self.config['benchmarks']

        benchmark_types = sorted(benchmarks.items(), key=lambda kv: kv[1]['id'])
        benchmark_names = [benchmark_type[0] for benchmark_type in benchmark_types]

        madrob_settings_response.benchmark_types = benchmark_names
        return madrob_settings_response

    def execute_benchmark(self):
        self.current_benchmark.execute()

        rospy.loginfo('\n---\n BENCHMARK FINISHED: %s | Robot name: %s \n---' %
                      (self.benchmark_group, self.current_benchmark.robot_name))

        self.current_benchmark = None

    def publish_core_state(self):
        core_state = BenchmarkCoreState()
        core_state.timestamp = rospy.Time.now()

        if self.current_benchmark:
            core_state.status = core_state.RUNNING_BENCHMARK
            core_state.current_benchmark_seconds_passed = (datetime.now() - self.current_benchmark.start_time).total_seconds()
            if core_state.current_benchmark_seconds_passed < 0:
                core_state.current_benchmark_seconds_passed -= 1  # so it reaches 0 a the right moment
        else:
            core_state.status = core_state.READY

        self.benchmark_core_state_publisher.publish(core_state)

    def run(self):
        rospy.Service('bmcore/start_benchmark', StartBenchmark, self.start_benchmark_callback)
        rospy.Service('bmcore/stop_benchmark', StopBenchmark, self.stop_benchmark_callback)
        rospy.Service('bmcore/shutdown', Empty, self.shutdown_callback)

        if self.benchmark_group == 'MADROB':
            rospy.Service('madrob/settings', MadrobSettings, self.madrob_settings_callback)

        self.benchmark_core_state_publisher = rospy.Publisher('bmcore/state', BenchmarkCoreState, queue_size=1)

        rate = rospy.Rate(50)  # 10Hz
        while not rospy.is_shutdown():
            self.publish_core_state()
            rate.sleep()


if __name__ == '__main__':
    try:
        core = BenchmarkCore()
        core.run()
    except rospy.ROSInterruptException:
        pass
