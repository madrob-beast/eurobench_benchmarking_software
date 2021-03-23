#!/usr/bin/env python
import traceback

import rospy
import benchmark_scripts.preprocess.madrob
from benchmark_scripts.preprocess.madrob import *
import benchmark_scripts.preprocess.beast
from benchmark_scripts.preprocess.beast import *


class Preprocess(object):
    def __init__(self, benchmark_group):
        self.benchmark_group = benchmark_group

        if benchmark_group == 'MADROB':
            self.preprocess_scripts = benchmark_scripts.preprocess.madrob.__all__
        
        if benchmark_group == 'BEAST':
            self.preprocess_scripts = benchmark_scripts.preprocess.beast.__all__

        self.saving_data = False

        self.running_preprocess_scripts = []
        self.preprocessed_files = {}

    def start(self, robot_name, condition_number, run_number, start_time, testbed_conf, preprocess_dir):

        for preprocess_module in self.preprocess_scripts:
            # noinspection PyBroadException
            try:
                preprocess_module_obj = globals()[preprocess_module].PreprocessObject()
                self.running_preprocess_scripts.append(preprocess_module_obj)
                rospy.loginfo("preprocess script initialised: {name}".format(name=preprocess_module))

                # noinspection PyBroadException
                try:
                    preprocess_module_obj.start(self.benchmark_group, robot_name, condition_number, run_number, start_time, testbed_conf, preprocess_dir)

                    rospy.loginfo("preprocess script started: {name}".format(name=preprocess_module))
                except Exception:
                    rospy.logerr("failed to start preprocess script {name} due to exception: {exception}".format(name=preprocess_module, exception=traceback.format_exc()))

            except Exception:
                rospy.logerr("failed to initialise preprocess script {name} due to exception: {exception}".format(name=preprocess_module, exception=traceback.format_exc()))

        self.saving_data = True
        return True
    
    def finish(self):
        for preprocess in self.running_preprocess_scripts:
            # noinspection PyBroadException
            try:
                file_type, file_name = preprocess.finish()
                self.preprocessed_files[file_type] = file_name
                rospy.loginfo("preprocess script finished: {name}".format(name=preprocess.data_format_name))
            except Exception:
                rospy.logerr("error finishing preprocess script {name} due to exception: {exception}".format(name=preprocess.data_format_name, exception=traceback.format_exc()))
                continue

        self.saving_data = False

        self.running_preprocess_scripts = []

        return self.preprocessed_files
