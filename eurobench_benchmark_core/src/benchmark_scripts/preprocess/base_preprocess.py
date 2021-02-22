#!/usr/bin/env python
# -*- coding: utf-8 -*-

from exceptions import NotImplementedError
from os import path


class BasePreprocess(object):
    def __init__(self, data_format_name):
        self.data_format_name = data_format_name
        self.preprocess_dir = None
        self.robot_name = None
        self.condition_number = None
        self.run_number = None

    def start(self, benchmark_group, robot_name, condition_number, run_number, start_time, testbed_conf, preprocess_dir, live_benchmark):
        raise NotImplementedError()

    def finish(self):
        raise NotImplementedError()

    def preprocessed_csv_file_path(self):
        file_name = 'subject_{n:03d}_cond_{c:03d}_run_{r:03d}_{name}.csv'.format(
            n=int(self.robot_name),
            c=int(self.condition_number),
            r=int(self.run_number),
            name=self.data_format_name)
        file_path = path.join(self.preprocess_dir, file_name)

        return file_path
