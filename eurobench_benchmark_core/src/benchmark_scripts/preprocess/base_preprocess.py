#!/usr/bin/env python
# -*- coding: utf-8 -*-

from exceptions import NotImplementedError
from os import path


class BasePreprocess(object):
    def __init__(self, data_format_name):
        self.data_format_name = data_format_name
        self.preprocess_dir = None
        self.robot_name = None
        self.run_number = None

    def start(self, benchmark_group, robot_name, run_number, start_time, testbed_conf, preprocess_dir):
        raise NotImplementedError()

    def finish(self):
        raise NotImplementedError()

    def preprocessed_csv_file_path(self):
        file_name = 'subject_{n:05d}_run_{r:05d}_{name}.csv'.format(
            n=int(self.robot_name),
            r=int(self.run_number),
            name=self.data_format_name)
        file_path = path.join(self.preprocess_dir, file_name)

        return file_path
