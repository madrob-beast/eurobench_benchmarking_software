#!/usr/bin/env python

from exceptions import NotImplementedError

class BasePreprocess(object):
    def __init__(self, data_type):
        raise NotImplementedError()

    def start(self, benchmark_group, robot_name, run_number, start_time, testbed_conf):
        raise NotImplementedError()

    def finish(self):
        raise NotImplementedError()