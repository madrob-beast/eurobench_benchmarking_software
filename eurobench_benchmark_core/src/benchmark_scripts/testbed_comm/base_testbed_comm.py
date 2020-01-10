#!/usr/bin/env python

from exceptions import NotImplementedError

class BaseTestbedComm(object):

    # Initializes the publishers/subscribers required to communicate with the testbed
    def __init__(self, config):
        raise NotImplementedError()

    # Sets the testbed parameters for a run
    def setup_testbed(self):
        raise NotImplementedError()

    # Writes the testbed parameters to the filepath
    def write_testbed_conf_file(self, filepath, start_time_ros):
        raise NotImplementedError()