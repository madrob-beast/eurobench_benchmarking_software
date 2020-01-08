#!/usr/bin/env python

from exceptions import NotImplementedError

class BasePerformance(object):

    def __init__(self, output_dir):
        self.output_dir = output_dir

    def run(self, preprocessed_filenames_dict, testbed_configuration_filepath, start_time):
        raise NotImplementedError()