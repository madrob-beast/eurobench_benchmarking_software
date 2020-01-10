#!/usr/bin/env python

from exceptions import NotImplementedError

class BasePreprocess(object):

    def start(self, rosbag_path=None):
        raise NotImplementedError()

    def finish(self):
        raise NotImplementedError()