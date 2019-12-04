#!/usr/bin/env python

import rospy
import rosnode
from std_srvs.srv import Trigger, TriggerResponse
from eurobench_bms_msgs_and_srvs.srv import StartRecording, StartRecordingResponse

import os
import subprocess


class DataRecorder():
    def __init__(self):
        self.start_recording_service = rospy.Service('/eurobench_rosbag_recorder/start_recording', StartRecording, self.start_recording)
        self.stop_recording_service = rospy.Service('/eurobench_rosbag_recorder/stop_recording', Trigger, self.stop_recording)

        self.process = None
        self.recording = False

        rospy.loginfo('Data Recorder Started')

    def start_recording(self, req):
        if self.recording:
            rospy.logerr('Already Recording')
            return StartRecordingResponse(False)

        command = ['rosrun', 'rosbag', 'record', '-a', '-x'] + ['|'.join(req.excluded_topics)] + ['-O'] + [req.rosbag_filepath] + ['__name:=eurobench_rosbag_recorder_node']

        self.process = subprocess.Popen(command)
        self.recording = True
        rospy.loginfo('Started recorder, PID %s' % self.process.pid)
        return StartRecordingResponse(True)

    def stop_recording(self, req):
        if not self.recording:
            rospy.logerr('Not Recording')
            return TriggerResponse(False, 'Not Recording')

        rosnode.kill_nodes(['/eurobench_rosbag_recorder_node'])

        self.process = None
        self.recording = False

        rospy.loginfo('Stopped Recording')
        return TriggerResponse(True, 'Stopped Recording')


if __name__ == "__main__":
    rospy.init_node('eurobench_rosbag_recorder')
    DataRecorder()
    rospy.spin()
