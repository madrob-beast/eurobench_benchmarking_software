import glob
from os import path

import rospy

from eurobench_bms_msgs_and_srvs.srv import *
from madrob_srvs.srv import *
import yaml


class MadrobTestbedComm(object):

    def __init__(self, config):
        self.config = config

        self.current_benchmark_name = None
        self.current_benchmark_type = None

        self.stop_benchmark = rospy.ServiceProxy('bmcore/stop_benchmark', StopBenchmark)

        self.door_opening_side = None
        self.robot_approach_side = None

    def setup_testbed(self):

        # Based on the currently selected benchmark type, set the brake_enabled and LUT values
        get_benchmark_params = rospy.ServiceProxy('madrob/gui/benchmark_params', MadrobBenchmarkParams)
        response = get_benchmark_params()
        self.current_benchmark_name = response.benchmark_type
        self.current_benchmark_type = self.config[self.current_benchmark_name]

        self.door_opening_side = response.door_opening_side
        self.robot_approach_side = response.robot_approach_side

        # Set door controller mode
        set_mode_service_name = '/madrob/door/set_mode'
        try:
            rospy.wait_for_service(set_mode_service_name, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr(set_mode_service_name + ' service unavailable.')
            self.stop_benchmark()

        set_door_mode = rospy.ServiceProxy(set_mode_service_name, SetDoorControllerMode)

        door_mode_request = SetDoorControllerModeRequest()
        if self.current_benchmark_type['brake_enabled']:
            door_mode_request.mode = SetDoorControllerModeRequest.MODE_LUT
        else:
            door_mode_request.mode = SetDoorControllerModeRequest.MODE_DISABLED
        door_mode_response = set_door_mode(door_mode_request)

        if door_mode_response.success:
            rospy.loginfo('Door controller mode successfully set')
        else:
            rospy.logerr('Error setting door controller mode: %s' % door_mode_response.message)
        
        # lutCCW has reverse order
        lut = self.current_benchmark_type['lut']
        lutCCW = list(reversed(self.current_benchmark_type['lut']))

        # Set door controller LUT
        set_door_lut = rospy.ServiceProxy('/madrob/door/set_lut', SetDoorControllerLUT)

        if self.current_benchmark_type['brake_enabled']:
            cw_door_lut_request = SetDoorControllerLUTRequest()
            cw_door_lut_request.type = SetDoorControllerLUTRequest.ANGLE_CW
            cw_door_lut_request.values = lut
            cw_door_lut_response = set_door_lut(cw_door_lut_request)
            if cw_door_lut_response.success:
                rospy.loginfo('CW door LUT successfully set')
            else:
                rospy.logerr('Error setting CW door LUT: %s' % cw_door_lut_response.message)

            ccw_door_lut_request = SetDoorControllerLUTRequest()
            ccw_door_lut_request.type = SetDoorControllerLUTRequest.ANGLE_CCW
            ccw_door_lut_request.values = lutCCW
            ccw_door_lut_response = set_door_lut(ccw_door_lut_request)
            if ccw_door_lut_response.success:
                rospy.loginfo('CCW door LUT successfully set')
            else:
                rospy.logerr('Error setting CCW door LUT: %s' % ccw_door_lut_response.message)

    def get_testbed_conf_file(self, start_time_ros, robot_name, run_number):
        testbed_params = dict()

        testbed_params['start_time'] = start_time_ros.to_sec()
        testbed_params['robot_name'] = robot_name
        testbed_params['run_number'] = run_number
        testbed_params['benchmark_type'] = self.current_benchmark_name
        testbed_params['door_controller_mode'] = SetDoorControllerModeRequest.MODE_LUT if self.current_benchmark_type['brake_enabled'] else SetDoorControllerModeRequest.MODE_DISABLED
        testbed_params['LUTcw'] = self.current_benchmark_type['lut']
        testbed_params['LUTccw'] = list(reversed(self.current_benchmark_type['lut']))
        testbed_params['door_opening_side'] = self.door_opening_side
        testbed_params['robot_approach_side'] = self.robot_approach_side

        return testbed_params
