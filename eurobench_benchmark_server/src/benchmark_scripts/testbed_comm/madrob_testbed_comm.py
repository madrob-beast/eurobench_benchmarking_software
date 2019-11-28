import rospy

from eurobench_bms_msgs_and_srvs.srv import *
from madrob_srvs.srv import *
from collections import OrderedDict
from benchmark_scripts.testbed_comm.base_testbed_comm import BaseTestbedComm
import yaml

class MadrobTestbedComm(BaseTestbedComm):

    def __init__(self, config):
        self.config = config

        self.current_benchmark_name = None
        self.current_benchmark_type = None

    def setup_testbed(self):
        # Based on the currently selected benchmark type, set the brake_enabled and LUT values
        get_benchmark_type_name = rospy.ServiceProxy('madrob/gui/benchmark_type', MadrobBenchmarkType)
        response = get_benchmark_type_name()
        self.current_benchmark_name = response.benchmark_type
        self.current_benchmark_type = self.config[self.current_benchmark_name]

        # Set door controller mode
        door_node_name = rospy.get_param('door_node_name')

        set_door_mode = rospy.ServiceProxy('/' + door_node_name + '/set_mode', SetDoorControllerMode)

        door_mode_request = SetDoorControllerModeRequest()
        if self.current_benchmark_type['brake_enabled']:
            door_mode_request.mode = SetDoorControllerModeRequest.MODE_LUT
        else:
            door_mode_request.mode = SetDoorControllerModeRequest.MODE_DISABLED
        door_mode_response = set_door_mode(door_mode_request)

        if door_mode_response.success:
            rospy.loginfo('Door controller mode successfully set')
        else:
            rospy.logerr('Error setting door controller mode: %s' % (door_mode_response.message))
        
        # lutCCW has reverse order
        lut = self.current_benchmark_type['lut']
        lutCCW = list(reversed(self.current_benchmark_type['lut']))

        # Set door controller LUT
        set_door_lut = rospy.ServiceProxy('/' + door_node_name + '/set_lut', SetDoorControllerLUT)

        if self.current_benchmark_type['brake_enabled']:
            cw_door_lut_request = SetDoorControllerLUTRequest()
            cw_door_lut_request.type = SetDoorControllerLUTRequest.ANGLE_CW
            cw_door_lut_request.values = lut
            cw_door_lut_response = set_door_lut(cw_door_lut_request)
            if cw_door_lut_response.success:
                rospy.loginfo('CW door LUT successfully set')
            else:
                rospy.logerr('Error setting CW door LUT: %s' % (cw_door_lut_response.message))

            ccw_door_lut_request = SetDoorControllerLUTRequest()
            ccw_door_lut_request.type = SetDoorControllerLUTRequest.ANGLE_CCW
            ccw_door_lut_request.values = lutCCW
            ccw_door_lut_response = set_door_lut(ccw_door_lut_request)
            if ccw_door_lut_response.success:
                rospy.loginfo('CCW door LUT successfully set')
            else:
                rospy.logerr('Error setting CCW door LUT: %s' % (ccw_door_lut_response.message))

    def write_testbed_conf_file(self, filepath):
        door_params = {}

        door_params['Benchmark type'] = self.current_benchmark_name
        door_params['Door controller mode'] = SetDoorControllerModeRequest.MODE_LUT if self.current_benchmark_type['brake_enabled'] else SetDoorControllerModeRequest.MODE_DISABLED
        door_params['LUTcw'] = self.current_benchmark_type['lut']
        door_params['LUTccw'] = list(reversed(self.current_benchmark_type['lut']))

        with open(filepath, 'w') as file:
            yaml.dump(door_params, file, default_flow_style=False)