import rospy

from eurobench_bms_msgs_and_srvs.srv import *
from beast_srvs.srv import *
from collections import OrderedDict
from benchmark_scripts.testbed_comm.base_testbed_comm import BaseTestbedComm
import yaml


class BeastTestbedComm(object):

    def __init__(self):
        self.current_benchmark_name = None
        self.current_benchmark_type = None

        self.stop_benchmark = rospy.ServiceProxy('bmcore/stop_benchmark', StopBenchmark)

        self.disturbance_type = None
        self.load = None
        self.start_already_gripping = None
        self.braking_mode = None

    def setup_testbed(self):
        # Get the 'stiffness' from the GUI
        get_benchmark_params = rospy.ServiceProxy('beast/gui/benchmark_params', BeastBenchmarkParams)
        response = get_benchmark_params()
        self.disturbance_type = response.disturbance_type
        self.load = response.load
        self.start_already_gripping = response.start_already_gripping

        # Set stiffness in the trolley
        # trolley_node_name = rospy.get_param('testbed_nodes')['trolley']
        # set_stiffness_service_name = '/' + trolley_node_name + '/set_stiffness'
        # set_stiffness = rospy.ServiceProxy(set_stiffness_service_name, SetStiffness)
        #
        # set_stiffness_response = set_stiffness(self.trolley_stiffness)
        #
        # if set_stiffness_response.success:
        #     rospy.loginfo('Trolley stiffness successfully set')
        # else:
        #     rospy.logerr('Error setting trolley stiffness: %s' % set_stiffness_response.message)
        # TODO send self.disturbance_type to testbed hardware using service

        # Set the braking mode to MODE_DISABLED with services
        # set_wheel_braking_mode(mode=MODE_DISABLED, lut_index=0)
        #   /left/set_wheel_braking_mode
        #   /right/set_wheel_braking_mode

        # Set lut with services
        # set_wheel_braking_lut(index=0, method=METHOD_BRAKE, mu=MU_PERCENT, values=self.lut_values)
        #   /right/set_wheel_braking_lut
        #   /left/set_wheel_braking_lut

        # start producing braking setpoints

        # change to lut mode
        # set_wheel_braking_mode(mode=MODE_LUT, lut_index=0)
        #   /left/set_wheel_braking_mode
        #   /right/set_wheel_braking_mode

    def keep_running(self):
        pass
        # keep publishing setpoints, otherwise the controller will timeout
        #   /left/braking
        #   /right/braking

        # check that the braking mode is correct
        #   /left/wheel_status
        #   /right/wheel_status

    def stop(self):
        pass
        # Set the braking mode to MODE_DISABLED with services
        # set_wheel_braking_mode(mode=MODE_DISABLED, lut_index=0)
        #   /left/set_wheel_braking_mode
        #   /right/set_wheel_braking_mode

    def get_testbed_conf_file(self, start_time_ros, robot_name, run_number):
        testbed_params = dict()
        testbed_params['start_time'] = start_time_ros.to_sec()
        testbed_params['robot_name'] = robot_name
        testbed_params['run_number'] = run_number
        testbed_params['disturbance_type'] = self.disturbance_type
        testbed_params['load'] = self.load
        testbed_params['start_already_gripping'] = self.start_already_gripping
        return testbed_params
