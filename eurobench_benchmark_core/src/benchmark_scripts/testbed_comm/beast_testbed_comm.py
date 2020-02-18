import rospy

from eurobench_bms_msgs_and_srvs.srv import *
from beast_srvs.srv import *
from collections import OrderedDict
from benchmark_scripts.testbed_comm.base_testbed_comm import BaseTestbedComm
import yaml

class BeastTestbedComm(BaseTestbedComm):

    def __init__(self):
        self.current_benchmark_name = None
        self.current_benchmark_type = None

        self.stop_benchmark = rospy.ServiceProxy('bmcore/stop_benchmark', StopBenchmark)

    def setup_testbed(self):
        # Get the 'stiffness' from the GUI
        get_benchmark_params = rospy.ServiceProxy('beast/gui/benchmark_params', BeastBenchmarkParams)
        response = get_benchmark_params()
        self.trolley_stiffness = response.stiffness

        # Set stiffness in the trolley
        trolley_node_name = rospy.get_param('testbed_nodes')['trolley']
        set_stiffness_service_name = '/' + trolley_node_name + '/set_stiffness'
        set_stiffness = rospy.ServiceProxy(set_stiffness_service_name, SetStiffness)

        set_stiffness_response = set_stiffness(self.trolley_stiffness)

        if set_stiffness_response.success:
            rospy.loginfo('Trolley stiffness successfully set')
        else:
            rospy.logerr('Error setting trolley stiffness: %s' % (set_stiffness_response.message))

    def write_testbed_conf_file(self, filepath, start_time_ros, robot_name, run_number, rosbag_filepath):
        testbed_params = {}

        testbed_params['Start time'] = '%d.%d' % (start_time_ros.secs, start_time_ros.nsecs)
        testbed_params['Robot name'] = robot_name
        testbed_params['Run number'] = run_number
        testbed_params['Rosbag path'] = rosbag_filepath
        testbed_params['Trolley stiffness'] = self.trolley_stiffness

        with open(filepath, 'w') as file:
            yaml.dump(testbed_params, file, default_flow_style=False)

        return testbed_params