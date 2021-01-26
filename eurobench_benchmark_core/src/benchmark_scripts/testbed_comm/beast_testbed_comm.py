import rospy
from std_msgs.msg import Float32

from eurobench_bms_msgs_and_srvs.srv import *
from beast_srvs.srv import *
from collections import OrderedDict
import yaml


class BeastTestbedComm(object):

    def __init__(self, config):
        self.config = config

        self.current_benchmark_name = None
        self.current_benchmark_type = None

        self.stop_benchmark = rospy.ServiceProxy('bmcore/stop_benchmark', StopBenchmark)

        self.set_left_wheel_braking_mode = rospy.ServiceProxy("/left/set_wheel_braking_mode", SetWheelBrakingMode)
        self.set_right_wheel_braking_mode = rospy.ServiceProxy("/right/set_wheel_braking_mode", SetWheelBrakingMode)

        self.set_left_wheel_braking = rospy.ServiceProxy("/left/set_wheel_braking", SetWheelBraking)
        self.set_right_wheel_braking = rospy.ServiceProxy("/right/set_wheel_braking", SetWheelBraking)

        self.set_left_wheel_braking_lut = rospy.ServiceProxy("/left/set_wheel_braking_lut", SetWheelBrakingLUT)
        self.set_right_wheel_braking_lut = rospy.ServiceProxy("/right/set_wheel_braking_lut", SetWheelBrakingLUT)

        self.left_braking_pub = rospy.Publisher("/left/braking", Float32, queue_size=1)
        self.right_braking_pub = rospy.Publisher("/right/braking", Float32, queue_size=1)

        self.disturbance_type = None
        self.load = None
        self.start_already_gripping = None
        self.braking_mode = None

    def setup_testbed(self):
        get_benchmark_params = rospy.ServiceProxy('beast/gui/benchmark_params', BeastBenchmarkParams)
        response = get_benchmark_params()
        self.disturbance_type = response.disturbance_type
        self.load = response.load
        self.start_already_gripping = response.start_already_gripping

        # Set the braking mode to MODE_DISABLED with services
        wheel_braking_mode_disabled = SetWheelBrakingModeRequest(
            mode=SetWheelBrakingModeRequest.MODE_DISABLED,
            lut_index=0)
        self.set_left_wheel_braking_mode(wheel_braking_mode_disabled)
        self.set_right_wheel_braking_mode(wheel_braking_mode_disabled)

        if self.config[self.disturbance_type]['mode'] == "lut":
            rospy.loginfo("BeastTestbedComm.setup_testbed: setting braking mode to LUT")
            lut_values = [0.0] * 360
            config_lut = self.config[self.disturbance_type]['lut']
            lut_values[0:len(config_lut)] = config_lut

            # Set lut with services
            wheel_braking_lut = SetWheelBrakingLUTRequest(
                index=0,
                method=SetWheelBrakingLUTRequest.METHOD_BRAKE,
                mu=SetWheelBrakingLUTRequest.MU_PERCENT,
                values=lut_values)
            self.set_left_wheel_braking_lut(wheel_braking_lut)
            self.set_right_wheel_braking_lut(wheel_braking_lut)

            # start producing braking setpoints
            self.left_braking_pub.publish(Float32(0.0))
            self.right_braking_pub.publish(Float32(0.0))

            # change to lut mode
            wheel_braking_mode = SetWheelBrakingModeRequest(
                mode=SetWheelBrakingModeRequest.MODE_LUT,
                lut_index=0)
            self.set_left_wheel_braking_mode(wheel_braking_mode)
            self.set_right_wheel_braking_mode(wheel_braking_mode)
        else:
            rospy.loginfo("BeastTestbedComm.setup_testbed: not setting any braking mode (disabled)")

    def keep_running(self):

        if self.config[self.disturbance_type]['mode'] == "lut":
            # keep publishing setpoints, otherwise the controller will timeout
            self.left_braking_pub.publish(Float32(0.0))
            self.right_braking_pub.publish(Float32(0.0))

        # check that the braking mode is correct TODO
        #   /left/wheel_status
        #   /right/wheel_status

    def stop(self):
        # Set the braking mode to MODE_DISABLED with services
        wheel_braking_mode_disabled = SetWheelBrakingModeRequest(
            mode=SetWheelBrakingModeRequest.MODE_DISABLED,
            lut_index=0)
        self.set_left_wheel_braking_mode(wheel_braking_mode_disabled)
        self.set_right_wheel_braking_mode(wheel_braking_mode_disabled)

    def get_testbed_conf_file(self, start_time_ros, robot_name, run_number):
        testbed_params = dict()
        testbed_params['start_time'] = start_time_ros.to_sec()
        testbed_params['robot_name'] = robot_name
        testbed_params['run_number'] = run_number
        testbed_params['disturbance_type'] = self.disturbance_type
        testbed_params['load'] = self.load
        testbed_params['start_already_gripping'] = self.start_already_gripping
        return testbed_params
