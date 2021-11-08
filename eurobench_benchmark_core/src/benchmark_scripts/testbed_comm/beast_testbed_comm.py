import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Quaternion
from std_msgs.msg import Float32, Header

from beast_msgs.msg import Wheel
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

        self.localization_pose_publisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)

        self.set_left_wheel_braking_mode = rospy.ServiceProxy("left/set_wheel_braking_mode", SetWheelBrakingMode)
        self.set_right_wheel_braking_mode = rospy.ServiceProxy("right/set_wheel_braking_mode", SetWheelBrakingMode)

        self.set_left_wheel_braking = rospy.ServiceProxy("left/set_wheel_braking", SetWheelBraking)
        self.set_right_wheel_braking = rospy.ServiceProxy("right/set_wheel_braking", SetWheelBraking)

        self.set_left_wheel_braking_lut = rospy.ServiceProxy("left/set_wheel_braking_lut", SetWheelBrakingLUT)
        self.set_right_wheel_braking_lut = rospy.ServiceProxy("right/set_wheel_braking_lut", SetWheelBrakingLUT)

        self.left_braking_pub = rospy.Publisher("left/setpoint", Float32, queue_size=1)
        self.right_braking_pub = rospy.Publisher("right/setpoint", Float32, queue_size=1)

        self.left_wheel_sub = rospy.Subscriber("left/wheel_status", Wheel, self.left_wheel_callback)
        self.right_wheel_sub = rospy.Subscriber("right/wheel_status", Wheel, self.right_wheel_callback)

        self.disturbance_type = None
        self.load = None
        self.start_already_gripping = None
        self.braking_mode = None
        self.setpoint_timer = None
        self.last_left_wheel_status_msg = None
        self.last_right_wheel_status_msg = None
        self.angular_distance = 0.0
        self.started_sudden_braking = None
        self.sudden_braking_distance = 15.0
        self.sudden_braking_duration = rospy.Duration.from_sec(0.1)

    def setup_testbed(self):
        get_benchmark_params = rospy.ServiceProxy('beast/gui/benchmark_params', BeastBenchmarkParams)
        response = get_benchmark_params()
        self.disturbance_type = response.disturbance_type
        self.load = response.load
        self.start_already_gripping = response.start_already_gripping

    def start(self):

        # set the localization node's pose at the starting pose (origin of the map)
        cov_mat = np.zeros((6, 6))
        cov_mat[0, 0] = 0.1
        cov_mat[1, 1] = 0.1
        cov_mat[5, 5] = 0.068
        initial_pose_msg = PoseWithCovarianceStamped(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id='map'
            ),
            pose=PoseWithCovariance(
                pose=Pose(
                    orientation=Quaternion(w=1.0)
                ),
                covariance=cov_mat.flatten()
            )
        )
        self.localization_pose_publisher.publish(initial_pose_msg)

        # Set the braking mode to MODE_DISABLED with services
        wheel_braking_mode_disabled = SetWheelBrakingModeRequest(mode=SetWheelBrakingModeRequest.MODE_DISABLED)
        self.set_left_wheel_braking_mode(wheel_braking_mode_disabled)
        self.set_right_wheel_braking_mode(wheel_braking_mode_disabled)

        if self.config[self.disturbance_type]['mode'] == "sudden_force":
            rospy.loginfo("BeastTestbedComm.setup_testbed: braking mode is sudden_force")
            # lut_values = [0.0] * 360
            # config_lut = self.config[self.disturbance_type]['lut']
            # lut_values[0:len(config_lut)] = config_lut

            # Set lut with services
            # wheel_braking_lut = SetWheelBrakingLUTRequest(
            #     index=0,
            #     method=SetWheelBrakingLUTRequest.METHOD_SPEED,
            #     mu=SetWheelBrakingLUTRequest.MU_PERCENT,
            #     values=lut_values)
            # self.set_left_wheel_braking_lut(wheel_braking_lut)
            # self.set_right_wheel_braking_lut(wheel_braking_lut)

            # Set braking with services
            wheel_braking = SetWheelBrakingRequest(
                method=SetWheelBrakingRequest.METHOD_SPEED,
                mu=SetWheelBrakingRequest.MU_PERCENT,
                setpoint=0.0
            )
            self.set_left_wheel_braking(wheel_braking)
            self.set_right_wheel_braking(wheel_braking)

            wheel_braking_mode = SetWheelBrakingModeRequest(mode=SetWheelBrakingModeRequest.MODE_DYNAMIC)
            self.set_left_wheel_braking_mode(wheel_braking_mode)
            self.set_right_wheel_braking_mode(wheel_braking_mode)

            self.setpoint_timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.setpoint_timer_callback)

        else:
            rospy.loginfo("BeastTestbedComm.setup_testbed: not setting any braking mode (disabled)")

    def left_wheel_callback(self, msg):
        if self.last_left_wheel_status_msg is not None:
            self.angular_distance += 0.5 * np.abs(msg.velocity) * (msg.header.stamp - self.last_left_wheel_status_msg.header.stamp).to_sec()  # 0.5 because there are two wheels
        self.last_left_wheel_status_msg = msg

    def right_wheel_callback(self, msg):
        if self.last_right_wheel_status_msg is not None:
            self.angular_distance += 0.5 * np.abs(msg.velocity) * (msg.header.stamp - self.last_right_wheel_status_msg.header.stamp).to_sec()  # 0.5 because there are two wheels
        self.last_right_wheel_status_msg = msg

    def setpoint_timer_callback(self, _):
        # keep publishing setpoints, otherwise the controller will timeout
        if self.config[self.disturbance_type]['mode'] == "sudden_force":

            if self.angular_distance > self.sudden_braking_distance:
                self.left_braking_pub.publish(Float32(1.0))
                self.right_braking_pub.publish(Float32(1.0))

                if self.started_sudden_braking is None:
                    self.started_sudden_braking = rospy.Time.now()
                    rospy.loginfo("start sudden braking")

                if rospy.Time.now() - self.started_sudden_braking > self.sudden_braking_duration:
                    self.angular_distance = 0.0
                    self.started_sudden_braking = None
                    rospy.loginfo("finish sudden braking")

            else:
                # self.left_braking_pub.publish(Float32(0.0))
                # self.right_braking_pub.publish(Float32(0.0))
                pass  # when publishing 0 there is braking for some reason, but never mind, just let it timeout: no more problems!

        # check that the braking mode is correct TODO
        #   /left/wheel_status
        #   /right/wheel_status

    def stop(self):
        # stop publishing setpoints
        if self.setpoint_timer is not None:
            self.setpoint_timer.shutdown()

        # Set the braking mode to MODE_DISABLED with services
        wheel_braking_mode_disabled = SetWheelBrakingModeRequest(mode=SetWheelBrakingModeRequest.MODE_DISABLED)
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
