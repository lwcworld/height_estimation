#!/usr/bin/env python
import numpy as np
import rospy

from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped
from mavros_msgs.srv import ParamSet, ParamGet
from mavros_msgs.msg import ParamValue

from tf.transformations import euler_from_quaternion

from height_estimation.srv import *

from Utils import Utils

class Rangefinder_aided_VIO(Utils):
    def __init__(self):
        # set util functions
        Utils.__init__(self)

        # ======== ROS communicators declaration ========
        # Subscriber
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.save_EKF2_pose)
        rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, self.save_EKF2_vel)
        rospy.Subscriber("/mavros/distance_sensor/lidarlite_pub", Range, self.save_RF_and_decision) # type_m = 1
        rospy.Subscriber("/mavros/px4flow/ground_distance", Range, self.save_RF_and_decision)
        rospy.Subscriber("/vio_pose_in", PoseWithCovarianceStamped, self.sub_and_pub_VIO)

        # publishers
        self.pub_vision_pose = rospy.Publisher('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped, queue_size=2)

        # ServiceProxy
        self.set_param_srv = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self.get_param_srv = rospy.ServiceProxy('/mavros/param/get', ParamGet)

        # Service
        self.srv1 = rospy.Service('hgt_meas_source_selection_policy', hgt_meas_source_selection_policy, self.cb_hgt_meas_source_selection_policy)

        # ===== PX4 parameter setting =====
        # sensor mode related
        self.resp_EKF2_HGT_MODE = self.get_param_srv('EKF2_HGT_MODE')
        self.resp_EKF2_AID_MASK = self.get_param_srv('EKF2_AID_MASK')
        self.resp_EKF2_HGT_MODE.success = False
        self.resp_EKF2_AID_MASK.success = False

        # rangefinder related
        self.resp_EKF2_RNG_GATE = self.get_param_srv('EKF2_RNG_GATE')
        self.resp_EKF2_RNG_AID = self.get_param_srv('EKF2_RNG_AID')
        self.resp_EKF2_RNG_NOISE = self.get_param_srv('EKF2_RNG_NOISE')
        self.resp_EKF2_RNG_A_HMAX = self.get_param_srv('EKF2_RNG_A_HMAX')
        self.resp_EKF2_RNG_A_VMAX = self.get_param_srv('EKF2_RNG_A_VMAX')
        self.resp_EKF2_RNG_A_IGATE = self.get_param_srv('EKF2_RNG_A_IGATE')
        self.resp_EKF2_RNG_GATE.success = False
        self.resp_EKF2_RNG_AID.success = False
        self.resp_EKF2_RNG_NOISE.success = False
        self.resp_EKF2_RNG_A_HMAX.success = False
        self.resp_EKF2_RNG_A_VMAX.success = False
        self.resp_EKF2_RNG_A_IGATE.success = False

        # hardware setting related
        self.resp_EKF2_EV_POS_Z = self.get_param_srv('EKF2_EV_POS_Z')
        self.resp_EKF2_RNG_POS_Z = self.get_param_srv('EKF2_RNG_POS_Z')
        self.resp_EKF2_EV_POS_Z.success = False
        self.resp_EKF2_RNG_POS_Z.success = False

        # ======== ROS msgs declaration ========
        self.PX4_PARAM_msg = ParamValue()
        self.VIO_meas_sub = PoseWithCovarianceStamped()
        self.VIO_meas_pub = PoseWithCovarianceStamped()

        # ======== initialization ========
        # PX4 parameter setting
        self.init_PX4_param_common()

    def init_PX4_param_common(self): # ROS
        print('---------PX4 parameter setting start---------')
        # sensor mode related
        self.resp_EKF2_HGT_MODE = self.set_PX4_param('EKF2_HGT_MODE', 3, 'int', self.resp_EKF2_HGT_MODE)
        self.resp_EKF2_AID_MASK = self.set_PX4_param('EKF2_AID_MASK', 24, 'int', self.resp_EKF2_AID_MASK)

        # rangefinder related
        self.resp_EKF2_RNG_GATE = self.set_PX4_param('EKF2_RNG_GATE', 5.0, 'real', self.resp_EKF2_RNG_GATE)
        self.resp_EKF2_RNG_AID = self.set_PX4_param('EKF2_RNG_AID', 0, 'int', self.resp_EKF2_RNG_AID)
        self.resp_EKF2_RNG_NOISE = self.set_PX4_param('EKF2_RNG_NOISE', 0.1, 'real', self.resp_EKF2_RNG_NOISE)
        self.resp_EKF2_RNG_A_HMAX = self.set_PX4_param('EKF2_RNG_A_HMAX', 10.0, 'real', self.resp_EKF2_RNG_A_HMAX)
        self.resp_EKF2_RNG_A_VMAX = self.set_PX4_param('EKF2_RNG_A_VMAX', 1.0, 'real', self.resp_EKF2_RNG_A_VMAX)
        self.resp_EKF2_RNG_A_IGATE = self.set_PX4_param('EKF2_RNG_A_IGATE', 1.0, 'real', self.resp_EKF2_RNG_A_IGATE)

        # hardware setting related
        self.resp_EKF2_EV_POS_Z = self.set_PX4_param('EKF2_EV_POS_Z', 0.00000001, 'real', self.resp_EKF2_EV_POS_Z)
        self.resp_EKF2_RNG_POS_Z = self.set_PX4_param('EKF2_RNG_POS_Z', 0.00000001, 'real', self.resp_EKF2_RNG_POS_Z)

        print('------all PX4 parameter setting finished------')
        self.flag_PX4_param_initialization = 1

    def set_PX4_param(self, param_name, value, value_type, resp): # ROS
        # param_name : name of parameter to set (ex. 'EKF2_RNG_GATE')
        # value : set value (ex. 1, 0.3)
        # value_type : type of value (ex. 'real', 'int')
        # resp : variable to receive respose from the service
        if value_type == 'real':
            self.PX4_PARAM_msg.real = value
            self.PX4_PARAM_msg.integer = 0
        elif value_type == 'int':
            self.PX4_PARAM_msg.real = 0
            self.PX4_PARAM_msg.integer = value
        else:
            return -1

        while resp.success == False:
            resp = self.set_param_srv(param_name, self.PX4_PARAM_msg)
        resp.success = False

        return resp

    def cb_hgt_meas_source_selection_policy(self, req): # ROS
        # 0 : automatic (rangefinder on floor, VIO on obstacle), recover rangefinder usability on obstacle
        # 1 : automatic (rangefinder on floor, VIO on obstacle), use VIO on obstacle
        # 2 : rangefinder only (rangefinder on the both of floor and obstacle)
        # 3 : VIO only (VIO on the both of floor and obstacle)
        # how to call this service ??
        # --$ rosservice call hgt_meas_source_selection_policy 0
        if req.hgt_mode == 0 or req.hgt_mode == 1 or req.hgt_mode == 2 or req.hgt_mode == 3:
            self.meas_selec_policy = req.hgt_mode
            if req.hgt_mode == 0:
                self.del_weight_rf_vio = 0.05
            else:
                self.del_weight_rf_vio = 0.0
            return hgt_meas_source_selection_policyResponse(True, self.meas_selec_policy)
        else:
            return hgt_meas_source_selection_policyResponse(False, req.hgt_mode)

    def save_EKF2_pose(self, msg): # ROS
        self.EKF2_pose = msg
        qw = self.EKF2_pose.pose.orientation.w
        qx = self.EKF2_pose.pose.orientation.x
        qy = self.EKF2_pose.pose.orientation.y
        qz = self.EKF2_pose.pose.orientation.z

        # attitude transformation from Quaternion to Euler
        quaternion_list = [qx, qy, qz, qw]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion_list)

    def save_EKF2_vel(self, msg): # ROS
        self.EKF2_vel = msg

    def save_RF_and_decision(self, msg): # ROS
        self.RF_meas = msg

        self.dist_RF_floor = self.gaussian_dist(self.EKF2_pose, self.EKF2_vel, self.RF_meas.range, 1.0)
        prob_mode = self.update_mode_prob(dist_RF=self.dist_RF_floor, dist_VIO=self. dist_VIO)
        self.downward_status_decision(prob_mode=prob_mode[0])

        if self.Downward_Status == 1:
            self.dist_RF_obstacle = self.gaussian_dist(self.EKF2_pose, self.EKF2_vel, self.RF_meas.range - self.bias_RF / (np.cos(self.roll) * np.cos(self.pitch)), 1.0)

    def assign_PoseWithCovarianceStamped_msg(self, msg_out, msg_in):
        msg_out.header.frame_id =  msg_in.header.frame_id
        msg_out.header.seq = msg_in.header.seq
        msg_out.header.stamp.nsecs = msg_in.header.stamp.nsecs
        msg_out.header.stamp.secs = msg_in.header.stamp.secs
        msg_out.pose.covariance = msg_in.pose.covariance
        msg_out.pose.pose.orientation.w = msg_in.pose.pose.orientation.w
        msg_out.pose.pose.orientation.x = msg_in.pose.pose.orientation.x
        msg_out.pose.pose.orientation.y = msg_in.pose.pose.orientation.y
        msg_out.pose.pose.orientation.z = msg_in.pose.pose.orientation.z
        msg_out.pose.pose.position.x = msg_in.pose.pose.position.x
        msg_out.pose.pose.position.y = msg_in.pose.pose.position.y
        msg_out.pose.pose.position.z = msg_in.pose.pose.position.z

        return msg_out

    def sub_and_pub_VIO(self, msg): # ROS
        time_sub = rospy.Time.now()
        # subscribed message
        self.VIO_meas_sub = self.assign_PoseWithCovarianceStamped_msg(msg_out = self.VIO_meas_sub, msg_in = msg)
        self.VIO_meas_pub = self.assign_PoseWithCovarianceStamped_msg(msg_out = self.VIO_meas_pub, msg_in = msg)

        self.MA_VIO_z = self.moving_average_VIO(self.VIO_meas_sub.pose.pose.position.z)
        # self.dist_RF_floor = self.gaussian_dist(self.EKF2_pose, self.EKF2_vel, self.RF_meas.range, 1.0)

        if self.meas_selec_policy == 0 or self.meas_selec_policy == 1: # automatic
            if self.Downward_Status == 0: # on the floor
                if self.dist_RF_floor < self.GateLevel_RF or self.EKF2_pose.pose.position.z < self.min_height:
                    # VIO bias correction
                    self.bias_VIO = (self.MA_VIO_z - self.RF_meas.range * np.cos(self.roll) * np.cos(self.pitch))
                    self.hist_bias_VIO[0, 0:(self.bufflen_VIO_bias - 1)] = self.hist_bias_VIO[0, 1:(self.bufflen_VIO_bias)]
                    self.hist_bias_VIO[0, (self.bufflen_VIO_bias - 1)] = self.bias_VIO

                    # use rangefinder
                    self.VIO_meas_pub.pose.pose.position.z = self.RF_meas.range*np.cos(self.roll)*np.cos(self.pitch)

                    # log measurement source
                    self.meas_source = 0 # rangefinder
                else:
                    self.VIO_meas_pub.pose.pose.position.z = self.VIO_meas_sub.pose.pose.position.z - self.hist_bias_VIO[0,0]

                    # log measurement source
                    self.meas_source = 1  # VIO
            elif self.Downward_Status == 2: # transition
                # use VIO
                self.VIO_meas_pub.pose.pose.position.z = self.VIO_meas_sub.pose.pose.position.z - self.hist_bias_VIO[0,0]

                # log measurement source
                self.meas_source = 1 # VIO
            elif self.Downward_Status == 1: # on an obstacle
                self.VIO_meas_pub.pose.pose.position.z = self.VIO_meas_sub.pose.pose.position.z - self.hist_bias_VIO[0,0]

                # rangefinder bias correction
                if self.weight_rf_vio < 0.5 and self.meas_selec_policy == 0:
                    self.MA_RF_z = self.moving_average_RF(np.cos(self.roll)*np.cos(self.pitch)*self.RF_meas.range)
                    self.bias_RF = np.cos(self.roll)*np.cos(self.pitch)*self.RF_meas.range - self.VIO_meas_pub.pose.pose.position.z

                # rangefinder residual w.r.t. obstacle
                # self.dist_RF_obstacle = self.gaussian_dist(self.EKF2_pose, self.EKF2_vel, self.RF_meas.range - self.bias_RF/(np.cos(self.roll)*np.cos(self.pitch)), 1.0)

                # choose meas source
                if self.dist_RF_floor < self.GateLevel_RF:
                    self.VIO_meas_pub.pose.pose.position.z = (1-self.weight_rf_vio)*self.VIO_meas_pub.pose.pose.position.z + self.weight_rf_vio*(np.cos(self.roll)*np.cos(self.pitch)*self.RF_meas.range)

                    # log measurement source
                    if self.weight_rf_vio > 0.9:
                        self.meas_source = 0 # rangefinder
                    elif self.weight_rf_vio < 0.1:
                        self.meas_source = 1 # VIO
                    else:
                        self.meas_source = 2 # mixed
                else:
                    if self.dist_RF_obstacle > self.GateLevel_RF_onobstacle:
                        self.firstrun_MA_RF_z = 1
                        self.weight_rf_vio = 0.0

                    self.VIO_meas_pub.pose.pose.position.z = (1-self.weight_rf_vio)*self.VIO_meas_pub.pose.pose.position.z + self.weight_rf_vio*(np.cos(self.roll)*np.cos(self.pitch)*self.RF_meas.range - self.bias_RF)
                    if self.weight_rf_vio > 0.9:
                        # # VIO bias correction on an obstacle
                        self.bias_VIO = (self.MA_VIO_z - self.RF_meas.range * np.cos(self.roll) * np.cos(self.pitch) + self.bias_RF)
                        self.hist_bias_VIO[0,0:(self.bufflen_VIO_bias - 1)] = self.hist_bias_VIO[0, 1:(self.bufflen_VIO_bias)]
                        self.hist_bias_VIO[0, (self.bufflen_VIO_bias - 1)] = self.bias_VIO

                        # log measurement source
                        self.meas_source = 0 # rangefinder
                    elif self.weight_rf_vio < 0.1:
                        # log measurement source
                        self.meas_source = 1 # VIO
                    else:
                        # log measurement source
                        self.meas_source = 2 # mixed

                self.weight_rf_vio = self.linear_increase(self.weight_rf_vio, self.del_weight_rf_vio, 1.0)

        elif self.meas_selec_policy == 2: # rangefinder only
            # # VIO bias estimate and save it into buffer
            self.VIO_meas_pub.pose.pose.position.z = self.RF_meas.range * np.cos(self.roll) * np.cos(self.pitch)

            # log measurement source
            self.meas_source = 0  # rangefinder

        elif self.meas_selec_policy == 3: # VIO only
            self.VIO_meas_pub.pose.pose.position.z = self.VIO_meas_sub.pose.pose.position.z - self.hist_bias_VIO[0, 0]

            # log measurement source
            self.meas_source = 1  # VIO

        # some modification of VIO covariance (experimental tuning)
        self.VIO_meas_pub.pose.covariance = (0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.01)

        self.pub_vision_pose.publish(self.VIO_meas_pub)

        time_after_pub = rospy.Time.now()
        time_delay_rostime = time_after_pub - time_sub

        self.time_delay = time_delay_rostime.secs + time_delay_rostime.nsecs*1e-9
