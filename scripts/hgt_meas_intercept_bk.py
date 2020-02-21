#!/usr/bin/env python
# import sys
import numpy as np
import math
import rospy

from sensor_msgs.msg import NavSatFix, Imu, Range
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped, PoseWithCovarianceStamped
from mavros_msgs.srv import ParamSet, ParamGet, ParamPushResponse, ParamPullResponse, ParamPull
from mavros_msgs.msg import ParamValue
from tf.transformations import euler_from_quaternion

from std_msgs.msg import Int8, Int32MultiArray, Float64

from BGarage_RF.srv import *

class control_hgt_mode():
    def __init__(self):
        # common PX4 param sets
        self.PX4_HGT_MODE = 3
        self.PX4_PARAM_msg = ParamValue()

        self.set_param_srv   = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self.get_param_srv   = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        self.pub_vision_pose = rospy.Publisher('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped, queue_size=2)

        self.srv1 = rospy.Service('hgtmode_userrequest', hgtmode_userrequest, self.cb_hgtmode_userrequest)
        self.srv2 = rospy.Service('sensor_on_obstacle', sensor_on_obstacle, self.cb_sensor_on_obstacle)

        self.Dim_mode = 2 # [RF mode, VIO mosde]
        self.Dim_VIO = 7
        self.Dim_RF = 1

        self.GatePerDim = 0.05
        self.GateLevel_VIO = self.GatePerDim**self.Dim_VIO
        self.GateLevel_RF = self.GatePerDim**self.Dim_RF

        self.dist_RF  = 0.0
        self.dist_VIO = 0.0

        self.EKF2_pose = PoseStamped()
        self.EKF2_vel = TwistStamped()
        self.RF_meas = Range()
        self.VIO_meas_sub = PoseWithCovarianceStamped()
        self.VIO_meas_pub = PoseWithCovarianceStamped()

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # param & variables for mode probability update
        self.prob_mode = 1.0 / self.Dim_mode * np.ones((1, self.Dim_mode))

        # set confusion matrix
        self.HGT_CTRL_TYPE = 0
        self.confusion_matrix_user0 = np.array([[0.8, 0.1], [0.2, 0.9]])
        self.confusion_matrix_user2 = np.array([[1.0, 1.0], [0.0, 0.0]])
        self.confusion_matrix_user3 = np.array([[0.0, 0.0], [1.0, 1.0]])
        self.confusion_matrix = self.confusion_matrix_user0

        self.bias_height_of_VIO_meas_sub = 0.0
        self.isVIObias_fixed = 0
        self.bufflen_VIO_bias = 50
        self.bias_height_of_VIO_meas_sub_hist = np.zeros((1,self.bufflen_VIO_bias)) # about 30ms history
        self.bias_height_of_RF_meas_sub = 0.0
        self.weight_rf_vio = 0.0
        self.del_weight_rf_vio = 0.05

        self.num_MA_buf = 7
        self.VIO_z_buf = np.ones((1,self.num_MA_buf+1))
        self.prev_MA_z = 1.0
        self.MA_z = 0
        self.firstrun_MA_VIO_z = 1
        self.first_mode_rng = 1

        # responses from PX4 params to control
        self.resp_EKF2_RNG_GATE = self.get_param_srv('EKF2_RNG_GATE')
        self.resp_EKF2_RNG_AID = self.get_param_srv('EKF2_RNG_AID')
        self.resp_EKF2_RNG_NOISE = self.get_param_srv('EKF2_RNG_NOISE')
        self.resp_EKF2_RNG_GATE.success = False
        self.resp_EKF2_RNG_AID.success = False
        self.resp_EKF2_RNG_NOISE.success = False

        # responses from PX4 params to maintain
        self.resp_EKF2_HGT_MODE = self.get_param_srv('EKF2_HGT_MODE')
        self.resp_EKF2_AID_MASK = self.get_param_srv('EKF2_AID_MASK')
        self.resp_EKF2_RNG_A_HMAX = self.get_param_srv('EKF2_RNG_A_HMAX')
        self.resp_EKF2_RNG_A_VMAX = self.get_param_srv('EKF2_RNG_A_VMAX')
        self.resp_EKF2_RNG_A_IGATE = self.get_param_srv('EKF2_RNG_A_IGATE')
        self.resp_EKF2_HGT_MODE.success = False
        self.resp_EKF2_AID_MASK.success = False
        self.resp_EKF2_RNG_A_HMAX.success = False
        self.resp_EKF2_RNG_A_VMAX.success = False
        self.resp_EKF2_RNG_A_IGATE.success = False

        # set some PX4 parameters
        self.on_bias_correction = 1
        self.on_mode_change = 1
        self.init_PX4_param_common()

    def init_PX4_param_common(self):
        # set EKF2_AID_MASK (common)
        self.resp_EKF2_AID_MASK = self.set_PX4_param('EKF2_AID_MASK', 24, 'int', self.resp_EKF2_AID_MASK)

        # set EKF2_RNG_A_HMAX (common)
        self.resp_EKF2_RNG_A_HMAX = self.set_PX4_param('EKF2_RNG_A_HMAX', 10.0, 'real', self.resp_EKF2_RNG_A_HMAX)

        # set EKF2_RNG_A_VMAX (common)
        self.resp_EKF2_RNG_A_VMAX = self.set_PX4_param('EKF2_RNG_A_VMAX', 1.0, 'real', self.resp_EKF2_RNG_A_VMAX)

        # set EKF2_RNG_A_IGATE (common)
        self.resp_EKF2_RNG_A_IGATE = self.set_PX4_param('EKF2_RNG_A_IGATE', 1.0, 'real', self.resp_EKF2_RNG_A_IGATE)

        # set EKF2_HGT_MODE (common)
        self.resp_EKF2_HGT_MODE = self.set_PX4_param('EKF2_HGT_MODE', 3, 'int', self.resp_EKF2_HGT_MODE)

        # set EKF2_RNG_AID (common)
        self.resp_EKF2_RNG_AID = self.set_PX4_param('EKF2_RNG_AID', 0, 'int', self.resp_EKF2_RNG_AID)

    def set_PX4_param(self, param_name, value, value_type, resp):
        # param_name : name of parameter to set (ex. 'EKF2_RNG_GATE')
        # value : set value (ex. 1, 0.3)
        # value_type : type of value (ex. 'real', 'int')
        # resp : variable to receive respose from parameter setting service
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

    def cb_hgtmode_userrequest(self, req):
        if req.hgt_mode == 0 or req.hgt_mode == 2 or req.hgt_mode == 3:
            if req.hgt_mode == 0: # automatic
                self.HGT_CTRL_TYPE = 0
                self.confusion_matrix = self.confusion_matrix_user0
            elif req.hgt_mode == 2: # rangefinder only
                self.HGT_CTRL_TYPE = 2
                self.confusion_matrix = self.confusion_matrix_user2
            else: # vio only
                self.HGT_CTRL_TYPE = 3
                self.confusion_matrix = self.confusion_matrix_user3
            return hgtmode_userrequestResponse(True, self.HGT_CTRL_TYPE)
        else:
            return hgtmode_userrequestResponse(False, self.HGT_CTRL_TYPE)

    def cb_sensor_on_obstacle(self, req):
        if self.PX4_HGT_MODE != 3:
            return sensor_on_obstacleResponse(False, 2)
        else:
            if req.sensor == 1: # use VIO
                self.del_weight_rf_vio = 0.0
                return sensor_on_obstacleResponse(True, 1)
            elif req.sensor == 2:
                self.del_weight_rf_vio = 0.1
                return sensor_on_obstacleResponse(True, 2)

    def save_EKF2_pose(self, msg):
        self.EKF2_pose = msg
        qw = self.EKF2_pose.pose.orientation.w
        qx = self.EKF2_pose.pose.orientation.x
        qy = self.EKF2_pose.pose.orientation.y
        qz = self.EKF2_pose.pose.orientation.z

        quaternion_list = [qx, qy, qz, qw]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion_list)

    def save_EKF2_vel(self, msg):
        self.EKF2_vel = msg

    def save_RF(self, msg):
        self.RF_meas = msg

    def sub_and_pub_VIO(self, msg):
        self.VIO_meas_sub = msg
        self.VIO_meas_pub = self.VIO_meas_sub

        self.MA_z = self.moving_average_VIO_z(self.VIO_meas_sub.pose.pose.position.z)
        dist_RF = obj_HGT.gaussian_dist(self.EKF2_pose, self.EKF2_vel, self.RF_meas, 1.0)

        if self.PX4_HGT_MODE == 2:
            if dist_RF < self.GateLevel_RF:
                if self.isVIObias_fixed == 1:
                    self.isVIObias_fixed = 0

                self.bias_height_of_VIO_meas_sub = (self.MA_z - self.RF_meas.range * np.cos(self.roll) * np.cos(self.pitch))*self.on_bias_correction
                self.bias_height_of_VIO_meas_sub_hist[0, 0:(self.bufflen_VIO_bias - 1)] = self.bias_height_of_VIO_meas_sub_hist[0, 1:(self.bufflen_VIO_bias)]
                self.bias_height_of_VIO_meas_sub_hist[0, (self.bufflen_VIO_bias - 1)] = self.bias_height_of_VIO_meas_sub

                self.VIO_meas_pub.pose.pose.position.z = self.RF_meas.range*np.cos(self.roll)*np.cos(self.pitch)
            else:
                self.VIO_meas_pub.pose.pose.position.z = self.VIO_meas_sub.pose.pose.position.z - self.bias_height_of_VIO_meas_sub_hist[0,0]
                if self.isVIObias_fixed == 0:
                    self.bias_height_of_VIO_meas_sub_hist[0, :] = self.bias_height_of_VIO_meas_sub_hist[0, 0]
                    self.isVIObias_fixed = 1
        elif self.PX4_HGT_MODE == 3:
            self.VIO_meas_pub.pose.pose.position.z = self.VIO_meas_sub.pose.pose.position.z - self.bias_height_of_VIO_meas_sub_hist[0,0]
            if self.isVIObias_fixed == 0:
                self.bias_height_of_VIO_meas_sub_hist[0, :] = self.bias_height_of_VIO_meas_sub_hist[0, 0]
                self.isVIObias_fixed = 1


            if dist_RF < self.GateLevel_RF:
                self.VIO_meas_pub.pose.pose.position.z = (1-self.weight_rf_vio)*self.VIO_meas_pub.pose.pose.position.z + self.weight_rf_vio*(np.cos(self.roll)*np.cos(self.pitch)*self.RF_meas.range)
            else:
                # VIO z bias correction when agent is on obstacles
                if self.weight_rf_vio < 0.5:
                    self.bias_height_of_RF_meas_sub = np.cos(self.roll) * np.cos(self.pitch) * self.RF_meas.range - self.VIO_meas_pub.pose.pose.position.z
                self.VIO_meas_pub.pose.pose.position.z = (1-self.weight_rf_vio)*self.VIO_meas_pub.pose.pose.position.z + self.weight_rf_vio*(np.cos(self.roll)*np.cos(self.pitch)*self.RF_meas.range - self.bias_height_of_RF_meas_sub)

            self.weight_rf_vio = self.linear_increase(self.weight_rf_vio, self.del_weight_rf_vio, 1.0)

        self.VIO_meas_pub.pose.covariance = (0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.01)

        # self.VIO_meas_pub.pose.covariance = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #                                      0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        #                                      0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        #                                      0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
        #                                      0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
        #                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.1)

        self.pub_vision_pose.publish(self.VIO_meas_pub)

    def linear_increase(self, var, del_inc, saturation=1.0):
        if var <= saturation:
            var = var + del_inc
        else:
            var = var

        return var

    def h_RF(self, q):
        z = q[2]
        qw = q[3]
        qx = q[4]
        qy = q[5]
        qz = q[6]

        quaternion_list = [qx, qy, qz, qw]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion_list)

        m_hat = 1/(np.cos(self.roll) * np.cos(self.pitch)) * z

        return m_hat

    def h_VIO(self, q):
        H = [[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]]

        m_hat = np.dot(H, q)

        return m_hat

    def gaussian_dist(self, msg_EKF2_pose, msg_EKF2_vel, msg_meas, S):
        # agent state estimate
        x = msg_EKF2_pose.pose.position.x
        y = msg_EKF2_pose.pose.position.y
        z = msg_EKF2_pose.pose.position.z
        qw = msg_EKF2_pose.pose.orientation.w
        qx = msg_EKF2_pose.pose.orientation.x
        qy = msg_EKF2_pose.pose.orientation.y
        qz = msg_EKF2_pose.pose.orientation.z
        vx = msg_EKF2_vel.twist.linear.x
        vy = msg_EKF2_vel.twist.linear.y
        vz = msg_EKF2_vel.twist.linear.z
        d_roll = msg_EKF2_vel.twist.angular.x
        d_pitch = msg_EKF2_vel.twist.angular.y
        d_yaw = msg_EKF2_vel.twist.angular.z
        q = [x, y, z, qw, qx, qy, qz, vx, vy, vz, d_roll, d_pitch, d_yaw]

        if msg_meas._type=='sensor_msgs/Range': # Rangefinder
            m = msg_meas.range
            m_hat = self.h_RF(q)
        elif msg_meas._type=='geometry_msgs/PoseWithCovarianceStamped': # VIO
            m = [msg_meas.pose.pose.position.x, msg_meas.pose.pose.position.y, msg_meas.pose.pose.position.z, msg_meas.pose.pose.orientation.w, msg_meas.pose.pose.orientation.x, msg_meas.pose.pose.orientation.y, msg_meas.pose.pose.orientation.z]
            m_hat = self.h_VIO(q)

        if np.size(S) > 1:
            invS = np.linalg.inv(S)
        else:
            invS = 1/S

        mahal = np.dot(np.dot(m_hat-m, invS), m_hat-m)

        return mahal

    def HGT_MODE_controller(self, prob_mode):
        if prob_mode[0]>0.5:
            self.PX4_HGT_MODE = 2
        else:
            self.PX4_HGT_MODE = 3

        if self.PX4_HGT_MODE == 2:
            self.weight_rf_vio = 0.0
            self.bias_height_of_RF_meas_sub = 0.0

    def update_mode_prob(self, dist_RF, dist_VIO):
        if dist_RF < self.GateLevel_RF or self.EKF2_pose.pose.position.z < 1.0:
            idx_status = 0  # (0:RF(o), 1:RF(x))
        else:
            idx_status = 1

        self.prob_mode = np.multiply(self.confusion_matrix[:,idx_status], self.prob_mode) / np.matmul(self.confusion_matrix[:,idx_status], self.prob_mode[0])

        min_prob = 0.01
        self.prob_mode = np.maximum(min_prob, self.prob_mode)
        self.prob_mode = self.prob_mode / np.sum(self.prob_mode)

        return self.prob_mode

    def moving_average_VIO_z(self, z):
        if self.firstrun_MA_VIO_z == 1:
            self.VIO_z_buf = np.ones((1, self.num_MA_buf+1))*self.VIO_meas_sub.pose.pose.position.z
            self.prev_MA_z = self.VIO_meas_sub.pose.pose.position.z
            self.firstrun_MA_VIO_z = 0

        for m in range(0, self.num_MA_buf):
            self.VIO_z_buf[0,m] = self.VIO_z_buf[0,m+1]

        self.VIO_z_buf[0,self.num_MA_buf] = z
        avg = self.prev_MA_z + (z - self.VIO_z_buf[0,0])/self.num_MA_buf

        self.prev_MA_z = avg

        return avg

if __name__ == '__main__':
    rospy.init_node('HGT_MODE_CONTROLLER')

    obj_HGT = control_hgt_mode()

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, obj_HGT.save_EKF2_pose)
    rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, obj_HGT.save_EKF2_vel)
    # rospy.Subscriber("/mavros/distance_sensor/lidarlite_pub", Range, obj_HGT.save_RF) # type_m = 1
    rospy.Subscriber("/mavros/px4flow/ground_distance", Range, obj_HGT.save_RF)  # type_m = 1
    rospy.Subscriber("/vio_pose_in", PoseWithCovarianceStamped, obj_HGT.sub_and_pub_VIO) # type_m = 2

    pub_HGT_MODE_mahaldist_RF = rospy.Publisher('/datalog/mahaldist_RF', Float64, queue_size=2)
    pub_HGT_MODE_mahaldist_VIO = rospy.Publisher('/datalog/mahaldist_VIO', Float64, queue_size=2)
    pub_HGT_MODE_est_VIO_z_bias = rospy.Publisher('/datalog/est_VIO_z_bias', Float64, queue_size=2)
    pub_HGT_MODE_prob_RF = rospy.Publisher('/datalog/RF_MODE_prob', Float64, queue_size=2)
    pub_HGT_MODE_prob_VIO = rospy.Publisher('/datalog/VIO_MODE_prob', Float64, queue_size=2)
    pub_HGT_MODE = rospy.Publisher('/datalog/HGT_MODE', Int8, queue_size=2)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # S = np.diag([1, 1, 1, 1, 1, 1, 1])
        # obj_HGT.dist_VIO = obj_HGT.gaussian_dist(obj_HGT.EKF2_pose, obj_HGT.EKF2_vel, obj_HGT.VIO_meas_pub, S)
        obj_HGT.dist_RF = obj_HGT.gaussian_dist(obj_HGT.EKF2_pose, obj_HGT.EKF2_vel, obj_HGT.RF_meas, 1)

        prob_mode = obj_HGT.update_mode_prob(dist_RF=obj_HGT.dist_RF, dist_VIO=obj_HGT.dist_VIO)

        obj_HGT.HGT_MODE_controller(prob_mode=prob_mode[0])

        print('--')
        print(['dist_RF  : ', round(obj_HGT.dist_RF,3)])
        print(['dist_VIO : ', round(obj_HGT.dist_VIO,3)])
        print(['bias_VIO_z : ', round(obj_HGT.bias_height_of_VIO_meas_sub_hist[0,0],3)])
        print(['rangefinder meas : ', round(obj_HGT.RF_meas.range,3)])
        print(['HGT mode prob : ', round(obj_HGT.prob_mode[0,0],3), round(obj_HGT.prob_mode[0,1],3)])
        print(['weight RF vs VIO : ', round(obj_HGT.weight_rf_vio,3)])
        print(['bias RF : ', round(obj_HGT.bias_height_of_RF_meas_sub,3)])
        print(['current HGT mode : ', round(obj_HGT.PX4_HGT_MODE,3)])

        # log using ROS msgs
        pub_HGT_MODE_mahaldist_RF.publish(obj_HGT.dist_RF)
        pub_HGT_MODE_mahaldist_VIO.publish(obj_HGT.dist_VIO)
        pub_HGT_MODE_est_VIO_z_bias.publish(obj_HGT.bias_height_of_VIO_meas_sub_hist[0,0])
        pub_HGT_MODE_prob_RF.publish(obj_HGT.prob_mode[0,0])
        pub_HGT_MODE_prob_VIO.publish(obj_HGT.prob_mode[0,1])
        pub_HGT_MODE.publish(obj_HGT.PX4_HGT_MODE)

        rate.sleep()
