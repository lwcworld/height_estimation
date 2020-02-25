#!/usr/bin/env python
import numpy as np

from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped

class Variables(object):
    def __init__(self):
        # ======== state variables ========
        # downward status (floor:0 / obstacle:1 / transition:2)
        self.Downward_Status = 0

        # residual of rangefinder & VIO measurements
        self.dist_RF_floor  = 0.0
        self.dist_RF_obstacle = 0.0
        self.dist_VIO = 0.0 # #not used now...

        # agent pose
        self.EKF2_pose = PoseStamped()
        self.EKF2_pose_on_obstacle = PoseStamped()
        self.EKF2_vel = TwistStamped()

        # rangefinder measurement
        self.RF_meas = Range()

        # subscribed & to be published VIO data
        self.VIO_meas_sub = PoseWithCovarianceStamped()
        self.VIO_meas_pub = PoseWithCovarianceStamped()

        # attitude of agent
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # common PX4 param sets
        self.meas_source = 0 # 0: rangefinder  // 1: VIO

        # downward status probability ([floor / VIO])
        self.prob_mode = 1.0 / self.Dim_mode * np.ones((1, self.Dim_mode))

        # measurement source selection
        self.meas_selection_policy = 0 # 0: automatic // 1: rangefinder only // 2: VIO only

        # VIO bias
        self.bias_height_of_VIO_meas_sub = 0.0
        self.isVIObias_fixed = 0
        self.bias_height_of_VIO_meas_sub_hist = np.zeros((1,self.bufflen_VIO_bias))

        # rangefinder bias
        self.bias_height_of_RF_meas_sub = 0.0

        # VIO -> rangefinder weight
        self.weight_rf_vio = 0.0

        # moving average related
        self.VIO_z_buf = np.ones((1,self.num_MA_buf+1))
        self.firstrun_MA_VIO_z = 1
        self.firstrun_MA_RF_z = 1
        self.prev_MA_z = 1.0
        self.MA_VIO_z = 0
        self.MA_RF_z = 0

        #
        self.time_delay = 0

        #
        self.first_init_DS1 = 0