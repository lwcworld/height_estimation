import numpy as np
from tf.transformations import euler_from_quaternion

from Params import Params
from Variables import Variables

class Utils(Params, Variables):
    def __init__(self):
        # set params
        Params.__init__(self)

        # declare variables
        Variables.__init__(self)

    def linear_increase(self, var, del_inc, saturation=1.0): # Util
        if var <= saturation:
            var = var + del_inc
        else:
            var = var
        return var

    def h_RF(self, q): # Util
        z = q[2]
        qw = q[3]
        qx = q[4]
        qy = q[5]
        qz = q[6]

        quaternion_list = [qx, qy, qz, qw]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion_list)

        m_hat = 1/(np.cos(self.roll) * np.cos(self.pitch)) * z

        return m_hat

    def h_VIO(self, q): # Util
        H = [[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]]

        m_hat = np.dot(H, q)

        return m_hat

    def gaussian_dist(self, msg_EKF2_pose, msg_EKF2_vel, msg_meas, S): # Util
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

        m = msg_meas
        m_hat = self.h_RF(q)

        if np.size(S) > 1:
            invS = np.linalg.inv(S)
        else:
            invS = 1/S

        mahal = np.dot(np.dot(m_hat-m, invS), m_hat-m)

        return mahal

    def downward_status_decision(self, prob_mode): # Util
        if prob_mode[0]>0.97:
            self.Downward_Status = 0
        elif prob_mode[1]>0.8:
            self.Downward_Status = 1
            self.firstrun_MA_RF_z = 1
        else:
            self.Downward_Status = 2

        if self.Downward_Status == 0:
            self.weight_rf_vio = 0.0
            self.bias_VIO = 0.0

    def update_mode_prob(self, dist_RF, dist_VIO): # Util
        if dist_RF < self.GateLevel_RF or self.EKF2_pose.pose.position.z < self.min_height:
            idx_status = 0  # (0:RF(o), 1:RF(x))
        else:
            idx_status = 1

        self.prob_mode = np.multiply(self.confusion_matrix[:,idx_status], self.prob_mode) / np.matmul(self.confusion_matrix[:,idx_status], self.prob_mode[0])

        min_prob = 0.01
        self.prob_mode = np.maximum(min_prob, self.prob_mode)
        self.prob_mode = self.prob_mode / np.sum(self.prob_mode)

        return self.prob_mode

    def moving_average_VIO(self, z): # Util
        if self.firstrun_MA_VIO_z == 1:
            self.VIO_z_buf = np.ones((1, self.num_MA_buf+1))*z
            self.prev_MA_VIO_z = z
            self.firstrun_MA_VIO_z = 0

        for m in range(0, self.num_MA_buf):
            self.VIO_z_buf[0,m] = self.VIO_z_buf[0,m+1]

        self.VIO_z_buf[0,self.num_MA_buf] = z
        avg = self.prev_MA_VIO_z + (z - self.VIO_z_buf[0,0])/self.num_MA_buf

        self.prev_MA_VIO_z = avg

        return avg

    def moving_average_RF(self, z): # Util
        if self.firstrun_MA_RF_z == 1:
            self.RF_z_buf = np.ones((1, self.num_MA_buf+1))*z
            self.prev_MA_RF_z = z
            self.firstrun_MA_RF_z = 0

        for m in range(0, self.num_MA_buf):
            self.RF_z_buf[0,m] = self.RF_z_buf[0,m+1]

        self.RF_z_buf[0,self.num_MA_buf] = z
        avg = self.prev_MA_RF_z + (z - self.RF_z_buf[0,0])/self.num_MA_buf

        self.prev_MA_RF_z = avg

        return avg

    def gcd(self, a, b):
        if b > a:
            tmp = a
            a = b
            b = tmp
        while b>0:
            c = b
            b = a%b
            a = c
        return a

    def lcm(self, a, b):
        return a*b // self.gcd(a,b)