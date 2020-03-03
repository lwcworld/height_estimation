#!/usr/bin/env python

import numpy as np
import math
import rospy

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

from coordinate_transform import coordinate_transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler


CT = coordinate_transform()

class gen_VIO():
    def __init__(self):
        self.is_ref_init = 0

        self.lat_ref = 0
        self.lon_ref = 0
        self.alt_ref = 0

        self.x = 0
        self.y = 0
        self.z = 0
        self.orientation = Quaternion

        self.bias_VIO_z = 0.0

        self.i_seq = 1

        self.VIO_msg = PoseWithCovarianceStamped()

        self.input_noise = 0

        self.VIO_publisher = rospy.Publisher("/vio_pose_in", PoseWithCovarianceStamped, queue_size=2)
        self.VIO_z_bias_publisher = rospy.Publisher("/datalog/VIO_z_bias", Float64, queue_size=2)

    def conv_GPS_to_Cart(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        if self.is_ref_init == 0:
            self.lat_ref = lat
            self.lon_ref = lon
            self.alt_ref = alt
            self.is_ref_init = 1

        self.x, self.y, self.z = CT.geodetic_to_enu(lat, lon, alt, self.lat_ref, self.lon_ref, self.alt_ref)

    def conv_IMU_to_orient(self, msg):
        self.orientation = msg.orientation

    def sub_pub_GT_based_VIO(self, msg):
        self.VIO_msg.header.frame_id = 'map'
        self.VIO_msg.header.seq = self.i_seq
        self.VIO_msg.header.stamp = rospy.Time.now()
        self.VIO_msg.pose.covariance = (0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001)
        self.VIO_msg.pose.pose.position.x = msg.pose.pose.position.x + np.random.randn(1,1)*math.sqrt(0.001)*self.input_noise
        self.VIO_msg.pose.pose.position.y = msg.pose.pose.position.y + np.random.randn(1,1)*math.sqrt(0.001)*self.input_noise
        self.VIO_msg.pose.pose.position.z = msg.pose.pose.position.z + np.random.randn(1,1)*math.sqrt(0.001)*self.input_noise + self.bias_VIO_z

        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z

        # attitude transformation from Quaternion to Euler
        quaternion_list = [qx, qy, qz, qw]

        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)

        roll = roll + np.random.randn(1,1) * math.sqrt(0.00001)*self.input_noise
        pitch = pitch + np.random.randn(1, 1) * math.sqrt(0.00001)*self.input_noise
        yaw = yaw + np.random.randn(1, 1) * math.sqrt(0.00001)*self.input_noise

        q = quaternion_from_euler(roll, pitch, yaw)

        self.VIO_msg.pose.pose.orientation.x = q[0]
        self.VIO_msg.pose.pose.orientation.y = q[1]
        self.VIO_msg.pose.pose.orientation.z = q[2]
        self.VIO_msg.pose.pose.orientation.w = q[3]

        self.VIO_publisher.publish(self.VIO_msg)
        self.VIO_z_bias_publisher.publish(self.bias_VIO_z)

        self.bias_VIO_z = self.bias_VIO_z - 0.0002
        self.i_seq += 1

if __name__ == '__main__':
    rospy.init_node('gen_vision_pose')

    gen_VIO_object = gen_VIO()
    rospy.Subscriber("/ground_truth_pose", Odometry, gen_VIO_object.sub_pub_GT_based_VIO)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()