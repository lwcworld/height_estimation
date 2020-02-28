#!/usr/bin/env python

import numpy as np
import math
import rospy

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import Float64

from coordinate_transform import coordinate_transform

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

if __name__ == '__main__':
    rospy.init_node('gen_vision_pose')

    gen_VIO_object = gen_VIO()
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, gen_VIO_object.conv_GPS_to_Cart)
    rospy.Subscriber("/mavros/imu/data", Imu, gen_VIO_object.conv_IMU_to_orient)

    VIO_publisher = rospy.Publisher("/vio_pose_in", PoseWithCovarianceStamped, queue_size=2)
    VIO_z_bias_publisher = rospy.Publisher("/datalog/VIO_z_bias", Float64, queue_size=2)

    VIO_msg = PoseWithCovarianceStamped()

    i_seq = 1
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        VIO_msg.header.frame_id = 'map'
        VIO_msg.header.seq = i_seq
        i_seq += 1
        VIO_msg.header.stamp = rospy.Time.now()
        VIO_msg.pose.covariance = (0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001)
        VIO_msg.pose.pose.position.x = gen_VIO_object.x
        VIO_msg.pose.pose.position.y = gen_VIO_object.y
        VIO_msg.pose.pose.position.z = gen_VIO_object.z + gen_VIO_object.bias_VIO_z
        VIO_msg.pose.pose.orientation = gen_VIO_object.orientation

        gen_VIO_object.bias_VIO_z = gen_VIO_object.bias_VIO_z - 0.0002 # bias increases

        VIO_publisher.publish(VIO_msg)
        VIO_z_bias_publisher.publish(gen_VIO_object.bias_VIO_z)

        rate.sleep()