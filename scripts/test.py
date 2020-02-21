#!/usr/bin/env python
import numpy as np
import rospy

from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped
from mavros_msgs.srv import ParamSet, ParamGet
from mavros_msgs.msg import ParamValue

from tf.transformations import euler_from_quaternion

from Height_Estimation.srv import *



def sub_and_pub_VIO(msg):
    time_sub = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9
    time_now = rospy.Time.now()
    time_now = time_now.secs + time_now.nsecs*1e-9
    print(time_now - time_sub)


if __name__ == '__main__':
    rospy.init_node('test')
    rospy.Subscriber("/vio_pose_in", PoseWithCovarianceStamped, sub_and_pub_VIO)
    while not rospy.is_shutdown():
        pass
