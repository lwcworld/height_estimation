#!/usr/bin/env python
import rospy
from Rangefinder_aided_VIO import Rangefinder_aided_VIO
from log_pub_print import log_pub_print

if __name__ == '__main__':
    rospy.init_node('Rangefinder_aided_VIO')

    # RAVIO : Rangefinder aided VIO
    obj_RAVIO = Rangefinder_aided_VIO()
    obj_log_pub_print = log_pub_print()

    # set log param
    is_print_log = 1
    is_pub_log = 1
    freq_print = 10
    freq_pub = 100
    freq_log_timeline = obj_RAVIO.lcm(freq_print, freq_pub)

    rate = rospy.Rate(freq_log_timeline)
    count = 1

    rospy.sleep(0.5)
    while not rospy.is_shutdown():
        if is_print_log == 1 and count%(freq_log_timeline/freq_print)==0:
            obj_log_pub_print.log_print(res_RF_floor = obj_RAVIO.dist_RF_floor,
                                                  res_RF_obs = obj_RAVIO.dist_RF_obstacle,
                                                  res_VIO_floor = obj_RAVIO.dist_VIO,
                                                  meas_RF = obj_RAVIO.RF_meas.range,
                                                  bias_VIO_z = obj_RAVIO.bias_height_of_VIO_meas_sub_hist[0,0],
                                                  bias_RF = obj_RAVIO.bias_height_of_RF_meas_sub,
                                                  weight_RF_VIO = obj_RAVIO.weight_rf_vio,
                                                  down_status_prob = obj_RAVIO.prob_mode,
                                                  down_status = obj_RAVIO.Downward_Status,
                                                  meas_used = obj_RAVIO.meas_source,
                                                  time_delay = obj_RAVIO.time_delay)

        # log publish
        if is_pub_log == 1 and count%(freq_log_timeline/freq_pub)==0:
            obj_log_pub_print.log_publish(res_RF_floor = obj_RAVIO.dist_RF_floor,
                                                    res_RF_obs = obj_RAVIO.dist_RF_obstacle,
                                                    res_VIO_floor = obj_RAVIO.dist_VIO,
                                                    meas_RF = obj_RAVIO.RF_meas.range,
                                                    bias_VIO_z = obj_RAVIO.bias_height_of_VIO_meas_sub_hist[0,0],
                                                    bias_RF = obj_RAVIO.bias_height_of_RF_meas_sub,
                                                    weight_RF_VIO = obj_RAVIO.weight_rf_vio,
                                                    down_status_prob = obj_RAVIO.prob_mode,
                                                    down_status = obj_RAVIO.Downward_Status,
                                                    meas_used = obj_RAVIO.meas_source,
                                                    time_delay = obj_RAVIO.time_delay)

        if count > freq_log_timeline:
            count = 0
        else:
            count = count + 1

        rate.sleep()
