#!/bin/bash

rosbag record -b 0 -o ./ \
    /diagnostics \
    /datalog/des_pose \
    /datalog/res_RF_floor \
    /datalog/res_RF_obs \
    /datalog/res_VIO_floor \
    /datalog/meas_RF \
    /datalog/bias_VIO_z \
    /datalog/bias_RF \
    /datalog/weight_RF_VIO \
    /datalog/down_status_prob_floor \
    /datalog/down_status_prob_obstacle \
    /datalog/down_status \
    /datalog/meas_used \
    /datalog/time_delay \
    /mavros/distance_sensor/lidarlite_pub \
    /mavros/px4flow/ground_distance\
    /mavros/local_position/pose \
    /mavros/global_position/raw/fix \
    /mavros/vision_pose/pose_cov
    
