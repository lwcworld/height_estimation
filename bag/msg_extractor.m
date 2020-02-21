function msg = msg_extractor(bag, time)
global hardware
start_t = time(1);
final_t = time(2);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/datalog/des_pose');
msg.des_pose = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/datalog/res_RF_floor');
msg.res_RF_floor = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/datalog/res_RF_floor');
msg.res_RF_floor = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/datalog/res_RF_obs');
msg.res_RF_obs = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/datalog/res_VIO_floor');
msg.res_VIO_floor = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/datalog/bias_VIO_z');
msg.bias_VIO_z = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/datalog/bias_RF');
msg.bias_RF = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/datalog/weight_RF_VIO');
msg.weight_RF_VIO = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/datalog/down_status_prob_floor');
msg.down_status_prob_floor = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/datalog/down_status_prob_obstacle');
msg.down_status_prob_obstacle = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/datalog/down_status');
msg.down_status = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/datalog/meas_used');
msg.meas_used = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/datalog/time_delay');
msg.time_delay = readMessages(bSel);

if hardware == 0
    bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/mavros/distance_sensor/lidarlite_pub');
else    
    bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/mavros/px4flow/ground_distance');
end
msg.RF_meas = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/mavros/local_position/pose');
msg.local_pose = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/mavros/global_position/raw/fix');
msg.gps_raw = readMessages(bSel);

bSel = select(bag, 'Time', [bag.StartTime+start_t bag.StartTime + final_t], 'Topic', '/mavros/vision_pose/pose_cov');
msg.VIO_z = readMessages(bSel);