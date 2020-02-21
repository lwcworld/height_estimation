function data = data_organize(msg, time)
data.des_pose = [cellfun(@(m) double(m.Pose.Position.X), msg.des_pose), cellfun(@(m) double(m.Pose.Position.Y), msg.des_pose), cellfun(@(m) double(m.Pose.Position.Z), msg.des_pose)];
data.des_pose = add_timeinfo(data.des_pose, time);

data.res_RF_floor = cellfun(@(m) double(m.Data), msg.res_RF_floor);
data.res_RF_floor = add_timeinfo(data.res_RF_floor, time);

data.res_RF_floor = cellfun(@(m) double(m.Data), msg.res_RF_floor);
data.res_RF_floor = add_timeinfo(data.res_RF_floor, time);

data.res_RF_obs = cellfun(@(m) double(m.Data), msg.res_RF_obs);
data.res_RF_obs = add_timeinfo(data.res_RF_obs, time);

data.res_VIO_floor = cellfun(@(m) double(m.Data), msg.res_VIO_floor);
data.res_VIO_floor = add_timeinfo(data.res_VIO_floor, time);

data.bias_VIO_z = cellfun(@(m) double(m.Data), msg.bias_VIO_z);
data.bias_VIO_z = add_timeinfo(data.bias_VIO_z, time);

data.bias_RF = cellfun(@(m) double(m.Data), msg.bias_RF);
data.bias_RF = add_timeinfo(data.bias_RF, time);

data.weight_RF_VIO = cellfun(@(m) double(m.Data), msg.weight_RF_VIO);
data.weight_RF_VIO = add_timeinfo(data.weight_RF_VIO, time);

data.down_status_prob_floor = cellfun(@(m) double(m.Data), msg.down_status_prob_floor);
data.down_status_prob_floor = add_timeinfo(data.down_status_prob_floor, time);

data.down_status_prob_obstacle = cellfun(@(m) double(m.Data), msg.down_status_prob_obstacle);
data.down_status_prob_obstacle = add_timeinfo(data.down_status_prob_obstacle, time);

data.down_status = cellfun(@(m) double(m.Data), msg.down_status);
data.down_status = add_timeinfo(data.down_status, time);

data.meas_used = cellfun(@(m) double(m.Data), msg.meas_used);
data.meas_used = add_timeinfo(data.meas_used, time);

data.time_delay = cellfun(@(m) double(m.Data), msg.time_delay);
data.time_delay = add_timeinfo(data.time_delay, time);

data.RF_meas = cellfun(@(m) double(m.Range_), msg.RF_meas);
% data.RF_meas = cellfun(@(m) double(m.Range_), msg.RF_meas);
data.RF_meas = add_timeinfo(data.RF_meas, time);

data.local_pose = [cellfun(@(m) double(m.Pose.Position.X), msg.local_pose), cellfun(@(m) double(m.Pose.Position.Y), msg.local_pose), cellfun(@(m) double(m.Pose.Position.Z), msg.local_pose)];
data.local_pose = add_timeinfo(data.local_pose, time);

data.gps_alt = cellfun(@(m) double(m.Altitude), msg.gps_raw);
data.gps_alt = add_timeinfo(data.gps_alt, time);
data.gps_alt(:,1) = data.gps_alt(:,1) - 535.32;

data.VIO_z = cellfun(@(m) double(m.Pose.Pose.Position.Z), msg.VIO_z);
data.VIO_z = add_timeinfo(data.VIO_z, time);