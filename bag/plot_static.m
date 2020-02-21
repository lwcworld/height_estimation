function plot_static(data, fignum, time_interv)
figure(fignum);
subplot(3,2,1); hold on; grid on
plot(data.local_pose(:,end), data.local_pose(:,3), 'LineWidth', 2)
plot(data.RF_meas(:,end), data.RF_meas(:,1), 'LineWidth', 2)
plot(data.VIO_z(:,end), data.VIO_z(:,1), 'LineWidth', 2)
xlabel('time [sec]', 'FontSize', 15)
ylabel('altitude [m]', 'FontSize', 15)
legend({'estimated altitude', 'rangefinder meas', 'VIO'}, 'FontSize', 15)
xlim(time_interv)
ylim([-0.2 2.5])
title('altitude', 'FontSize', 15)

subplot(3,2,2); hold on; grid on
plot(data.res_RF_floor(:,end), data.res_RF_floor(:,1), 'LineWidth', 2)
plot(data.res_RF_obs(:,end), data.res_RF_obs(:,1), 'LineWidth', 2)
%plot(data.res_VIO_floor(:,end), data.res_VIO_floor(:,end), 'LineWidth', 2)
xlabel('time [sec]', 'FontSize', 15)
ylabel('residual', 'FontSize', 15)
legend({'RF-floor', 'RF-obs', 'VIO-floor'}, 'FontSize', 15)
xlim(time_interv)
title('residual', 'FontSize', 15)

subplot(3,2,3); hold on; grid on
plot(data.down_status(:,end), data.down_status(:,1), 'LineWidth', 2)
plot(data.meas_used(:,end), data.meas_used(:,1), 'LineWidth', 2)
xlabel('time [sec]', 'FontSize', 15)
ylim([-0.5 2.5])
xlim(time_interv)
legend({'down status : 0(floor) / 1(obstacle) / 2(transition)', 'used meas : 0(rf) / 1(VIO) / 2(mixed)'}, 'FontSize', 15)
title('down status & used measurement', 'FontSize', 15)

subplot(3,2,4); hold on; grid on
plot(data.down_status_prob_floor(:,end), data.down_status_prob_floor(:,1), 'LineWidth', 2)
plot(data.down_status_prob_floor(:,end), 1-data.down_status_prob_floor(:,1), 'LineWidth', 2)
xlabel('time [sec]', 'FontSize', 15)
ylabel('%', 'FontSize', 15)
legend({'floor', 'obstacle'}, 'FontSize', 15)
xlim(time_interv)
ylim([-0.05 1.05])
title('downward status probability', 'FontSize', 15)

subplot(3,2,5); hold on; grid on
plot(data.local_pose(:,end), data.local_pose(:,1), 'LineWidth', 2)
plot(data.local_pose(:,end), data.local_pose(:,2), 'LineWidth', 2)
plot(data.local_pose(:,end), data.local_pose(:,3), 'LineWidth', 2)
legend({'x', 'y', 'z'}, 'FontSize', 15)
xlim(time_interv)
title('local pose', 'FontSize', 15)

subplot(3,2,6); hold on; grid on
plot(data.bias_VIO_z(:,end), data.bias_VIO_z(:,1), 'LineWidth', 2)
plot(data.bias_RF(:,end), data.bias_RF(:,1), 'LineWidth', 2)
xlabel('time [sec]', 'FontSize', 15)
ylabel('[m]', 'FontSize', 15)
legend({'VIO bias', 'RF bias'}, 'FontSize', 15)
title('bias estimation', 'FontSize', 15)
xlim(time_interv)

figure(fignum+1); hold on; grid on 
plot(data.time_delay(:,end), data.time_delay(:,1), 'LineWidth', 2)
time_delay_filtered = nan(1, length(data.time_delay(:,1)));
for i = 1:length(time_delay_filtered)
    time_delay_filtered(1,i) = LPF(data.time_delay(i,1));
end
plot(data.time_delay(:,end), time_delay_filtered', 'LineWidth', 2)
xlabel('time [sec]', 'FontSize', 15)
ylabel('time delay [sec]', 'FontSize', 15)
xlim(time_interv)
title('processing delay', 'FontSize', 15)
legend({'raw', 'filtered'}, 'FontSize', 15)