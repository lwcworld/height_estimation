clc;clear;close all
global hardware
hardware = 0; % 0: SITL , 1: hardware
IP_master = 'localhost';
rosinit(IP_master, 'NodeName', 'plot')
disp('ros init completed')

%%
file_name = '_2020-02-19-15-57-08.bag';
start_t = 0;
final_t = 50;
disp(['---start ', file_name ,' parsing'])
bag = rosbag(file_name);
msg_bundle = msg_extractor(bag, [start_t final_t]);
disp(['---', file_name, ' parse completed'])

%%
disp('---start data organization')
data = data_organize(msg_bundle, [start_t final_t]);
disp('---data organize completed')

%%
close all
plot_static(data, 1, [start_t final_t])
