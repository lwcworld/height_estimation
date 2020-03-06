#!/bin/bash

tmux has-session -t BGARAGE 
if [ $? == 0 ]; then
    tmux kill-session -t BGARAGE
fi
tmux new-session -s BGARAGE -n editor -d # new session

# divide into 4 rows 
tmux split-window -v -t 0
tmux split-window -v -t 0
tmux split-window -v -t 1

# divide into 2 columns for 4 panes
tmux split-window -h -t 0
tmux split-window -h -t 0
tmux split-window -h -t 2
tmux split-window -h -t 2
tmux split-window -h -t 0
tmux split-window -h -t 2

# divide into 2 columns for left panes on each side
tmux split-window -h -t 4
tmux split-window -h -t 6
tmux split-window -h -t 4
tmux split-window -h -t 6

# divide large pane into 2 columns
tmux split-window -h -t 1

# bottom pane (htop)
tmux resize-pane -D -t 3 7
tmux send-keys -t 3 htop C-m

# first two rows
tmux resize-pane -D -t 2 6
tmux resize-pane -D -t 0 3

# upper right corner
tmux split-window -v -t 10

## shrink 1, 2, 4, 5 panes horizontally
#tmux resize-pane -L -t 0 5
#tmux resize-pane -L -t 6 5
#tmux resize-pane -L -t 1 10
#tmux resize-pane -L -t 7 10
#tmux resize-pane -L -t 2 1
#tmux resize-pane -L -t 8 1

#tmux resize-pane -L -t 3 5
#tmux resize-pane -L -t 9 5
#tmux resize-pane -L -t 4 10
#tmux resize-pane -L -t 10 10

tmux resize-pane -R -t 8 2
tmux resize-pane -R -t 9 2
tmux resize-pane -D -t 3 1

# RESIZE for KENCO
tmux resize-pane -U -t 0 4
tmux resize-pane -U -t 8 4
tmux resize-pane -U -t 5 4

tmux resize-pane -U -t 2 6
tmux resize-pane -U -t 9 6
tmux resize-pane -U -t 7 6


# scripts to run

# roscore
tmux send-keys -t 0 roscore C-m
sleep 4 # to roscore to run first

# mavros
tmux send-keys -t 11 "cd /home/nvidia/scripts" C-m
tmux send-keys -t 11 /home/nvidia/scripts/mavros.sh C-m

# local position
tmux send-keys -t 2 /home/nvidia/scripts/echo_local_pos.sh C-m

# vision position
tmux send-keys -t 9 /home/nvidia/scripts/echo_vision_pos_cov.sh C-m

# ZED or ZED VIO 
#tmux send-keys -t 5 "cd /home/nvidia/scripts" C-m # it was run
#tmux send-keys -t 5 /home/nvidia/scripts/zed.sh C-m # it was run
#tmux send-keys -t 5 /home/nvidia/scripts/zed_vio.sh C-m

# image proc
#tmux send-keys -t 5 "ROS_NAMESPACE=camera rosrun image_proc image_proc" C-m

# mavlink shell
tmux send-keys -t 6 "cd /home/nvidia/scripts" C-m
#tmux send-keys -t 6 /home/nvidia/scripts/mavlink_shell.sh

# lidarlite echo 
tmux send-keys -t 13 "cd /home/nvidia/scripts" C-m
tmux send-keys -t 13 "rostopic echo -c /mavros/distance_sensor/lidarlite_pub" C-m

# T265 or T265 VIO
tmux send-keys -t 7 "cd /home/nvidia/scripts" C-m
#tmux send-keys -t 7 /home/nvidia/scripts/t265.sh C-m
tmux send-keys -t 7 /home/nvidia/scripts/t265_vio.sh C-m
#tmux send-keys -t 7 /home/nvidia/scripts/t265_vio_wc.sh C-m

# zed svo START recording
#tmux send-keys -t 15 /home/nvidia/scripts/record_start_zed_svo.sh \  /data/baylands_park

# zed svo STOP recording 
#tmux send-keys -t 12 /home/nvidia/scripts/record_stop_zed_svo.sh

# bag
#tmux send-keys -t 10 /home/nvidia/scripts/record_collision_avoidance.sh \  /data/baylands_park

# collision checker
#tmux send-keys -t 14 /home/nvidia/scripts/flir_std.sh #C-m 
#tmux send-keys -t 14 ./scripts/collision_checker.sh \  true
tmux send-keys -t 14 "source workspaces/hgt_est_wc/devel/setup.bash" C-m

# ACTUAL SCRIPT
# tmux send-keys -t 1 "cd /home/nvidia/scripts" C-m
# tmux send-keys -t 1 ./test.sh
#tmux send-keys -t 1 "source workspaces/tracey_ws/devel/setup.bash" C-m
tmux send-keys -t 1 "source workspaces/hgt_est_wc/devel/setup.bash" C-m
#tmux send-keys -t 1 "rosrun guidance_planning test_waypoint_outdoor.py"

################################################################
tmux attach -t BGARAGE # needed to run
