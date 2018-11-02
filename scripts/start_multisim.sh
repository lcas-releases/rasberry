#!/bin/bash

SESSION=$USER
DISPLAY=0

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'mongo'
tmux new-window -t $SESSION:2 -n 'simulation'
tmux new-window -t $SESSION:3 -n 'map_server'
tmux new-window -t $SESSION:4 -n 't1-move_base'
tmux new-window -t $SESSION:5 -n 't2-move_base'
#tmux new-window -t $SESSION:6 -n 'navigation'
#tmux new-window -t $SESSION:7 -n 'rviz'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "DISPLAY=:$DISPLAY roslaunch mongodb_store mongodb_store.launch db_path:=$1"

tmux select-window -t $SESSION:2
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_bringup robot_bringup_multisim.launch with_actors:=false robot_name_1:=thorvald_001 robot_pose_x_1:=0.0 robot_pose_y_1:=0.0 robot_pose_Y_1:=-1.7 robot_name_2:=thorvald_002 robot_pose_x_2:=2.0 robot_pose_y_2:=4.0 robot_pose_Y_2:=1.7"

tmux select-window -t $SESSION:3
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_move_base map_server.launch map:=$(rospack find rasberry_navigation)/maps/riseholme_sim.yaml"

tmux select-window -t $SESSION:4
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_move_base move_base_teb_multisim.launch robot_name:=thorvald_001 robot_pose_x:=0.0 robot_pose_y:=0.0 robot_pose_Y:=-1.7"
#tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_move_base move_base_dwa_multisim.launch robot_name:=thorvald_001 robot_pose_x:=0.0 robot_pose_y:=0.0 robot_pose_Y:=-1.7"

tmux select-window -t $SESSION:5
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_move_base move_base_teb_multisim.launch robot_name:=thorvald_002 robot_pose_x:=2.0 robot_pose_y:=4.0 robot_pose_Y:=1.7"
#tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_move_base move_base_dwa_multisim.launch robot_name:=thorvald_002 robot_pose_x:=2.0 robot_pose_y:=4.0 robot_pose_Y:=1.7"

#tmux select-window -t $SESSION:6
#tmux send-keys "DISPLAY=:$DISPLAY roslaunch topological_navigation topological_navigation.launch map:=riseholme move_base_reconf_service:=TebLocalPlannerROS"

#tmux select-window -t $SESSION:7
#tmux send-keys "DISPLAY=:$DISPLAY  rosrun rviz rviz -d $(rospack find rasberry_bringup)/resources/topological_navigation.rviz"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
