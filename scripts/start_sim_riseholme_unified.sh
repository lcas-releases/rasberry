#!/bin/bash

SESSION=$USER
DISPLAY=0
ROBOT_NO='007'

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'mongo_topo_man'
tmux new-window -t $SESSION:2 -n 'simulation'
tmux new-window -t $SESSION:3 -n 'map_server'
tmux new-window -t $SESSION:4 -n 'localisation'
tmux new-window -t $SESSION:5 -n 'move_base'
tmux new-window -t $SESSION:6 -n 'topo_nav'
tmux new-window -t $SESSION:7 -n 'rviz'
tmux new-window -t $SESSION:8 -n 'people'
tmux new-window -t $SESSION:9 -n 'coordination'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "echo 'mongodb'" C-m
tmux send-keys "roslaunch mongodb_store mongodb_store.launch db_path:=$1"
tmux select-pane -t 1
tmux send-keys "echo 'topological map server'" C-m
tmux send-keys "roslaunch rasberry_navigation topological_map_manager_central.launch map:=riseholme_sim"

tmux select-window -t $SESSION:2
tmux send-keys "echo 'robot localisation'" C-m
tmux send-keys "DISPLAY=:$DISPLAY roslaunch rasberry_bringup robot_bringup.launch robot_model:=$(rospack find rasberry_bringup)/config/robot_"$ROBOT_NO".yaml model_extras:=$(rospack find rasberry_bringup)/urdf/robot_"$ROBOT_NO"_sensors.xacro simple_sim:=true world_name:=riseholme with_actors:=false"

tmux select-window -t $SESSION:3
tmux send-keys "echo '2D map server'" C-m
tmux send-keys "roslaunch rasberry_move_base map_server.launch map:=$(rospack find rasberry_navigation)/maps/riseholme_sim.yaml no_go_map:=$(rospack find rasberry_navigation)/maps/riseholme_sim_no_go.yaml use_no_go_map:=true"

tmux select-window -t $SESSION:4
tmux send-keys "echo 'robot localisation'" C-m
tmux send-keys "roslaunch rasberry_move_base amcl.launch"

tmux select-window -t $SESSION:5
tmux send-keys "echo 'move_base actions for the robot'" C-m
tmux send-keys "roslaunch rasberry_move_base move_base_dwa.launch"

tmux select-window -t $SESSION:6
tmux send-keys "echo 'topological navigation actions for the robot'" C-m
tmux send-keys "roslaunch rasberry_navigation topological_navigation_robot.launch move_base_reconf_service:=DWAPlannerROS"

tmux select-window -t $SESSION:7
tmux send-keys "echo 'rviz'" C-m
tmux send-keys "DISPLAY=:$DISPLAY rosrun rviz rviz -d $(rospack find rasberry_bringup)/resources/topological_navigation_riseholme.rviz"

tmux select-window -t $SESSION:8
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "echo 'mimic marvelmind_nav/hedge_rcv_bin or run that node if available'" C-m
tmux send-keys "echo 'rosrun marvelmind_nav hedge_rcv_bin'" C-m
tmux send-keys "rostopic pub -r 10 /hedge_pos_a marvelmind_nav/hedge_pos_a -- '1' '0' '2.0' '6.7' '0.0' '0'"
tmux select-pane -t 1
tmux send-keys "echo 'mimic marvelmind_nav/hedge_rcv_bin or run that node if available'" C-m
tmux send-keys "echo 'rosrun marvelmind_nav hedge_rcv_bin'" C-m
tmux send-keys "rostopic pub -r 10 /hedge_pos_a marvelmind_nav/hedge_pos_a -- '5' '0' '2.0' '-4.2' '0.0' '0'"
tmux select-pane -t 2
tmux send-keys "echo 'marvelmind localiser'" C-m
tmux send-keys "rosrun rasberry_people_perception simple_marvel_localiser.py"

tmux select-window -t $SESSION:9
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "echo 'callarobot websocket client'" C-m
tmux send-keys "WEBSOCKET_URL='wss://lcas.lincoln.ac.uk/car/ws' python $(rospack find rasberry_coordination)/callarobot/ws_client.py"
tmux select-pane -t 1
tmux send-keys "echo 'task scheduler'" C-m
tmux send-keys "rosrun rasberry_coordination simple_task_executor_node.py $(rospack find rasberry_coordination)/config/map_config_riseholme_sim.yaml"

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
