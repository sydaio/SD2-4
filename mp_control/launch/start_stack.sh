#!/bin/bash

SESSION="mp"

tmux new-session -d -s $SESSION -n bringup

# Window 1: Control
tmux send-keys -t $SESSION:bringup "ce" C-m
tmux send-keys -t $SESSION:bringup " ros2 launch mp_control start_mp.py" C-m

tmux split-window -h -t $SESSION:bringup
tmux send-keys -t $SESSION:bringup "ros2 launch ldlidar ldlidar.launch.py serial_port:=/dev/ttyAMA0 serial_baudrate:=230400" C-m


# Window 1: Teleop
tmux send-keys -t $SESSION:bringup "ce" C-m
tmux send-keys -t $SESSION:bringup " ros2 run twist_stamper twist_stamper --ros-args -r /cmd_vel_in:=/spacenav/twist -r cmd_vel_out:=/mecanum_drive_controller/reference -p frame_id:=base_link" C-m


# tmux split-window -h -t $SESSION:bringup
# tmux send-keys -t $SESSION:bringup "ros2 launch my_slam cartographer.launch.py" C-m

# Window 2: navigation
tmux new-window -t $SESSION -n navigation
tmux send-keys -t $SESSION:navigation "ce" C-m
tmux send-keys -t $SESSION:navigation "ros2 launch nav2_bringup bringup_launch.py use_localization:=False" C-m

tmux split-window -h -t $SESSION:navigation
tmux send-keys -t $SESSION:navigation "ros2 run cartographer_ros cartographer_node -configuration_directory ~/checkout/mp/colcon_ws/src/navigation/config -configuration_basename cartographer_config.lua" C-m

tmux split-window -h -t $SESSION:navigation
tmux send-keys -t $SESSION:navigation "ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0" C-m

# Window 2: Misc
tmux new-window -t $SESSION -n misc
tmux send-keys -t $SESSION:misc "ce" C-m
tmux send-keys -t $SESSION:misc "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser" C-m

tmux split-window -h -t $SESSION:misc
tmux send-keys -t $SESSION:misc "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint" C-m



# Window 3: debug
#tmux new-window -t $SESSION -n debug
#tmux send-keys -t $SESSION:debug "source /opt/ros/humble/setup.bash" C-m
#tmux send-keys -t $SESSION:debug "source ~/ros2_ws/install/setup.bash" C-m
#tmux send-keys -t $SESSION:debug "rviz2" C-m

# Attach
tmux attach -t $SESSION