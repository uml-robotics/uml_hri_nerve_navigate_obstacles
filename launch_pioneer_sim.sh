#!/bin/bash

# launch_pioneer_sim.sh
# =====================
# Launch a world in Gazebo
# Spawn a pioneer3dx robot
# Optionally load or build an octomap
#
# original source (for pioneers):
# https://github.com/JenJenChung/pioneer_test/blob/master/run-pioneer-robot

my_pid=$$
echo "My process ID is $my_pid"

# Launch a world in Gazebo w/ Pioneer3dx robot model
# World options: nerve_mobility1, nerve_mobility2, nerve_mobility3
echo "Launching Gazebo world w/ a Pioneer..."
roslaunch uml_3d_race gazebo.launch world:=nerve_mobility3 &
pid=$!
sleep 5s

roslaunch uml_3d_race spawn_robot.launch model_name:=pioneer x:=0.0 y:=0.0 yaw:=0.0 &
pid=$!
sleep 2s


echo "Launching Pioneer's nav stack..."
roslaunch uml_3d_race navigation.launch &
pid=$!
sleep 2s

echo "Launching rviz..."
roslaunch uml_3d_race rviz.launch &
#rosrun rviz rviz -d fetch_rviz_config.rviz &
pid="$! $pid"
sleep 3s

# Optionally load or build an octomap
# echo "Launching octomap_server..."
# cd src/uml_3d_race/launch/navigation
# roslaunch octomap_server.launch &
# pid=$!
# sleep 2s

# echo "Launching a_to_b_node..."
# rosrun a_to_b a_to_b_node &
# pid="$! $pid"
# sleep 3s


trap "echo Killing all processes.; kill -2 $pid; exit" SIGINT SIGTERM

sleep 24h