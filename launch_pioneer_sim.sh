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
echo "*"
echo "**"
echo "***"
echo "Launching Gazebo world w/ a Pioneer..."
echo "***"
echo "**"
echo "*"
roslaunch uml_3d_race level2.launch &
pid=$!
sleep 5s

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
echo "Launching octomap_server..."
cd src/uml_3d_race/launch/navigation
roslaunch octomap_server.launch &
pid=$!
sleep 2s


trap "echo Killing all processes.; kill -2 $pid; exit" SIGINT SIGTERM

sleep 24h