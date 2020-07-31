/*  Author: Daniel Lynch
 *  File:   gz_robot_position_logger/src/gz_robot_position_logger.cpp
 *  Task:   Program to log robot's position, etc during gazebo simulation
 *
 */
#include "gz_robot_position_logger/gz_robot_position_logger.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "gz_robot_position_logger_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    GzRobotPositionLogger loguacious(&nh);
    loguacious.run();

    ros::spinOnce();

    return 0;

}
