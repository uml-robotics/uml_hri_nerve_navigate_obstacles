/*  Author: Daniel Lynch
 *  File:   a_to_b/src/a_to_b.cpp
 *  Task:   Program to move robot from point A to point B, n times.
 *
 */
#include "go_to_goal.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "a_to_b_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    GoToGoal goal(&nh, "nerve1", 3);
    goal.run();

    ros::spin();

    return 0;

}
