/*	Author: Daniel Lynch
 *	File:	a_to_b/src/a_to_b.cpp
 *	Task:	Program to move robot from point A to point B, n times.
 *
 */
#include "go_to_goal.h"

int main(int argc, char* argv[]) {

	ros::init(argc, argv, "a_to_b_node");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(10);

	geometry_msgs::Point pA, pB;
	pA = GoToGoal::pointInit(0.0, 4.0, 1.0);
	pB = GoToGoal::pointInit(0.0, 9.0, 0.0);

	GoToGoal goal(&nh, "nerve1", 10);

	while (ros::ok()){

		//visit point
		goal.visit( pA );
		//goal.visit( pB );
		
	}	
	ros::spinOnce();

	return 0;

}
