/*	Author: Daniel Lynch
 *	File:	a_to_b/src/a_to_b.cpp
 *	Task:	Simple program to route Fetch to 1 of 4 corners randomly.
 *
 */
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>

#include "go_to_goal.h"

//auxiliary function for simple geometry_msgs::Point initializing
geometry_msgs::Point pointInit(double x, double y, double z) {
	geometry_msgs::Point pp;
	pp.x = x;
	pp.y = y;
	pp.z = z;
	return pp;
}

int main(int argc, char* argv[]) {

	ros::init(argc, argv, "wander_node");
	ros::NodeHandle nh;
	std_srvs::Empty srv;	
	ros::ServiceClient clearCostmap = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmap");
	ros::Rate loop_rate(50);

	geometry_msgs::Point pA, pB;
	GoToGoal goal;
	pA = pointInit(0.0, 4.0, 0.0);
	pB = pointInit(0.0, 9.0, 0.0);

	while (ros::ok()){

		//visit point
		goal.visit( pA );

		//clearcostmap then sleep for half a second
		clearCostmap.call(srv);
		ros::Duration(0.5).sleep();
		
	}
	
	ros::spinOnce();

	return 0;

}
