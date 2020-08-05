/*	Author: Daniel Lynch
 *	File:	a_to_b/include/go_to_goal.h
 *	Task:	GoToGoal class header/definition
 *	Ref:	http://library.isr.ist.utl.pt/docs/roswiki/actionlib_tutorials(2f)Tutorials(2f)Writing(20)a(20)Callback(20)Based(20)Simple(20)Action(20)Client.html
 *	Usage:	
 * 			GoToGoal goal;
 *			geometry_msgs::Point somePoint{x1,y1,z1}
 * 			goal.visit( somePoint );
 */
#ifndef GO_TO_GOAL_H_
#define GO_TO_GOAL_H_

#include <boost/bind.hpp> // needed to pass in the "this" pointer in ac.sendGoal(...)

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>

/* define a client so we can send goal requests to move_base server via SimpleActionClient */
class GoToGoal
{
  public:
	GoToGoal();
	void visit(double, double, double, double, double, double);
	void visit(const geometry_msgs::Point& p);
	
	static geometry_msgs::Point pointInit(double x, double y, double z);

  private:	
	ros::NodeHandle n;
	std_srvs::Empty srv;
	ros::ServiceClient clearCostmap = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmap");
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> FetchMoveBaseClient;
	FetchMoveBaseClient ac;
	move_base_msgs::MoveBaseGoal goal_;
	void sendGoalCallback(const actionlib::SimpleClientGoalState& state);
};

GoToGoal::GoToGoal() : ac("move_base", true) {
	// wait for action server to come up
	while (!ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting on the move_base action server to come up");
	}
	ROS_INFO("move_base action server is up");
}

void GoToGoal::visit(double x, double y, double o_x=0.0, double o_y=0.0, double o_z=0.0, double o_w=1.0) {
	/* go_to_goal: given x,y coordinates, attempt action move_base */

	// frame parameters
	goal_.target_pose.header.frame_id = "map";
	goal_.target_pose.header.stamp = ros::Time::now();

	// goal_ pose
	goal_.target_pose.pose.position.x = x;
	goal_.target_pose.pose.position.y = y;
	goal_.target_pose.pose.position.z = 0.0;
	goal_.target_pose.pose.orientation.x = 0.0;
	goal_.target_pose.pose.orientation.y = 0.0;
	goal_.target_pose.pose.orientation.z = 0.0;
	goal_.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal...");
	// when sending a goal, need to register the callback to use on movebase status update
	// (see https://answers.ros.org/question/259418/sending-goals-to-navigation-stack-using-code/)
	ac.sendGoal(goal_, boost::bind(&GoToGoal::sendGoalCallback, this, _1), FetchMoveBaseClient::SimpleActiveCallback());
	ac.waitForResult();

	// status is processed in our callback
}

void GoToGoal::visit(const geometry_msgs::Point& p) {
	// frame/header parameters
	goal_.target_pose.header.frame_id = "map";
	goal_.target_pose.header.stamp = ros::Time::now();
	
	// goal_ pose
	goal_.target_pose.pose.position.x = p.x;
	goal_.target_pose.pose.position.y = p.y;
	goal_.target_pose.pose.position.z = p.z;

	ROS_INFO("Sending goal...");
	
	ac.sendGoal(goal_, boost::bind(&GoToGoal::sendGoalCallback, this, _1), FetchMoveBaseClient::SimpleActiveCallback());
	ac.waitForResult();		
}

void GoToGoal::sendGoalCallback(const actionlib::SimpleClientGoalState& state) {	
	ROS_INFO("Hello from sendGoalCallback. Finished in state [%s]", state.toString().c_str());
	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("I did it!");

			//clearcostmap then sleep for half a second
			clearCostmap.call(srv);
			ros::Duration(0.5).sleep();
	        
	} else {
			ROS_INFO("Whoops! Can I try again?");
			// go back to start, try again
	}
}

//auxiliary function for simple geometry_msgs::Point initializing
geometry_msgs::Point GoToGoal::pointInit(double x, double y, double z) {
	geometry_msgs::Point pp;
	pp.x = x;
	pp.y = y;
	pp.z = z;
	return pp;
}


#endif //GO_TO_GOAL_H_
