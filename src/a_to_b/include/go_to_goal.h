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

class GoToGoal
{
  public:
	GoToGoal();// : ac("move_base", true);
	void visit(double, double, double, double, double, double);
	void visit(const geometry_msgs::Point& p);
	void sendGoalCallback(const actionlib::SimpleClientGoalState& state);
  private:	
	ros::NodeHandle n;
	// define a client so we can send goal requests
	// to move_base server via SimpleActionClient
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> FetchMoveBaseClient;
	FetchMoveBaseClient ac;
	move_base_msgs::MoveBaseGoal goal_;
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

	// frame/header parameters
	goal_.target_pose.header.frame_id = "map";
	goal_.target_pose.header.stamp = ros::Time::now();

	/* alternative: move 1 meter forward */
	//goal_.target_pose.header.frame_id = "base_link"; //or "base_footprint"
	//goal_.target_pose.header.stamp = ros::Time::now();

	//goal_.target_pose.pose.position.x =  1.0;
	//goal_.target_pose.pose.position.y =  0;

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
	} else {
			ROS_INFO("Whoops! I guess I don't know what I'm doing...");
	}
}

#endif //GO_TO_GOAL_H_
