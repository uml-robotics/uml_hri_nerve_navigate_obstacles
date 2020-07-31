/*  File: gz_robot_position_logger.h
 *  Date: 2020-06-06
 *

    subscribes to 
      /base_pose_ground_truth (nav_msgs/Odometry)
        retrieves:
          pose: pose: position: 
              * x: -0.0236804571188
              * y: 12.3050744419
      /pioneer_move_base/current_goal (geometry_msgs/PoseStamped)
        retrieves x,y from goal position; measures dist 

    measuring:
          gz_robot_x | gz_robot_y | at_goal_a [0,1] | dist_from_goal_a | at_goal_b | dist_from_goal_b

    write to .csv
    when complete, output logs; + report 
 */
#ifndef GZ_ROBOT_POSITION_LOGGER_H
#define GZ_ROBOT_POSITION_LOGGER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Time.h>

//Custom msg GzPosLog defined in gz_robot_position_logger/msg/GzPosLog.msg
#include <gz_robot_position_logger/GzPosLog.h>

class GzRobotPositionLogger {
public:
  GzRobotPositionLogger(ros::NodeHandle* nodehandle);
  void run();  
private:
  ros::NodeHandle nh_;
  ros::Subscriber pos_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher log_pub_;

  void initSubs();
  void initPubs();

  void pos_sub_cb(const nav_msgs::Odometry& msg);
  void goal_sub_cb(const geometry_msgs::PoseStamped& msg);
  void log_msg_pub();

};

#endif //GZ_ROBOT_POSITION_LOGGER_H 


GzRobotPositionLogger::GzRobotPositionLogger() {
  ROS_INFO("GzRobotPositionLogger constructor liftoff");
}

void GzRobotPositionLogger::initSubs() {
  
  pos_sub_ = nh_.subscribe("/base_pose_ground_truth", 1000, pos_sub_cb);
  goal_sub_ = nh_.subscribe("/pioneer_move_base/current_goal", 1000, goal_sub_cb);
  
  ROS_INFO("POS and GOAL subscribers initialized");
}

void GzRobotPositionLogger::initPubs() {
  
  log_pub_ = nh_.advertise<gz_robot_position_logger::GzPosLog>("loggies", 1000);

}

void GzRobotPositionLogger::pos_sub_cb(const nav_msgs::Odometry& msg) {
  // get data from msg
  float x = msg.pose.pose.position.x;
  float y = msg.pose.pose.position.y;
}

void GzRobotPositionLogger::goal_sub_cb(const geometry_msgs::PoseStamped& msg) {

}

void GzRobotPositionLogger::log_msg_pub() {
  gz_robot_position_logger::GzPosLog poslog;

}

