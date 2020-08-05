/*  File: gz_robot_position_logger.h
 *  Date: 2020-07-26
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

    sources:
    https://wiki.ros.org/message_filters#Time_Synchronizer
    https://wiki.ros.org/message_filters#ApproximateTime_Policy
    https://wiki.ros.org/message_filters/ApproximateTime
    https://answers.ros.org/question/235778/synchronization-of-multiple-messages/
    https://answers.ros.org/question/284758/roscpp-message_filters-approximatetime-api-questions/
    https://answers.ros.org/question/83316/trying-to-use-timesynchronizergetting-compile-error/
    https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
    https://gist.github.com/tdenewiler/e2172f628e49ab633ef2786207793336
 */
#ifndef GZ_ROBOT_POSITION_LOGGER_H
#define GZ_ROBOT_POSITION_LOGGER_H

#include <cmath>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Time.h>

#include <gz_robot_position_logger/GzPosLog.h> // gz_robot_position_logger/msg/GzPosLog.msg

class GzRobotPositionLogger 
{
 public:
  GzRobotPositionLogger(ros::NodeHandle* nodehandle);

 private:
  void initSubs();
  void initPubs();
  void pos_goal_sub_cb(const nav_msgs::Odometry::ConstPtr& odom, const geometry_msgs::PoseStamped::ConstPtr& pose);
  float dist(float r_x, float r_y, float g_x, float g_y);

  ros::NodeHandle nh_;
  ros::Subscriber pos_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher log_pub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PoseStamped> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
};


GzRobotPositionLogger::GzRobotPositionLogger(ros::NodeHandle* nodehandle) {
  ROS_INFO("GzRobotPositionLogger constructor liftoff");
  //initSubs();
  odom_sub_.subscribe(nh_, "/base_pose_ground_truth", 1);
  pose_sub_.subscribe(nh_, "/rapid_goal", 1);
  sync_.reset(new Sync(MySyncPolicy(10), odom_sub_, pose_sub_));
  sync_->registerCallback(boost::bind(&GzRobotPositionLogger::pos_goal_sub_cb, this, _1, _2));

  ROS_INFO("POS and GOAL subscribers initialized");
  initPubs();
}

void GzRobotPositionLogger::initSubs() {
  
  // odom_sub_.subscribe(nh_, "base_pose_ground_truth", 1);
  // pose_sub_.subscribe(nh_, "pioneer_move_base/current_goal", 1);
  // sync_.reset(new Sync(MySyncPolicy(10), odom_sub_, pose_sub_));
  // sync_->registerCallback(boost::bind(&GzRobotPositionLogger::pos_goal_sub_cb, this, _1, _2));

  // ROS_INFO("POS and GOAL subscribers initialized");
}

void GzRobotPositionLogger::initPubs() {
  
  log_pub_ = nh_.advertise<gz_robot_position_logger::GzPosLog>("loggerz", 1000);

  ROS_INFO("LOG pub initialized");

}

void GzRobotPositionLogger::pos_goal_sub_cb(const nav_msgs::Odometry::ConstPtr& odom, const geometry_msgs::PoseStamped::ConstPtr& posest) {
  //create new poslog msg
  gz_robot_position_logger::GzPosLog poslog;

  // get data from odom
  float x = odom->pose.pose.position.x;
  float y = odom->pose.pose.position.y;

  // get data from posest
  float g_x = posest->pose.position.x;
  float g_y = posest->pose.position.y;

  // calc distance to goal
  float d = dist(x,y,g_x,g_y);

  // time
  ros::Time t = ros::Time::now();

  ROS_INFO("PUBLISHING NEW MSG!");

  // populate poslog msg then publish
  // msg info:
  // float64 gz_robot_x
  // float64 gz_robot_y 
  // bool at_goal_a
  // bool at_goal_b
  // float64 dist_from_goal_a
  // float64 dist_from_goal_b
  // time data
  poslog.gz_robot_x = x;
  poslog.gz_robot_y = y;
  poslog.at_goal_a = false;
  poslog.at_goal_b = false;
  poslog.dist_from_goal_a = d;
  poslog.dist_from_goal_b = d;
  poslog.data = t;

  log_pub_.publish(poslog);

}

float GzRobotPositionLogger::dist(float r_x, float r_y, float g_x, float g_y) {
  float dx = r_x - g_x;
  float dy = r_y - g_y;
  return sqrt(dx*dx + dy*dy);
}

#endif //GZ_ROBOT_POSITION_LOGGER_H 
