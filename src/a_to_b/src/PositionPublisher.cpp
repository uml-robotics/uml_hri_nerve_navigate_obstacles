/* Author: Peter Gavriel */
/* Source: https://github.com/uml-robotics/uml_nist_mobility_tools/blob/master/src/positionpublisher_v2.cpp */

#include <ros/ros.h>
#include <time.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <ctime>
using namespace std;

string getTime();
string df(double);
double calcspeed(double,double,double,double&,double&);
void findTriggers(double&,double&,double&,double&,double,double,geometry_msgs::Pose);

string logName;
stringstream logstring;
ofstream logfile;

bool newgoal = false;
geometry_msgs::Pose goal;
//This method is called when the goal topic is updated (aka recieves a new goal)
void goalwatcher(const geometry_msgs::PoseStamped current_goal){
    newgoal = true;
    goal = current_goal.pose;
}

//This method appends the log file using an output stream and can output to terminal as well
void appendlog(ostream& ls,bool print){
    try{
      stringstream ss;
      ss << ls.rdbuf();
      string output = ss.str();
      if(print) cout << ss.str();
      logfile << ss.rdbuf();
      ls.flush();
    }catch(...){
      ROS_WARN("Error writing to log.");
    }
  return;
}

/*
PROGRAM OVERVIEW
From a top level view, this node tries to do the following:
1. Subscribe to move_base/current_goal so it knows when a new goal is sent
2. Open a log file to write out to
3. Obtain and track the position of the robot within the map.
4. Initialize variables, and find triggers to start position logging
5. After triggers are tripped, log positional data and do speed calculations every
    time the robot moves more than the tracking RESOLUTION in either the x or y
    direction.

What are those trigger things?
For testing purposes, we really only want to start tracking the robots position
once it passes some kind of starting line, and these triggers attempt to find the
starting lines on all four sides using the robots starting position. Combining that
with the transform tracking the position of the robots rear, we can know when the
robot passes one of those lines.

Why are the logs so ugly?
Rude, but good question. The outputs are specifically formatted to be turned into
spreadsheets, so try changing the log.txt file to log.csv, and import it into a google
sheet. If you don't care about that, try using positionpublisher V1, the output
should be more readable, but it won't have as much info because many features were
added in V2.
*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "position_publisher");

  ros::NodeHandle n;

  ros::Publisher position_pub = n.advertise<geometry_msgs::Pose>("/map_position", 1000);

  ros::Subscriber goal_sub = n.subscribe("/move_base/current_goal",1000,goalwatcher);

  ros::Rate loop_rate(5);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  geometry_msgs::Pose robotpose,rearpose;

  //There are a ton of working variables here, I know. But they all do something I promise.
  bool init = true, x_trigger = false, y_trigger = false, new_goal_distance_tracker = false,wait_for_trigger = false;
  char linesymbol = '-';
  double current_x,current_y,last_x,last_y;
  string current_time, last_time="";
  double resolution = 0.5; //Incremental Meters to track position
  string robot, test;
  double avgspeed;
  int counter;
  double x_grid_offset = 0.5, y_grid_offset = 0.5;
  double pos_x_trig,neg_x_trig,pos_y_trig,neg_y_trig;
  string consoleout;
  double totaldist, totaltime;
  int goal_count = 0;
  double start_time;
  double goal_start_time, goal_elapsed_time;
  double goal_start_x, goal_start_y,goal_route_distance;
  double route_distance,route_time,first_trigger_time;

  n.getParam(ros::this_node::getName()+"/robot",robot);
  n.getParam(ros::this_node::getName()+"/test",test);
  n.getParam(ros::this_node::getName()+"/resolution",resolution);
  n.getParam(ros::this_node::getName()+"/logfile",logName);

  try{
  logfile.open(logName, ios_base::app);
  cout << "Log opened ("<<logName<<")\n";
  }catch (...){
    ROS_WARN("Exception opening log file. :(");
  }

  //Retrieve current date and time
  time_t *rawtime = new time_t;
  struct tm * timeinfo;
  time(rawtime);
  timeinfo = localtime(rawtime);

  //Log Testing Info
  logstring <<(1900+timeinfo->tm_year)<<"/"<<(1+timeinfo->tm_mon)<<"/"<<timeinfo->tm_mday
          <<"\t"<<timeinfo->tm_hour<<":"<<(1+timeinfo->tm_min)<<":"<<(1+timeinfo->tm_sec)<<endl
          <<"Robot: \t"<<robot<<endl
          <<"Test:  \t"<<test<<endl
          <<"Resolution: \t"<<resolution<<endl<<endl;
  cout << "\nPostion Tracker Initialized:\n";
  cout << logstring.str();
  appendlog(logstring,false);
  while (ros::ok())
  {
    try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/map", "/base_link", now, ros::Duration(3.0));
      listener.lookupTransform("/map", "/base_link", now, transform);
      robotpose.position.x = transform.getOrigin().getX();
      robotpose.position.y = transform.getOrigin().getY();
      robotpose.position.z = transform.getOrigin().getZ();
      robotpose.orientation.x = transform.getRotation().getX();
      robotpose.orientation.y = transform.getRotation().getY();
      robotpose.orientation.z = transform.getRotation().getZ();
      robotpose.orientation.w = transform.getRotation().getW();
      listener.waitForTransform("/map", "/robot_rear", now, ros::Duration(3.0));
      listener.lookupTransform("/map", "/robot_rear", now, transform);
      rearpose.position.x = transform.getOrigin().getX();
      rearpose.position.y = transform.getOrigin().getY();
      rearpose.position.z = transform.getOrigin().getZ();
      rearpose.orientation.x = transform.getRotation().getX();
      rearpose.orientation.y = transform.getRotation().getY();
      rearpose.orientation.z = transform.getRotation().getZ();
      rearpose.orientation.w = transform.getRotation().getW();

      //INITIALIZE: set pose, start time, find triggers, log format
      if(init){
        current_x = robotpose.position.x;
        current_y = robotpose.position.y;
        start_time = stod(getTime());
        current_time = getTime();
        totaldist = 0;
        totaltime = 0;
        goal_route_distance = 0;
        findTriggers(pos_x_trig,neg_x_trig,pos_y_trig,neg_y_trig,x_grid_offset,y_grid_offset,robotpose);
        avgspeed  = 0;
        counter = 0;
        logstring<<"Message Type,Global Time,Elapsed Time,X Position,Y Position,dT,dX,dY,Calculated Speed,Goal Time,Goal Distance,Avg Goal Speed,Route Time,Route Distance,Avg Route Speed\n";
        appendlog(logstring,false);
        logstring<<"!-I\t,"<<current_time<<","<<(stod(current_time)-start_time)<<"\t,"<<df(current_x)<<","<<df(current_y)<<endl<<endl;
        appendlog(logstring,true);

        init = false;
      }
    }catch(tf::TransformException &ex) {
      //ROS_INFO("ERROR: %s",ex.what());
    }

    //Publish Robots Pose to Topic
    position_pub.publish(robotpose);

    //NEW GOAL RECIEVED
    if(newgoal){
      if(goal_count!=0){
        goal_elapsed_time = stod(current_time)-goal_start_time;
        route_time = stod(current_time)-first_trigger_time;
        logstring<<"G-F\t,"<<current_time<<","<<(stod(current_time)-start_time)<<"\t,"<<robotpose.position.x<<","<<robotpose.position.y<<",,,,,\t\t\t\t"
                 <<goal_elapsed_time<<","<<goal_route_distance<<","<<(goal_route_distance/goal_elapsed_time)<<","
                 <<route_time<<","<<route_distance<<","<<(route_distance/route_time)<<endl;
        appendlog(logstring,true);
      }
      new_goal_distance_tracker = true;
      wait_for_trigger = true;
      goal_start_time = stod(current_time);
      goal_start_x = robotpose.position.x;
      goal_start_y = robotpose.position.y;
      goal_count++;
      logstring <<"\nG-"<<goal_count<<"\t,"<<current_time<<","<<(stod(current_time)-start_time)<<"\t,"<<goal.position.x<<","<<goal.position.y<<endl;
      logstring<<"G-@\t,"<<current_time<<","<<(stod(current_time)-start_time)<<"\t,"<<df(goal_start_x)<<","<<df(goal_start_y)<<endl;
      appendlog(logstring,true);
      newgoal = false;
    }

    //Set x and y tracking triggers true when the robot crosses its gridline.
    if((rearpose.position.x>=pos_x_trig||rearpose.position.x<=neg_x_trig)&&!x_trigger&&!init) {
      x_trigger = true;
      logstring <<"!-XTRIG\t,"<<getTime()<<","<<(stod(getTime())-start_time)<<","<<robotpose.position.x<<","<<robotpose.position.y<<"\n";
      appendlog(logstring,true);
      }
    if((rearpose.position.y>=pos_y_trig||rearpose.position.y<=neg_y_trig)&&!y_trigger&&!init) {
      y_trigger = true;
      logstring <<"!-YTRIG\t,"<<getTime()<<","<<(stod(getTime())-start_time)<<","<<robotpose.position.x<<","<<robotpose.position.y<<"\n";
      appendlog(logstring,true);
    }

    //Log Writing for position tracking
    if((abs(robotpose.position.x-current_x)>=resolution&&x_trigger)||(abs(robotpose.position.y-current_y)>=resolution)&&y_trigger){
      if(abs(robotpose.position.x-current_x)>=resolution){linesymbol = 'x';}
      if(abs(robotpose.position.y-current_y)>=resolution){linesymbol = 'y';}
      if(abs(robotpose.position.x-current_x)>=resolution&&abs(robotpose.position.y-current_y)>=resolution){
        linesymbol = '+';
      }
      current_time = getTime();
      current_x = robotpose.position.x;
      current_y = robotpose.position.y;

      //Calculate approximate goal and route distances using the hypotenuses between changes in x and y.
      double dx,dy;
      if(new_goal_distance_tracker){
        goal_route_distance = 0;
        dx = current_x-goal_start_x;
        dy = current_y-goal_start_y;
        dx = abs(dx*dx); //dx^2
        dy = abs(dy*dy); //dy^2
        goal_route_distance += sqrt(dx+dy);
        new_goal_distance_tracker = false;
      }else{
        dx = current_x-last_x;
        dy = current_y-last_y;
        dx = abs(dx*dx); //dx^2
        dy = abs(dy*dy); //dy^2
        goal_route_distance += sqrt(dx+dy);
      }

      if(wait_for_trigger){
        route_distance = 0;
        first_trigger_time = stod(getTime());
        wait_for_trigger = false;
      }else
      route_distance += sqrt(dx+dy);

      //TRIGGER LINE OUTPUT
      if(linesymbol=='+') logstring << "x+y\t,"; //Message Type
      else logstring <<linesymbol<<"\t,";
      logstring <<current_time<<","<<(stod(current_time)-start_time)<<"\t,"; //Time, Relative Time
      logstring <<df(robotpose.position.x)<<","<<df(robotpose.position.y)<<"\t,"; //X, Y
      if(last_time!=""){
        double time_diff = stod(current_time)-stod(last_time);
        double x_diff = current_x - last_x;
        double y_diff = current_y - last_y;
        logstring <<df(time_diff)<<",";// dT
        logstring <<df(x_diff)<<","<<df(y_diff)<<"\t,"; //dX, dY
        double speed = calcspeed(x_diff,y_diff,time_diff,totaldist,totaltime);
        avgspeed += speed;
        counter++;
        logstring.precision(3);
        logstring <<speed; //Speed (m/s)
      }else{
        logstring << " ---------------------------";
      }

      logstring <<endl;
      appendlog(logstring,true);
      last_time = current_time;
      last_x = current_x;
      last_y = current_y;
    }
    ros::spinOnce();

    loop_rate.sleep();
  }
  //Special case for last goal recieved (Where no new goal is called)
  if(goal_count!=0){
    goal_elapsed_time = stod(current_time)-goal_start_time;
    route_time = stod(current_time)-first_trigger_time;
    logstring<<"G-F\t,"<<current_time<<","<<(stod(current_time)-start_time)<<"\t,"<<robotpose.position.x<<","<<robotpose.position.y<<",,,,,\t\t\t\t"
    <<goal_elapsed_time<<","<<goal_route_distance<<","<<(goal_route_distance/goal_elapsed_time)<<","
    <<route_time<<","<<route_distance<<","<<(route_distance/route_time)<<endl;
    appendlog(logstring,true);
  }
  logstring<<"\n!-F\t,"<<current_time<<","<<(stod(current_time)-start_time)<<"\t,"<<df(current_x)<<","<<df(current_y)<<endl;
  logstring << "\n=================================================================================\n";
  appendlog(logstring,true);
  return 0;
}

double calcspeed(double dx, double dy, double t,double& totaldist, double& totaltime){
  double x = abs(dx*dx);
  double y = abs(dy*dy);
  double dist = sqrt(x+y);
  totaldist += dist;
  totaltime += t;
  return (dist/t);
}
string df(double d){//Double format
  stringstream ss;
  ss.setf(ios::fixed);
  ss.precision(2);
  ss << d;
  string s = ss.str();
  return s;
}
string getTime(){
  double now = ros::Time::now().toSec();
  stringstream ss;
  ss.setf(ios::fixed);
  ss.precision(2);
  ss << now;
  string t = ss.str();
  return t;
}
void findTriggers(double& pos_x, double& neg_x, double& pos_y, double& neg_y, double x_off, double y_off,geometry_msgs::Pose robotpose){
  cout << endl;
  ROS_INFO("[Pose sent for triggers: X:%.2f  Y:%.2f]",robotpose.position.x,robotpose.position.y);
  pos_x = round(robotpose.position.x)+x_off;
  neg_x = round(robotpose.position.x)-x_off;
  pos_y = round(robotpose.position.y)+y_off;
  neg_y = round(robotpose.position.y)-y_off;

  ROS_INFO("[Triggers Found: X+:%.2f, X-:%.2f  ||  Y+:%.2f, Y-:%.2f]\n",pos_x,neg_x,pos_y,neg_y);

  return;
}

