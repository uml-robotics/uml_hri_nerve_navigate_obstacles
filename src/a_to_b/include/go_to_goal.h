/*  Author: Daniel Lynch
 *  File:   a_to_b/include/go_to_goal.h
 *  Task:   GoToGoal class defines a client so we can send goal requests 
 *          to move_base server via SimpleActionClient
 *  Ref:    http://library.isr.ist.utl.pt/docs/roswiki/actionlib_tutorials(2f)Tutorials(2f)Writing(20)a(20)Callback(20)Based(20)Simple(20)Action(20)Client.html
 *  Usage:  
 *          GoToGoal goal(&nh, "nerve1", 10) //nodehandle, map name, number of reps
 */
#ifndef GO_TO_GOAL_H_
#define GO_TO_GOAL_H_

#include <unordered_map>
#include <utility>

#include <boost/bind.hpp> // needed to pass in the "this" pointer in ac_.sendGoal(...)

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>

//TODO
// check costmap-clearing service function; 
// add counters for successes/failures; output summary at program completion

//auxiliary function for simple geometry_msgs::Point initializing
geometry_msgs::Point pointInit(double x, double y, double z=0.0) {
    geometry_msgs::Point pp;
    pp.x = x;
    pp.y = y;
    pp.z = z;
    return pp;
}

struct NerveMapGoals {
    /* Dictionary storing points A and B for each Nerve map.
     * TODO
     *  - double-check A,B for nerve2 and nerve3
     *  - add entries for new maps
     *
     * Ref: 
     * Nerve maps 1-3: https://github.com/uml-robotics/uml_3d_race/tree/master/resources/maps
     */
    using mapTable = std::unordered_map<std::string,std::pair<geometry_msgs::Point,geometry_msgs::Point>>;
    mapTable maps_;

    NerveMapGoals() {
        maps_ = {
        {"nerve1", std::make_pair( pointInit(0.0, 4.0), pointInit(0.0, 9.0) )},
        {"nerve2", std::make_pair( pointInit(0.0, 6.0), pointInit(0.0, 11.0) )},
        {"nerve3", std::make_pair( pointInit(0.0, 8.0), pointInit(0.0, 13.0) )}
      };
    }

    std::pair<geometry_msgs::Point,geometry_msgs::Point> operator()(std::string name) {
        return maps_.at(name); 
    }
};

class GoToGoal
{
    using moveBaseAction = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

  public:
    GoToGoal(ros::NodeHandle* nodehandle, std::string mapname, int n);
    void run();
    
    
    
    static geometry_msgs::Point pointInit(double x, double y, double z);

  private:  
    ros::NodeHandle nh_;
    ros::ServiceClient clearCostmap_;
    move_base_msgs::MoveBaseGoal goal_;
    std_srvs::Empty srv_;
    int reps_;
    geometry_msgs::Point A_,B_; // first and second goals

    //mapTable maps_;   
    moveBaseAction ac_;
    
    void sendGoalCallback(const actionlib::SimpleClientGoalState& state);
    void visit(const geometry_msgs::Point& p);
    void visit(double, double, double, double, double, double, double);
    void sendGoal();
    void initServices();
    void clearCostmap();
};

GoToGoal::GoToGoal(ros::NodeHandle* nodehandle, std::string mapname, int n)
     : nh_(*nodehandle), ac_("move_base", true), reps_(n)
{
    ROS_INFO("Initializing GoToGoal constructor");

    //get points A and B for specified map
    NerveMapGoals getPoints;
    std::tie(A_,B_) = getPoints(mapname);

    initServices();

    // wait for action server to come up
    while (!ac_.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting on the move_base action server to come up");
    }
    ROS_INFO("move_base action server is up");


}

void GoToGoal::initServices() {
    ROS_INFO("Initializing clear_costmap service");
    clearCostmap_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmap");
}

void GoToGoal::clearCostmap() {
    //clearcostmap then sleep for half a second
    clearCostmap_.call(srv_);
    ros::Duration(0.5).sleep();
}

void GoToGoal::visit(double x, double y, double z, double o_x=0.0, double o_y=0.0, double o_z=0.0, double o_w=1.0) {
    /* given x,y,z,ox,oy,oz,ow, attempt action move_base */

    // frame parameters
    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.header.stamp = ros::Time::now();

    // goal_ pose
    goal_.target_pose.pose.position.x = x;
    goal_.target_pose.pose.position.y = y;
    goal_.target_pose.pose.position.z = z;
    goal_.target_pose.pose.orientation.x = o_x;
    goal_.target_pose.pose.orientation.y = o_y;
    goal_.target_pose.pose.orientation.z = o_z;
    goal_.target_pose.pose.orientation.w = o_w;

    sendGoal();
 //    ROS_INFO("Sending goal...");
    // // when sending a goal, need to register the callback to use on movebase status update
    // // (see https://answers.ros.org/question/259418/sending-goals-to-navigation-stack-using-code/)
    // ac_.sendGoal(goal_, boost::bind(&GoToGoal::sendGoalCallback, this, _1), moveBaseAction::SimpleActiveCallback());
    // ac_.waitForResult();
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
    goal_.target_pose.pose.orientation.x = 0.0;
    goal_.target_pose.pose.orientation.y = 0.0;
    goal_.target_pose.pose.orientation.z = 0.0;
    goal_.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Received a geometry_msgs::Point, and now Sending goal...");
    sendGoal();
    
    // ac_.sendGoal(goal_, boost::bind(&GoToGoal::sendGoalCallback, this, _1), moveBaseAction::SimpleActiveCallback());
    // ac_.waitForResult();     
}

void GoToGoal::sendGoalCallback(const actionlib::SimpleClientGoalState& state) {    
    ROS_INFO("Hello from sendGoalCallback. Finished in state [%s]", state.toString().c_str());
    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("I did it! What do I win?");

            clearCostmap();
            
    } else {
            ROS_INFO("Whoops! Can I try again?");
            
            clearCostmap();

    }
}

void GoToGoal::sendGoal() {
    ROS_INFO("Sending goal...");
    // when sending a goal, need to register the callback to use on movebase status update
    // (see https://answers.ros.org/question/259418/sending-goals-to-navigation-stack-using-code/)
    ac_.sendGoal(goal_, boost::bind(&GoToGoal::sendGoalCallback, this, _1), moveBaseAction::SimpleActiveCallback());
    ac_.waitForResult();
}

void GoToGoal::run() {
    //move to starting (A) position
    visit(A_);
    ROS_INFO("Here I go!");
    for (int i=0; i<reps_; i++) {
        visit(B_);
        visit(A_);
    }
}



#endif //GO_TO_GOAL_H_
