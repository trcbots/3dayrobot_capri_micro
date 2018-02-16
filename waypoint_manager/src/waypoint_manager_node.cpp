#include <ros/ros.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <queue>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <math.h>
#include "waypoint_manager/constants.h"
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class waypoints
{

  public:
    waypoints(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~waypoints();
    void
    sendGoals();

  private:
    geometry_msgs::Twist getControl(const nav_msgs::Odometry::ConstPtr& msg, const geometry_msgs::PoseStamped & cur_goal);
    void waypointCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void readWaypoints(std::string file, std::queue<geometry_msgs::PoseStamped> &waypoints);
    bool reachedWaypoint(const nav_msgs::Odometry::ConstPtr& msg, const geometry_msgs::PoseStamped & cur_goal);
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    std::queue<geometry_msgs::PoseStamped> waypointsQueue;
    std::string waypoints_file_name_;
    bool has_waypoints_file;
    ros::Subscriber state_sub;
    ros::Publisher ctl_pub;
    geometry_msgs::PoseStamped cur_goal;
    double thresh_dis, max_steer, max_vel, steer_p, vel_p; // Currently set to 0.2 meter error
};

waypoints::waypoints(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh), nh_private_(nh_private), thresh_dis(REACHED_GOAL),
  max_steer(MAX_STEER), max_vel(MAX_VEL), steer_p(STEER_PROP_GAIN), vel_p(VEL_PROP_GAIN)
{
  if (nh_private.getParam("waypoints_file", waypoints_file_name_))
    {
      //msg.data = "I am reading waypoints from my launch files";
      //voice_pub.publish(msg);
      has_waypoints_file = true;
      // Read the waypoints from file
      readWaypoints(waypoints_file_name_, waypointsQueue);

      // This publisher publishes the curretn goal
      cur_goal = waypointsQueue.front();
      waypointsQueue.push(cur_goal);
      waypointsQueue.pop();
      std::cout << "Initialize current goal as " << cur_goal.pose.position.x
                << " " << cur_goal.pose.position.y << " "
                <<cur_goal.pose.orientation.z << std::endl;

      // This publisher publishes the control command for the simulation
      ctl_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
      // This subscriber reads the waypoint information
      state_sub = nh.subscribe("odometry/filtered", 10, &waypoints::waypointCallback,this);

    }
  else
    {
      //msg.data = "I did not find waypoints in my launch files";
      //voice_pub.publish(msg);
      has_waypoints_file = false;
      ROS_WARN("No waypoints file name given as parameter");
    }
}

waypoints::~waypoints()
{
  // NADA
}
void waypoints::waypointCallback(const nav_msgs::Odometry::ConstPtr& msg){
  if (reachedWaypoint(msg,cur_goal)){
      // Update current goal again
      cur_goal = waypointsQueue.front();
      waypointsQueue.push(cur_goal);
      waypointsQueue.pop();
      std::cout << "Goal reached, new goal at " << cur_goal.pose.position.x
                << " " << cur_goal.pose.position.y << " "
                <<cur_goal.pose.orientation.z << std::endl;

  }

  geometry_msgs::Twist ctl_cmd = getControl(msg,cur_goal);
  ctl_pub.publish(ctl_cmd);

}

geometry_msgs::Twist waypoints::getControl(const nav_msgs::Odometry::ConstPtr& msg, const geometry_msgs::PoseStamped & cur_goal){
  // This is the function that computes the control input for the car
  // Extensive debugging information at the moment
  // It is published everytime the odometry information is received
  // First calculate the desired heading
  // TODO Evantually doubled computation
  geometry_msgs::Twist ctl_input;
  double delta_x = cur_goal.pose.position.x-msg->pose.pose.position.x;
  double delta_y = cur_goal.pose.position.y-msg->pose.pose.position.y;
  double des_steer = (-atan2(delta_y,delta_x)+tf::getYaw(cur_goal.pose.orientation))*steer_p;
  if (des_steer > 0.7) ROS_ERROR("The desired steering angle is way too high!");
  std::cout << "x difference: " << delta_x << "y difference" << delta_y << std::endl;
  std::cout << "cur orientation: " << tf::getYaw(msg->pose.pose.orientation) << "desired steering" << des_steer << std::endl;
  if (des_steer>max_steer) des_steer = max_steer;
  if (des_steer<-max_steer) des_steer = -max_steer;
  ctl_input.angular.z = des_steer;
  double des_speed = sqrt(delta_x*delta_x + delta_y*delta_y)*vel_p;
  std::cout << "desired speed " << des_speed <<std::endl;


  if (des_speed>max_vel) des_speed = max_vel;
  ctl_input.linear.x = des_speed;

  ROS_INFO("Computed control input: %f, %f ", des_steer, des_speed );
  return ctl_input;

}

bool waypoints::reachedWaypoint(const nav_msgs::Odometry::ConstPtr& msg, const geometry_msgs::PoseStamped & cur_goal){
  if (msg->header.frame_id != cur_goal.header.frame_id) {
    ROS_ERROR("The frame of goal and current odometry do not match %s and %s",
            msg->header.frame_id.c_str(), cur_goal.header.frame_id.c_str() );
  }
  double delta_x = msg->pose.pose.position.x-cur_goal.pose.position.x;
  double delta_y = msg->pose.pose.position.y-cur_goal.pose.position.y;
  double delta_theta = tf::getYaw(msg->pose.pose.orientation)-
                    tf::getYaw(cur_goal.pose.orientation);
  return (delta_x*delta_x + delta_y * delta_y) < thresh_dis ;
}

void waypoints::readWaypoints(std::string file, std::queue<geometry_msgs::PoseStamped> &wpoints)
{
  geometry_msgs::PoseStamped cur_waypoint;
  std::ifstream fh(file.c_str());
    if(fh.fail()){
        //File does not exist code here
        ROS_ERROR("Requested file doesn't exist! Using zero values instead");
        wpoints.push(cur_waypoint);
    }
  char comma;

  int num;
  fh >> num;
  while (num--)
    {
      double x, y,theta;
      std::string zone;
      fh >> std::setprecision(18) >> x; //reads in the double value
      fh.get(comma);
      fh >> std::setprecision(18) >> y; //reads in the double value
      fh.get(comma);
      fh >> std::setprecision(18) >> theta; //reads in the double value

      cur_waypoint.header.frame_id = "map";
      cur_waypoint.header.stamp = ros::Time::now();
      cur_waypoint.pose.position.x = x;
      cur_waypoint.pose.position.y = y;
      cur_waypoint.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
      std::cout << "Loaded user defined pose x: " << x << " y: " << y << " theta: " << theta << std::endl;
      wpoints.push(cur_waypoint);
    }
  fh.close();

  ROS_INFO("Finished reading file %s", file.c_str());
}


int
main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_manager");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  waypoints wp(nh, nh_private);
  ros::Rate loop_rate(50);
  ros::spin();

  return 0;
}
