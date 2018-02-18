#include <ros/ros.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <queue>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <math.h>
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class waypoints
{

  public:
    waypoints(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~waypoints();
    void
    sendGoals();

  private:
    geometry_msgs::Twist getControl(const nav_msgs::Odometry::ConstPtr& msg, geometry_msgs::PoseStamped & cur_goal);
    void waypointCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void readWaypoints(std::string file, std::vector<geometry_msgs::PoseStamped> &waypoints);
    bool reachedWaypoint(const nav_msgs::Odometry::ConstPtr& msg, const geometry_msgs::PoseStamped & cur_goal);
    void gpscallback(const sensor_msgs::NavSatFix::ConstPtr & msg);
    geometry_msgs::PoseStamped findClosestGoal(const std::vector<geometry_msgs::PoseStamped> &waypoints , const geometry_msgs::PoseStamped& last_gps);
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    std::vector<geometry_msgs::PoseStamped> waypointVector;
    std::string waypoints_file_name_;
    bool has_waypoints_file, is_yaw_aligned;
    ros::Subscriber state_sub, gps_sub;
    ros::Publisher ctl_pub;
    geometry_msgs::PoseStamped cur_goal, last_gps_;
    std::queue<geometry_msgs::PoseStamped> last_gps_5ago_;
    int initialize_counter_, running_counter_, publish_skip_;
    double calib_utm_n_init, calib_utm_e_init;
    double thresh_dis, max_steer, max_vel, steer_p, steer_i, steer_d,vel_p; // Currently set to 0.2 meter error
    double error_steer, error_steer_acc, last_time,offcourse_dist,vel_ref,init_yaw_;

};

waypoints::waypoints(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh), nh_private_(nh_private)
{
  nh_private.getParam("steer_p", steer_p);
  nh_private.getParam("steer_i", steer_i);
  nh_private.getParam("steer_d", steer_d);
  nh_private.getParam("vel_p", vel_p);
  nh_private.getParam("max_steer", max_steer);
  nh_private.getParam("max_vel", max_vel);
  nh_private.getParam("vel_ref", vel_ref);
  error_steer = 0; running_counter_ =0;
  error_steer_acc = 0;
  initialize_counter_ = 0;
  last_time = ros::Time::now().toSec();
  calib_utm_n_init = 0; calib_utm_e_init = 0;
  offcourse_dist = 8.0;
  is_yaw_aligned = false;
  //ROS_INFO("Using PID controller parameter for steering: %f , %f, %f", steer_p, steer_i, steer_d);
  ROS_INFO("Proportional gain: %f , refernce speed: %f , max speed: %f ", steer_p, vel_ref, max_vel);

  if (nh_private.getParam("waypoints_file", waypoints_file_name_))
    {
      //msg.data = "I am reading waypoints from my launch files";
      //voice_pub.publish(msg);
      has_waypoints_file = true;
      // Read the waypoints from file
      readWaypoints(waypoints_file_name_, waypointVector);

      // This publisher publishes the current goal
      // cur_goal = waypointVector.front();
      // waypointVector.push(cur_goal);
      // waypointVector.pop();
      // std::cout << "Initialize current goal as " << cur_goal.pose.position.x
      //           << " " << cur_goal.pose.position.y << " "
      //           <<cur_goal.pose.orientation.z << std::endl;

      // This publisher publishes the control command for the simulation
      ctl_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
      // This subscriber reads the waypoint information
      state_sub = nh.subscribe("odometry/filtered", 10, &waypoints::waypointCallback,this);
      gps_sub = nh.subscribe("gps/fix", 1, &waypoints::gpscallback,this);
    }
  else
    {
      //msg.data = "I did not find waypoints in my launch files";
      //voice_pub.publish(msg);
      has_waypoints_file = false;
      ROS_WARN("No waypoints file name given as parameter");
    }
    ROS_WARN("Initialization of the vehicle heading. Vehicle moves forward at minimum speed.");

}

waypoints::~waypoints()
{
  // NADA
}
void waypoints::gpscallback(const sensor_msgs::NavSatFix::ConstPtr & msg){
  double utm_n, utm_e;
  std::string zone;
  gps_common::LLtoUTM(msg->latitude,msg->longitude,utm_n,utm_e,zone);
  last_gps_.pose.position.x = utm_e;
  last_gps_.pose.position.y = utm_n;
  ROS_INFO("Read last GPS location, %f, %f", utm_n, utm_e);
  if (last_gps_5ago_.size()>=5) last_gps_5ago_.pop();
  last_gps_5ago_.push(last_gps_);

  if (! is_yaw_aligned ){

    if (calib_utm_n_init == 0 && ++initialize_counter_>=2) {
      // Initialize location
      ROS_INFO("Loading this location as the initial location");
      calib_utm_n_init = utm_n;
      calib_utm_e_init = utm_e;
      return;
    }
    double diff_e = utm_e-calib_utm_e_init;
    double diff_n = utm_n-calib_utm_n_init;
    double total_dist = diff_e*diff_e + diff_n*diff_n;
    if (++initialize_counter_ >=5 && total_dist>=9 ){
      init_yaw_ = atan2(diff_n,diff_e);
      ROS_INFO("YAW initialization complete, results: %f", init_yaw_);
      is_yaw_aligned = true;
    }

  }
  ++running_counter_;
}

void waypoints::waypointCallback(const nav_msgs::Odometry::ConstPtr& msg){
  // if (reachedWaypoint(msg,cur_goal)){
  //     // Update current goal again
  //     cur_goal = waypointVector.front();
  //     waypointVector.push(cur_goal);
  //     waypointVector.pop();
  //     ROS_WARN("Goal reached, new goal at %f, %f, yaw %f",cur_goal.pose.position.x
  //             ,cur_goal.pose.position.y , cur_goal.pose.orientation.z );
  //     ROS_WARN("Current position x: %f y: %f , Goal position: x: %f y: %f", msg->pose.pose.position.x,
  //           msg->pose.pose.position.y,waypointVector.back().pose.position.x,waypointVector.back().pose.position.y );
  //     error_steer_acc = 0;
  // }
  publish_skip_++;
  if (publish_skip_%15 !=0) return;
  // Skip 15 messages until computation the right one


  geometry_msgs::Twist ctl_cmd;
  if (is_yaw_aligned) {
    cur_goal =   findClosestGoal(waypointVector,last_gps_);
    ctl_cmd = getControl(msg, cur_goal);
}
  else{
    ctl_cmd.linear.x = vel_ref;
  }
  ctl_pub.publish(ctl_cmd);

}

geometry_msgs::Twist waypoints::getControl(const nav_msgs::Odometry::ConstPtr& msg, geometry_msgs::PoseStamped & cur_goal){
  // This is the function that computes the control input for the car
  // Extensive debugging information at the moment
  // It is published everytime the odometry information is received
  // First calculate the desired heading
  // TODO Evantually doubled computation
  geometry_msgs::Twist ctl_input;
  ROS_DEBUG("Current Goal position is %f, %f and last gps location at %f, %f ",
      cur_goal.pose.position.x, cur_goal.pose.position.y,
      last_gps_.pose.position.x,  last_gps_.pose.position.y);
  double delta_x = cur_goal.pose.position.x-last_gps_.pose.position.x;
  double delta_y = cur_goal.pose.position.y-last_gps_.pose.position.y;
  double des_theta = tf::getYaw(cur_goal.pose.orientation);
  // The IMU yaw appears to be in the wrong direction?
  ROS_DEBUG("Yaw angle from the filter: %f",tf::getYaw(msg->pose.pose.orientation ));
  double cur_theta = -tf::getYaw(msg->pose.pose.orientation)+init_yaw_;
  //ROS_INFO("orientation dx: %f, orientation dy: %f", delta_x, delta_y);
  //ROS_INFO("orientation %f  , orientation %f", cos(cur_theta), sin(cur_theta));
  //ROS_INFO("desired theta: %f, current theta including init alignment: %f", des_theta, cur_theta);
  double signErr = delta_x*sin(des_theta)-delta_y*cos(des_theta);
  //ROS_INFO("Cross Correction product: %f", signErr);
  double error = copysign(sqrt(delta_x*delta_x + delta_y * delta_y),signErr) ;
  double orient_error = cur_theta - tf::getYaw(cur_goal.pose.orientation);
  while (orient_error > M_PI) orient_error -= 2*M_PI;
  while (orient_error < -M_PI) orient_error += 2*M_PI;
  ROS_INFO("Signed difference to goal %f", error);
  double des_steer = -(orient_error + atan2(steer_p*error,vel_ref));
  ROS_INFO("From orientation error: %f, from path deviation: %f",orient_error,  atan2(steer_p*error,vel_ref));
  //double error = atan2(delta_y,delta_x)-tf::getYaw(cur_goal.pose.orientation);
  // error_steer_acc+=error;
  // ROS_INFO("steering angle command coming from : %f %f %f", error*steer_p, (error-error_steer)/(ros::Time::now().toSec()-last_time)*steer_d,error_steer_acc*steer_i);
  // double des_steer = error*steer_p+(error-error_steer)/(ros::Time::now().toSec()-last_time)*steer_d + error_steer_acc*steer_i;
  // error_steer = error;
  // last_time = ros::Time::now().toSec();
  // if (des_steer > 0.7) ROS_ERROR("The desired steering angle is way too high!");
  // //std::cout << "x difference: " << delta_x << "y difference" << delta_y << std::endl;
  // //std::cout << "cur orientation: " << tf::getYaw(msg->pose.pose.orientation) << "desired steering" << des_steer << std::endl;
  if (des_steer>max_steer) des_steer = max_steer;
  if (des_steer<-max_steer) des_steer = -max_steer;
  ctl_input.angular.z = des_steer;
  if (error > offcourse_dist){
    ROS_ERROR("The Car seems to be completely off track! Reduce speed to zero");

  }
  else ctl_input.linear.x = vel_ref;
  //std::cout << "desired speed " << des_speed <<std::endl;
  ROS_INFO("steering angle command: %f", des_steer);

  if (des_steer < 0.1 && running_counter_ > 30) {
    running_counter_ = 0;
    double yaw_estimate_error = atan2(last_gps_5ago_.back().pose.position.y-last_gps_5ago_.front().pose.position.y,
            last_gps_5ago_.back().pose.position.x-last_gps_5ago_.front().pose.position.x)-cur_theta;
    ROS_WARN("Yaw realignment sequence, new correction term found:  %f", yaw_estimate_error);
    init_yaw_+=yaw_estimate_error;


  }
  //ROS_INFO("Computed control input: %f, %f ", des_steer, des_speed );
  return ctl_input;

}
geometry_msgs::PoseStamped waypoints::findClosestGoal
(const std::vector<geometry_msgs::PoseStamped> &waypoints , const geometry_msgs::PoseStamped& last_gps)
{
  double min_dist = 99999999.0;
  int closest = 0;
  for (int i = 0; i < waypoints.size(); i++){
    double delta_x = -last_gps.pose.position.x+waypoints[i].pose.position.x;
    double delta_y = -last_gps.pose.position.y+waypoints[i].pose.position.y;
    double cur_dist = (delta_x*delta_x + delta_y*delta_y);
    if (cur_dist<min_dist){
      min_dist = cur_dist;
      closest = i;
    }
  }
  ROS_INFO("Current closest from waypoints: %i", closest);
  return waypoints[closest];
}
bool waypoints::reachedWaypoint(const nav_msgs::Odometry::ConstPtr& msg, const geometry_msgs::PoseStamped & cur_goal){
  if (msg->header.frame_id != cur_goal.header.frame_id) {
    ROS_ERROR("The frame of goal and current odometry do not match %s and %s",
            msg->header.frame_id.c_str(), cur_goal.header.frame_id.c_str() );
  }
  double delta_x = -msg->pose.pose.position.x+cur_goal.pose.position.x;
  double delta_y = -msg->pose.pose.position.y+cur_goal.pose.position.y;
  double cur_theta = tf::getYaw(msg->pose.pose.orientation);

  return delta_x*cos(cur_theta)+delta_y*sin(cur_theta) < 0 ;
}

void waypoints::readWaypoints(std::string file, std::vector<geometry_msgs::PoseStamped> &wpoints)
{
  geometry_msgs::PoseStamped cur_waypoint;
  std::ifstream fh(file.c_str());
    if(fh.fail()){
        //File does not exist code here
        ROS_ERROR("Requested file doesn't exist! Using zero values instead");
        wpoints.push_back(cur_waypoint);
    }

  int num;

  fh >> num;
  int p = num;
  while (num--)
    {
      double x, y,theta;
      std::string zone;
      char comma;
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
      //std::cout << "Loaded user defined pose x: " << x << " y: " << y << " theta: " << theta << std::endl;
      wpoints.push_back(cur_waypoint);
    }
  fh.close();

  ROS_INFO("Finished reading file %s, read %i lines", file.c_str(), p);
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
