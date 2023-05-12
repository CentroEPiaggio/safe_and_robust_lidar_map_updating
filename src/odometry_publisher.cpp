#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

double x,y,z;
geometry_msgs::Quaternion odom_quat;
ros::Time current_time;
bool wait =false;
void odomCallback(const nav_msgs::OdometryConstPtr& msg){
 
  odom_quat = msg->pose.pose.orientation;
  x = msg->pose.pose.position.x;  
  y = msg->pose.pose.position.y;  
  z = msg->pose.pose.position.z;  
 current_time = msg->header.stamp;
  wait = true;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/robot/robotnik_base_control/odom", 10, &odomCallback);

  tf::TransformBroadcaster odom_broadcaster;
  x = 0.0;
  y = 0.0;
  z = 0.0;
  odom_quat.x = 0.0;
  odom_quat.y = 0.0;
  odom_quat.z = 0.0;
  odom_quat.w = 0;
  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
//   last_time = ros::Time::now();
  ros::Rate r(100.0);
  while(n.ok()){
    ros::spinOnce();               // check for incoming messages
    if(wait){
    current_time = ros::Time::now();

    
    
  geometry_msgs::TransformStamped odom_trans;
   
  odom_trans.header.frame_id = "robot_odom";
  odom_trans.child_frame_id = "robot_base_footprint";
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = z;
  odom_trans.transform.rotation = odom_quat;
  odom_trans.header.stamp = current_time;

    //send the transform
  odom_broadcaster.sendTransform(odom_trans);
    
      last_time = current_time;
    }
    r.sleep();
  }
}
