#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

float linear_vel;
float angular_vel;
bool stop_robot = false;
ros::Publisher cmd_pub_;


bool callbackStop(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  stop_robot = true;
}
bool callbackStart(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  stop_robot = false;
}

void vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  linear_vel = msg->linear.x ;
  angular_vel = msg->angular.z;
  
}



int main(int argc, char** argv){
  ros::init(argc, argv, "publish_cmd_vel");

  ros::NodeHandle n("~");
  
  ros::ServiceServer serverStop = n.advertiseService("stop", callbackStop); 
  ros::ServiceServer serverStart = n.advertiseService("start", callbackStart); 

  ros::Subscriber vel_sub_ = n.subscribe("/teleop_cmd_vel", 10, vel_callback);
  geometry_msgs::Twist cmd_vel;

  
  cmd_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
      
  
  
  
  ros::Rate r(100);
  
    while(ros::ok())
    {
        
            
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;

        if(stop_robot)
        {
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = 0;
        }
        cmd_pub_.publish(cmd_vel);
        
       
    	ros::spinOnce();
        r.sleep();
    }

    return 0;
}


