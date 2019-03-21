#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "pid_test/SetSpeedTime.h"

ros::Publisher twist_pub;
geometry_msgs::Twist target_twist;
ros::Time start_time;
float delay = 0.0;

bool move(pid_test::SetSpeedTime::Request  &req,
          pid_test::SetSpeedTime::Response &res)
{
    target_twist.linear.x = req.velocity;
    delay = (float)req.time;
    
    start_time = ros::Time::now();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_test_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("set_speed_time", move);
    
    twist_pub = n.advertise<geometry_msgs::Twist>("demand_vel", 1);
    
    ROS_INFO("Ready to move");
  
    ros::Rate r(20); // 20Hz
      
    while(ros::ok())
    {
        if((ros::Time::now() - start_time).toSec() > delay)
        {
            target_twist.linear.x = 0.0;
        }
        
        twist_pub.publish(target_twist);
        
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}
