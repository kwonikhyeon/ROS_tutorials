#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("LaserScan (val)=(%f %f %f %f %f)", msg->ranges[0], msg->ranges[90], msg->ranges[180], msg->ranges[270], msg->ranges[360]);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/scan", 1000, counterCallback);
    ros::spin();
    return 0;
}