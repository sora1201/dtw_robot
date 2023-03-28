#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <math.h>

class Converter
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    tf::TransformBroadcaster br_;

public:
    Converter()
    {
        joy_sub_ = nh_.subscribe("/tracker", 10, &Converter::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry& msg)
    {
        tf::Transform transform;
        tf::poseMsgToTF(msg.pose.pose, transform);
        br_.sendTransform(tf::StampedTransform(transform, msg.header.stamp, "odom", msg.child_frame_id));
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_tf_converter");
    Converter converter;
    ros::spin();
    return 0;
}