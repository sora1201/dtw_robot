#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>

geometry_msgs::Twist real_vel;
geometry_msgs::Point postion;
geometry_msgs::Twist cmd_vel;
nav_msgs::Odometry odom;

void realVelCallback(const geometry_msgs::Twist& msg)
{
    real_vel = msg;
}

void debugCallback(const std_msgs::Float32MultiArray& msg)
{
    std::string data = "debug:";
    for (int i = 0; i < msg.data.size(); i++)
    {
        data += std::to_string(msg.data[i]) + "\t";
    }
    // ROS_INFO("%s", data.c_str());
}

void positionCallback(const geometry_msgs::Point& msg)
{
    postion.x = msg.x;
    postion.y = msg.y;
    postion.z = msg.z;
}

void odomCallback(const nav_msgs::Odometry& msg)
{
    odom = msg;
}

void broadcast_static_tf(
    std::string frame_id,
    std::string child_frame_id,
    std::array<double, 3> translation,
    double yaw,
    tf2_ros::StaticTransformBroadcaster& static_tf_broadcaster)
{
    geometry_msgs::TransformStamped static_tf;

    static_tf.header.stamp = ros::Time::now();
    static_tf.header.frame_id = frame_id;
    static_tf.child_frame_id = child_frame_id;
    static_tf.transform.translation.x = translation.at(0);
    static_tf.transform.translation.y = translation.at(1);
    static_tf.transform.translation.z = translation.at(2);
    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0.0, yaw);
    static_tf.transform.rotation.x = quat.x();
    static_tf.transform.rotation.y = quat.y();
    static_tf.transform.rotation.z = quat.z();
    static_tf.transform.rotation.w = quat.w();

    static_tf_broadcaster.sendTransform(static_tf);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle nh;

    // ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber real_vel_sub = nh.subscribe("real_vel", 10, realVelCallback);
    ros::Subscriber postion_sub = nh.subscribe("position", 10, positionCallback);
    ros::Subscriber debug_sub = nh.subscribe("debug", 10, debugCallback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    // ros::Subscriber odom_sub = nh.subscribe("/dtw_robot1/diff_drive_controller/odom", 10, odomCallback);

    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped odom_tf;
    nav_msgs::Odometry odom;
    ros::Time last_time;
    double x = 0, y = 0, theta = 0;

    // broadcast_static_tf(
    //     "base_link",
    //     "front_lidar_link",
    //     {0.38, -0.15, 0.0065}, 0,
    //     static_tf_broadcaster);

    // ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 10);
    
    ros::Rate rate(50.0);

    while (ros::ok())
    {
        ros::spinOnce();

        ros::Time current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        double delta_theta = real_vel.angular.z * dt;
        double delta_x = real_vel.linear.x * std::cos(delta_theta) * dt;
        double delta_y = real_vel.linear.x * std::sin(delta_theta) * dt;
        x += delta_x*std::cos(theta) - delta_y*std::sin(theta);
        y += delta_x*std::cos(theta) + delta_y*std::sin(theta);
        theta += delta_theta;
        
        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta);

        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        // odom_tf.transform.translation.x = x;
        // odom_tf.transform.translation.y = y;
        // odom_tf.transform.translation.z = 0;
        // odom_tf.transform.rotation.x = quat.x();
        // odom_tf.transform.rotation.y = quat.y();
        // odom_tf.transform.rotation.z = quat.z();
        // odom_tf.transform.rotation.w = quat.w();
        odom_tf.transform.translation.x = odom.pose.pose.position.x;
        odom_tf.transform.translation.y = odom.pose.pose.position.y;
        odom_tf.transform.translation.z = odom.pose.pose.position.z;
        odom_tf.transform.rotation.x = odom.pose.pose.orientation.x;
        odom_tf.transform.rotation.y = odom.pose.pose.orientation.y;
        odom_tf.transform.rotation.z = odom.pose.pose.orientation.z;
        odom_tf.transform.rotation.w = odom.pose.pose.orientation.w;


        tf_broadcaster.sendTransform(odom_tf);

        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();
        odom.twist.twist = real_vel;

        odom_pub.publish(odom);

        // cmd_pub.publish(cmd_vel);

        // ROS_INFO("odom: %f, %f, %f", x, y, theta);
        // ROS_INFO("real_vel: %f, %f, %f", real_vel.linear.x, real_vel.linear.y, real_vel.angular.z);
        
        last_time = current_time;

        rate.sleep();   
    }

    return 0;
}