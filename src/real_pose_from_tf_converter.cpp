#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "real_pose_from_tf_converter");
    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    ros::Publisher real_pose_pub = nh.advertise<geometry_msgs::Pose>("real_pose", 10);

    tf::TransformListener listener;

    std::string target_frame_id, source_frame_id;

    pnh.getParam("target_frame_id", target_frame_id);
    pnh.getParam("source_frame_id", source_frame_id);

    ros::Rate rate(50.0);
    while (ros::ok())
    {
        ros::spinOnce();
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform transform;
        try {
            listener.lookupTransform(target_frame_id, source_frame_id, ros::Time(0), transform);
        }
        catch(tf2::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        geometry_msgs::Pose real_pose;

        real_pose.position.x = transform.getOrigin().x();
        real_pose.position.y = transform.getOrigin().y();
        real_pose.position.z = transform.getOrigin().z();
        real_pose.orientation.x = transform.getRotation().x();
        real_pose.orientation.y = transform.getRotation().y();
        real_pose.orientation.z = transform.getRotation().z();
        real_pose.orientation.w = transform.getRotation().w();

        real_pose_pub.publish(real_pose);

        rate.sleep();
    }

    return 0;
}