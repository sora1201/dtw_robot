#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

class RealPosePublisher
{
private:
    ros::NodeHandle _nh;
    ros::Publisher _real_pose_pub;
    ros::Subscriber _joint_states_sub;

    geometry_msgs::Pose _real_pose;
    sensor_msgs::JointState _joint_states;

    double _yaw;

    tf2_ros::TransformBroadcaster _dynamic_broadcaster;

    std::vector<double> getWheelVel();

    void staticTFBrloadcaster();

    void dynamicTFBroadcaster();

public:
    RealPosePublisher();

    void publishRealPose();

    void jointStatesCallback(const sensor_msgs::JointState& msg);
};

RealPosePublisher::RealPosePublisher() :
_nh(),
_real_pose_pub(),
_joint_states_sub(),
_real_pose(),
_joint_states(),
_yaw(0)
{
    _real_pose_pub = _nh.advertise<geometry_msgs::Pose>("real_pose", 10);
    _joint_states_sub = _nh.subscribe("joint_states", 10, &RealPosePublisher::jointStatesCallback, this);
}

// void RealPosePublisher::staticTFBrloadcaster()
// {
//     geometry_msgs::TransformStamped static_transformStamped;
//     static_transformStamped.header.stamp = ros::Time::now();
//     static_transformStamped.header.frame_id = "base_link";
//     static_transformStamped.child_frame_id = "wheel";
// }

// void RealPosePublisher::dynamicTFBroadcaster()
// {
//     geometry_msgs::TransformStamped transformStamped;
//     transformStamped.header.stamp = ros::Time::now();
//     transformStamped.header.frame_id = "odom";
//     transformStamped.child_frame_id = "base_link";
//     transformStamped.transform.translation.x = _real_pose.position.x;
//     transformStamped.transform.translation.y = _real_pose.position.y;
//     transformStamped.transform.translation.z = 0;
//     tf2::Quaternion q;
//     q.setRPY(0, 0, _yaw);
//     transformStamped.transform.rotation.x = q.x();
//     transformStamped.transform.rotation.y = q.y();
//     transformStamped.transform.rotation.z = q.z();
//     transformStamped.transform.rotation.w = q.w();
//     _dynamic_broadcaster.sendTransform(transformStamped);
// }

std::vector<double> RealPosePublisher::getWheelVel()
{
    static std::vector<double> pre_joint_states_positions(2, 0);
    static double pre_time = 0;
    double now_time = ros::Time::now().toSec();

    std::vector<double> wheel_vel(2, 0);

    for (auto i = 0; i < 2; i++)
        wheel_vel[i] = (_joint_states.position[i]-pre_joint_states_positions[i])/(now_time-pre_time);
    
    ROS_INFO("%.2lf,%.2lf,%.2lf,%.2lf", _joint_states.position[0], pre_joint_states_positions[0], now_time, pre_time);
    
    pre_time = now_time;
    pre_joint_states_positions = _joint_states.position;

    return wheel_vel;
}

void RealPosePublisher::publishRealPose()
{
    static double pre_time = 0;
    double now_time = ros::Time::now().toSec();

    double speeds[2] = {
        getWheelVel().at(0),
        getWheelVel().at(1)
    };

    double real_speed   = (speeds[0]+speeds[1])/2;
    double real_w_speed = (speeds[0]-speeds[1]/0.2);

    double delta_yaw = real_w_speed*(now_time-pre_time);
    double delta_x = cos(delta_yaw)*real_speed*(now_time-pre_time);
    double delta_y = sin(delta_yaw)*real_speed*(now_time-pre_time);

    _yaw += delta_yaw;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, _yaw);
    
    _real_pose.position.x += delta_x;
    _real_pose.position.y += delta_y;
    _real_pose.position.z = 0;
    _real_pose.orientation.x = quat.x();
    _real_pose.orientation.y = quat.y();
    _real_pose.orientation.z = quat.z();
    _real_pose.orientation.w = quat.w();

    pre_time = now_time;

    ROS_INFO("(vel,w):(%.2lf,%.2lf), (x,y,yaw):(%.2lf,%.2lf,%.2lf)",
        real_speed, real_w_speed, _real_pose.position.x, _real_pose.position.y, _yaw);

    _real_pose_pub.publish(_real_pose);
}

void RealPosePublisher::jointStatesCallback(const sensor_msgs::JointState& msg)
{
    _joint_states = msg;
    
    publishRealPose();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "real_pose_publisher");

    RealPosePublisher real_pose_publisher;

    ros::Rate rate(50.0);

    while (ros::ok())
    {
        ros::spin();

        // real_pose_publisher.publishRealPose();

        rate.sleep();    
    }
    
    return 0;
}
