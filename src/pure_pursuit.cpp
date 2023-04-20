#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#define CONV2(x) (x>M_PI ? x-2*M_PI : x)

class PurePursuit
{
private:
    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;

    ros::Subscriber _real_pose_sub;
    ros::Publisher _cmd_vel_pub;
    ros::Publisher _marker_pub;
    std::string _csv_file_path;

    tf2_ros::Buffer _buffer;
    tf2_ros::TransformListener _listenr;

    std::string _target_frame_id, _source_frame_id;
    
    std::vector<std::vector<double>> _waypoints;
    geometry_msgs::Pose _real_pose;

    int _start_index; //0~
    int _pause_index; //0~

    std::vector<int> _pause_index_list;

    int _ahead;
    double _max_vel;
    double _min_vel;
    double _max_w_vel;
    double _accel_range;
    double _decel_range;
    double _goal_range;
    double _goal_yaw_range;
    std::string _show_waypoints_frame;

    enum AXIS
    {
        X,
        Y
    };

    void setWayPoints();

    double getDistance(std::vector<double> point0, std::vector<double> point1);

    int getTargetWayPoint();

    double setVelocity();
    double setWVelocity(double x_diff, double y_diff, double yaw, double x_vel);

    bool isGoal();

    double getYaw(geometry_msgs::Quaternion quat);

    void setPauseIndex(int index);

    void setAhead(int ahead);
    
public:
    PurePursuit();

    bool follow();

    void showWayPoints();

    void start();

    void updatePauseIndex();

    void realPoseCallback(const geometry_msgs::Pose& msg);
};

PurePursuit::PurePursuit() : 
_nh(),
_pnh("~"),
_real_pose_sub(),
_cmd_vel_pub(),
_marker_pub(),
_csv_file_path(),
_listenr(_buffer),
_waypoints{0},
_real_pose(),
_start_index(0),
_pause_index(0),
_ahead(0),
_max_vel(0),
_min_vel(0),
_max_w_vel(0),
_accel_range(0),
_decel_range(0),
_goal_range(0),
_goal_yaw_range(0),
_show_waypoints_frame()
{
    _pnh.getParam("csv_file_path", _csv_file_path);
    ROS_INFO("%s", _csv_file_path.c_str());

    _pnh.getParam("ahead", _ahead);
    _pnh.getParam("max_vel", _max_vel);
    _pnh.getParam("min_vel", _min_vel);
    _pnh.getParam("max_w_vel", _max_w_vel);
    _pnh.getParam("accel_range", _accel_range);
    _pnh.getParam("decel_range", _decel_range);
    _pnh.getParam("goal_range", _goal_range);
    _pnh.getParam("goal_yaw_range", _goal_yaw_range);
    _pnh.getParam("show_waypoints_frame", _show_waypoints_frame);
    _pnh.getParam("target_frame_id", _target_frame_id);
    _pnh.getParam("source_frame_id", _source_frame_id);

    _real_pose_sub = _nh.subscribe("real_pose", 10, &PurePursuit::realPoseCallback, this);
    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    _marker_pub = _nh.advertise<visualization_msgs::Marker>("marker", 10);

    setWayPoints();
}

void PurePursuit::setWayPoints()
{
    static bool first = false;
    std::ifstream ifs_csv_file(_csv_file_path);

    std::string str_buf;
    while (std::getline(ifs_csv_file, str_buf))
    {
        std::istringstream i_stream(str_buf);

        std::string str_conma_buf;
        
        if (first == true) 
        {
            std::vector<int> temp_data;
            while (std::getline(i_stream, str_conma_buf, ','))
            {
                temp_data.push_back(std::stod(str_conma_buf));
            }
            _pause_index_list = temp_data;
            for (auto i = 0; i < _pause_index_list.size(); i++)
                ROS_INFO("pindex:%d", _pause_index_list[i]);
            setPauseIndex(_pause_index_list[0]);
            first = true;
        }
        else
        {
            std::vector<double> temp_data;
            while (std::getline(i_stream, str_conma_buf, ','))
            {
                temp_data.push_back(std::stod(str_conma_buf));
            }

            _waypoints.push_back(temp_data);
        }

    }

    for (int i = 0; i < _waypoints.size(); i++)
        ROS_INFO("[%d]:(%lf,%lf)", i, _waypoints[i].at(AXIS::X), _waypoints[i].at(AXIS::Y));
}

double PurePursuit::getDistance(std::vector<double> point0, std::vector<double> point1)
{
    double x = fabs(point1.at(AXIS::X) - point0.at(AXIS::X));
    double y = fabs(point1.at(AXIS::Y) - point0.at(AXIS::Y));
    return sqrt(x*x + y*y);
}

int PurePursuit::getTargetWayPoint()
{
    double distance[_waypoints.size()] = {0};
    static int target_index = 0;

    std::vector<double> temp_real_pose(2, 0);
    temp_real_pose.at(AXIS::X) = _real_pose.position.x;
    temp_real_pose.at(AXIS::Y) = _real_pose.position.y;

    for (auto i = 0; i < _pause_index; i++)
    {
        distance[i] = getDistance(_waypoints[i], temp_real_pose);
        if (distance[target_index] >= distance[i])
        {
            target_index = i;
        }
    }
    
    if (target_index >= _pause_index)
        return target_index;
    
    return target_index+_ahead;
}

bool PurePursuit::isGoal()
{
    std::vector<double> temp_real_pose(2, 0);
    temp_real_pose.at(AXIS::X) = _real_pose.position.x;
    temp_real_pose.at(AXIS::Y) = _real_pose.position.y;

    if (getDistance(_waypoints[_pause_index], temp_real_pose) < _goal_range)
    {
        return true;
    }

    return false;
}

double PurePursuit::setVelocity()
{
    double velocity = 0;

    std::vector<double> temp_real_pose(2, 0);
    temp_real_pose.at(AXIS::X) = _real_pose.position.x;
    temp_real_pose.at(AXIS::Y) = _real_pose.position.y;

    if (getDistance(_waypoints[_pause_index], temp_real_pose) < _decel_range)
    {
        velocity = getDistance(_waypoints[_pause_index], temp_real_pose) * _max_vel/_decel_range;

        if (velocity < _min_vel)        velocity = _min_vel;
        else if (velocity > _max_vel)   velocity = _max_vel;
    }
    else if (getDistance(_waypoints[_start_index], temp_real_pose) < _accel_range)
    {
        velocity = getDistance(_waypoints[_start_index], temp_real_pose) * _max_vel/_accel_range;

        if (velocity < _min_vel)        velocity = _min_vel;
        else if (velocity > _max_vel)   velocity = _max_vel;
    }
    else
    {
        velocity = _max_vel;
    }

    return velocity;
}

double PurePursuit::setWVelocity(double x_diff, double y_diff, double yaw, double x_vel)
{
    double a = atan2(y_diff, x_diff) - yaw; //方位誤差
    double L = sqrt(x_diff*x_diff+y_diff*y_diff); //目標点と現在地の距離

    double w_vel = 2*x_vel*sin(a)/L;
    if (w_vel > _max_w_vel) w_vel = _max_w_vel;

    return w_vel;
}

bool PurePursuit::follow()
{
    geometry_msgs::Twist cmd_vel;

    geometry_msgs::TransformStamped transform_stamped;
    
    try {
        transform_stamped = _buffer.lookupTransform(_target_frame_id, _source_frame_id, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    _real_pose.position.x = transform_stamped.transform.translation.x;
    _real_pose.position.y = transform_stamped.transform.translation.y;
    _real_pose.position.z = transform_stamped.transform.translation.z;
    _real_pose.orientation.x = transform_stamped.transform.rotation.x;
    _real_pose.orientation.y = transform_stamped.transform.rotation.y;
    _real_pose.orientation.z = transform_stamped.transform.rotation.z;
    _real_pose.orientation.w = transform_stamped.transform.rotation.w;
    
    int target = getTargetWayPoint();
    double x_diff = _waypoints[target].at(AXIS::X) - _real_pose.position.x;
    double y_diff = _waypoints[target].at(AXIS::Y) - _real_pose.position.y;
    
    if (!isGoal())
    {
        cmd_vel.linear.x = setVelocity();
        cmd_vel.angular.z = setWVelocity(x_diff, y_diff, getYaw(_real_pose.orientation), cmd_vel.linear.x);
        ROS_INFO("v:%lf, w:%lf", cmd_vel.linear.x, cmd_vel.angular.z);
        _cmd_vel_pub.publish(cmd_vel);
    
        return false;
    }
    
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    _cmd_vel_pub.publish(cmd_vel);

    ROS_INFO("GOAL!!!");

    if (target == _waypoints.size()-1)
    {
        while (1)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            _cmd_vel_pub.publish(cmd_vel);
            ROS_INFO("finish!!!");

            ros::shutdown();
        }
    }

    return true;
}

void PurePursuit::showWayPoints()
{
    for (auto i = 0; i < _waypoints.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = _show_waypoints_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoint";
        marker.lifetime = ros::Duration();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        marker.id = i;
        marker.pose.position.x = _waypoints[i].at(0);
        marker.pose.position.y = _waypoints[i].at(1);
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        _marker_pub.publish(marker);
    }
}

double PurePursuit::getYaw(geometry_msgs::Quaternion quat)
{
    double roll, pitch, yaw;

    tf2::getEulerYPR(quat, yaw, pitch, roll);

    return yaw;
}

void PurePursuit::setPauseIndex(int index)
{
    _pause_index = index;
}

void PurePursuit::start()
{
    _start_index = _pause_index;
    _pause_index = _waypoints.size()-1;
}

void PurePursuit::updatePauseIndex()
{
    static int i = 0;
    i++;

    if (i >= _pause_index_list.size()) 
    {
        _pause_index = _waypoints.size()-1;
    }
    else
    {
        _pause_index = _pause_index_list[i];
    }
}

void PurePursuit::setAhead(int ahead)
{
    _ahead = ahead;
}

void PurePursuit::realPoseCallback(const geometry_msgs::Pose& msg)
{
    _real_pose = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit");

    PurePursuit pure_pursuit;
    pure_pursuit.updatePauseIndex();

    ros::Rate rate(50.0);
    while (ros::ok())
    {
        ros::spinOnce();
    
        if (pure_pursuit.follow())
        {
            pure_pursuit.start();
            pure_pursuit.updatePauseIndex();
        }

        pure_pursuit.showWayPoints();

        rate.sleep();
    }

    return 0;
}