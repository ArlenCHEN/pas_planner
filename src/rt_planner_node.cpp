#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <tf/transform_listener.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/Transform.h>
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/Marker.h"

using namespace std;

vector<Eigen::Vector3d> path;
vector<Eigen::Vector3d> traj_cmd;
bool is_path_ready = false;
bool is_path_init = false;
Eigen::Vector3d odom_pos;
double odom_yaw;
double time_temp_start;

ros::Publisher cmd_pub;
ros::Publisher traj_pub;
ros::Subscriber way_pts_sub;
ros::Subscriber odom_sub;

ros::Time time_traj_start;

quadrotor_msgs::PositionCommand cmd;

enum status 
{
    waiting,
    execution
};

int traj_id = 0;
int i = 0;

status machine_status = waiting;

void path_process(double t_cur, Eigen::Vector3d pos)
{
    int num_way_pts = path.size();

    if(num_way_pts == 1)
    {
        pos(0) = path[0](0);
        pos(1) = path[0](1);
        pos(2) = 0; 
        return;
    }

    double n = 4.0;
    double t;

    if(t_cur == 0)
    {   
        time_temp_start = t_cur;    
        traj_id = 0;
        i = 0;
    }

    if(traj_id < num_way_pts - 1)
    {
        // Limit the range of t to [0, n]
        t = t_cur - time_temp_start;
        if(t > n) t = n;
        if(t < 0) t = 0;

        pos(0) = path[traj_id](0) + (path[traj_id+1](0) - path[traj_id](0)) * t / n;
        pos(1) = path[traj_id](1) + (path[traj_id+1](1) - path[traj_id](1)) * t / n;
        pos(2) = 0; 

        if(traj_id == i && pos(0) == path[traj_id+1](0) && pos(1) == path[traj_id+1](1))
        {
            time_temp_start = t_cur; 
            i = i+1;
            traj_id = i;
        }
    }
    else
    {
        pos(0) = path[traj_id](0);
        pos(1) = path[traj_id](1);
        pos(2) = 0; 

        machine_status = waiting;
    }
}

void cmdCallback(const ros::TimerEvent& e)
{
    if(!is_path_init)return;

    Eigen::Vector3d pos;

    if(machine_status == waiting)
    {
        time_traj_start = ros::Time::now();
        if(is_path_ready)
        {
            machine_status = execution;
        }
    }

    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - time_traj_start).toSec();
    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";

    path_process(t_cur, pos);

    cmd.position.x = pos(0);
    cmd.position.y = pos(1);
    cmd.position.z = pos(2);

    cmd.yaw = 0;

    cmd_pub.publish(cmd);

    traj_cmd.push_back(Eigen::Vector3d(cmd.position.x,
                                       cmd.position.y,
                                       0));
    if(traj_cmd.size()>10000) traj_cmd.erase(traj_cmd.begin(), traj_cmd.begin()+1000);                                   
}

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id)
{
    visualization_msgs::Marker mk;
    mk.header.frame_id = "odom";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.id = id;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(path.size()); i++) {
        pt.x = path[i](0);
        pt.y = path[i](1);
        pt.z = path[i](2);
        mk.points.push_back(pt);
    }
    traj_pub.publish(mk);
}

void visCallback(const ros::TimerEvent& e)
{
    displayTrajWithColor(traj_cmd, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
}

void wayPtsCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    int num_way_pts = msg->poses.size();
    if(num_way_pts == 1)
    {
        is_path_ready = false;
        return;
    }
    else
    {
        is_path_init = true;
        is_path_ready = true;

        for(int i = 0; i < num_way_pts; ++i)
        {
            path[i](0) = msg->poses[i].position.x;
            path[i](1) = msg->poses[i].position.y;
            path[i](2) = msg->poses[i].position.z;
        }
    }
}

void odomCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
    odom_pos(0) = msg->transform.translation.x;
    odom_pos(1) = msg->transform.translation.y;
    odom_pos(2) = msg->transform.translation.z;

    odom_yaw = tf::getYaw(msg->transform.rotation);
    ROS_INFO("Position: (%f, %f); Yaw: %f", odom_pos(0), odom_pos(1), odom_yaw);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rt_planner_node");
    ros::NodeHandle nh;

    cmd_pub     = nh.advertise<quadrotor_msgs::PositionCommand>("rt_cmd", 1);
    traj_pub    = nh.advertise<visualization_msgs::Marker>("rt_traj", 1);
    way_pts_sub = nh.subscribe("local_way_points", 10, wayPtsCallback);
    odom_sub    = nh.subscribe("/vicon/m100/m100", 10, odomCallback);

    ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);
    ros::Timer vis_timer = nh.createTimer(ros::Duration(0.25), visCallback);

    return 0;
}