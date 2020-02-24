#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/Transform.h>
#include "geometry_msgs/PoseArray.h"

vector<Eigen::Vector3d> path;
vector<Eigen::Vector3d> traj_cmd;
bool is_path_ready = false;
bool is_path_init = false;
Eigen::Vector3d odom_pos;
double odom_yaw;

ros::Publisher cmd_pub;
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

status machine_status = waiting;

void path_process(double t_cur)
{
    int num_way_pts = path.size();


}

void cmdCallback(ros::TimerEvent& e)
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

    pos = path_process(t_cur);

    cmd.position.x = pos(0);
    cmd.position.y = pos(1);
    cmd.position.z = pos(2);

    cmd.yaw = 0;

    cmd_pub.publish(cmd);

    traj_cmd.push_bcak(Eigen::Vector3d(cmd.position.x,
                                       cmd.position.y,
                                       0));
    if(traj_cmd.size()>10000) traj_cmd.erase(traj_cmd.begin(), traj_cmd.begin()+1000);                                   
}

void visCallback(ros::TimerEvent& e)
{

}

void wayPtsCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    int num_way_pts = msg->pose.size();
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

    odom_yaw = tf:getYaw(msg->transform.rotation);
    ROS_INFO("Position: (%f, %f); Yaw: %f", odom_pos(0), odom_pos(1), odom_yaw);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rt_planner_node");
    ros::NodeHandle nh;

    cmd_pub     = nh.advertise<quadrotor_msgs::PositionCommand>("rt_cmd", 1);
    way_pts_sub = nh.subscribe("local_way_points", 10, wayPtsCallback);
    odom_sub    = nh.subscribe("/vicon/m100/m100", 10, odomCallback);

    ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);
    ros::Timer vis_timer = nh.createTimer(ros::Duration(0.25), visCallback);

    return 0;
}