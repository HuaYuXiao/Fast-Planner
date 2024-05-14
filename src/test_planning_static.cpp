/***************************************************************************************************************************
* test_planning_static.cpp
*
* Author: Tao JIANG
*
* Update Time: 2020.01.20
*
* Introduction:  Planner  
*         1. 读取地图pcd文件，然后通过话题（"/planning/global_point_cloud"） 发布出去。
*         2. 设置初始里程信息，这里只做静态测试，所以认为机器人位置不动（"/planning/odom_world"）。
***************************************************************************************************************************/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "prometheus_msgs/PositionReference.h"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明及定义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::PositionReference traj_now;
ros::Publisher map_pub;
ros::Publisher odom_pub;
ros::Publisher waypoint_pub;

bool is_load_map{false};
sensor_msgs::PointCloud2 cloud_map_msg;
nav_msgs::Path waypoints;
nav_msgs::Odometry init_odom;
nav_msgs::Odometry odom_now;
bool is_run_odom(false);
bool is_static_mode(true);
int odom_mode; // 0: manual; 1: sub

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void trajCallbck(const prometheus_msgs::PositionReference& msg);
void visualizer_one_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string name);
void omdpubCallback(const ros::TimerEvent& e);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数定义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// 【订阅】订阅轨迹，并赋值给odom，实现近似动态
void trajCallbck(const prometheus_msgs::PositionReference& msg){
    traj_now = msg; 
    odom_now.header.stamp = ros::Time::now();;
    odom_now.header.frame_id = "map";
    odom_now.pose.pose.position.x = traj_now.position_ref[0];
    odom_now.pose.pose.position.y = traj_now.position_ref[1];
    odom_now.pose.pose.position.z = traj_now.position_ref[2];

    odom_now.twist.twist.linear.x = traj_now.velocity_ref[0];
    odom_now.twist.twist.linear.y = traj_now.velocity_ref[1];
    odom_now.twist.twist.linear.z = traj_now.velocity_ref[2];
    is_run_odom = true;
}

// pcl 中显示点云地图
void visualizer_one_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string name)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(name));

    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud,0,0,255);
    viewer->addPointCloud(cloud,cloud_color,name);

    viewer->spinOnce(100);
}

// 【发布】处理里程计信息，根据模式选择是否发布
void omdpubCallback(const ros::TimerEvent& e) {

    //we'll publish the odometry message over ROS
    if (!is_run_odom){
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();;
        odom.header.frame_id = "map";

        //set the position and quaternion
        odom.pose.pose.position.x = init_odom.pose.pose.position.x;
        odom.pose.pose.position.y = init_odom.pose.pose.position.y;
        odom.pose.pose.position.z = init_odom.pose.pose.position.z;

        geometry_msgs::Quaternion odom_quat;
        odom_quat.x = 0;
        odom_quat.y = 0;
        odom_quat.z = 0;
        odom_quat.w = 0;
        odom.pose.pose.orientation = odom_quat;

        // 发布odom
        odom_pub.publish(odom);
    }else if (is_run_odom && odom_mode==2)    //如果模式为动态模式，就是odom会随生成轨迹运动
    {
        odom_pub.publish(odom_now);
    }
}

// void waypointpubCallback(const ros::TimerEvent &e){
//     geometry_msgs::PoseStamped pt;
//     pt.header.frame_id = "map";
//     pt.header.stamp = ros::Time::now();
//     pt.pose.position.x = 0.0;
//     pt.pose.position.y = -2.0;
//     pt.pose.position.z = 1.0;

//     // waypoints.poses.clear();
//     // waypoints.poses.push_back(pt);
//     // waypoints.header.frame_id = std::string("map");
//     // waypoints.header.stamp = ros::Time::now();
//     // waypoint_pub.publish(waypoints);
//     waypoint_pub.publish(pt);
//     ROS_INFO("--- test_planning_static: pub wayoint successful!---");
// }

// void waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg)
// {
//   cout << "[waypointCallback]: receive goal!" << endl;
//   ROS_INFO("waypoint: [%f, %f, %f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
// }

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// test: 
// $ roscore
// $ rosrun prometheus_plan_manage test_planning_static _test/file_name:= $(your pcd file)
int main(int argc,char** argv)
{   
    // 1. initialization node
    ros::init(argc,argv,"pub_map_odom");
    ros::NodeHandle nh("~");

    // 2. 发布odom和点云地图
    odom_pub = nh.advertise<nav_msgs::Odometry>("/prometheus/drone_odom", 50);
    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/prometheus/planning/global_pcl",1);
    // waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/planning/waypoint", 50);
    // ros::Subscriber waypoint_sub_ = nh.subscribe("/planning/goal", 1, &waypointCallback);

    // 3. set the trigger frequece for different events. 
    
    ros::Timer pub_odom_timer_;
    ros::Subscriber traj_sub;
    nh.param("test/odom_mode", odom_mode, 0);
    
    if(odom_mode==0){
        // 认为无人机不动，进行静态规划
        pub_odom_timer_ = nh.createTimer(ros::Duration(0.05), &omdpubCallback);
    }else if(odom_mode==1){
       // 由px4发布odom信息 
    }else if(odom_mode==2){
        // 无人机随轨迹运动，订阅轨迹，更新odom
        traj_sub = nh.subscribe("/prometheus/fast_planner/position_cmd", 50, trajCallbck);
        pub_odom_timer_ = nh.createTimer(ros::Duration(0.05), &omdpubCallback);
    }

    // ros::Timer pub_waypoint_timer = nh.createTimer(ros::Duration(2.0), &waypointpubCallback);
    
    // 4. get param from launch file
    nh.param("test/init_odom_x", init_odom.pose.pose.position.x, 0.0);
    nh.param("test/init_odom_y", init_odom.pose.pose.position.y, 0.0);
    nh.param("test/init_odom_z", init_odom.pose.pose.position.z, 1.0);

    // 6. loop
    ros::spin();

    return 0;
}

