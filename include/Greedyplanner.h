//author:srikanth malla

#ifndef _GREEDYPLANNER_H_
#define _GREEDYPLANNER_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "Markers.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include "ca_nav_msgs/PathXYZVPsi.h"
#include <iostream> 
#include <Eigen/Dense>
#include "boiler_gazebo/djicommand.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"

class Greedyplanner{
public:
	Greedyplanner();//constructor
	void GetOdom(const nav_msgs::Odometry::ConstPtr &msg); 
	void waypoint_cb(const geometry_msgs::PoseArray& input);
	void pathtracking();
	void transform_and_sendtopixhawk(const geometry_msgs::PoseArray & local_selec_waypoints);
	void dji_callback(const nav_msgs::Odometry& msg);
 
private:
	ros::NodeHandle nh_;
	ros::Publisher pub_waypoints,pub_waypoint_marker,local_pos_pub,dji_msg;
	ros::Subscriber waypoints_sub,odometry_sub,dji_odom_sub;

	geometry_msgs::PoseArray waypoints,selec_waypoints;
	geometry_msgs::Pose lidar_odom,dji_odom;
	
	boiler_gazebo::djicommand vel_and_heading_msg;
	std::string mode, controller;

};
void Greedyplanner::GetOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
	selec_waypoints.poses.clear();
	nav_msgs::Odometry curr_pose = *msg;

	lidar_odom.position = curr_pose.pose.pose.position;
	lidar_odom.orientation =curr_pose.pose.pose.orientation;

}
void Greedyplanner::dji_callback(const nav_msgs::Odometry& msg)
{

    nav_msgs::Odometry dji_pose = msg;

	dji_odom.position = dji_pose.pose.pose.position;
	dji_odom.orientation =dji_pose.pose.pose.orientation;

}

Greedyplanner::Greedyplanner(){
	pub_waypoints=nh_.advertise<geometry_msgs::PoseArray>("selec_waypoints",10);
	// pub_waypoint_marker = nh_.advertise<visualization_msgs::Marker> ("tsp_waypoint", 10);
	
 	nh_.param<std::string>("mode", mode, "real");
 	nh_.param<std::string>("controller", controller, "dji");
    if(mode=="real")
    {
    	odometry_sub = nh_.subscribe<nav_msgs::Odometry>("lidar_ekf/imu_odom", 10, &Greedyplanner::GetOdom, this);
	}
    else
    {
      	odometry_sub = nh_.subscribe<nav_msgs::Odometry>("quad_sim_sysid_node/pose", 10, &Greedyplanner::GetOdom, this);
    }
	waypoints_sub =nh_.subscribe("waypoints", 1, &Greedyplanner::waypoint_cb,this);
	if(controller=="pixhawk")
    {
		local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
	}
	if(controller=="dji")
	{
		dji_msg = nh_.advertise<boiler_gazebo::djicommand> ("trajectory_control/command", 50);
		dji_odom_sub= nh_.subscribe("/dji_odom", 10, &Greedyplanner::dji_callback,this);
	}
}

void Greedyplanner::waypoint_cb(const geometry_msgs::PoseArray& input)
{ 
	waypoints.poses.clear();
	for (int i = 0; i < input.poses.size(); ++i)
	{
	  waypoints.poses.push_back(input.poses.at(i));
	}
	int closest_id=0;float dist=0,temp=0;
	if(waypoints.poses.size()==0)
	{	selec_waypoints.poses.push_back(lidar_odom);
		pub_waypoints.publish(selec_waypoints);
	}
	else
	{
		temp=sqrt(pow(lidar_odom.position.x-waypoints.poses.at(0).position.x,2)+pow(lidar_odom.position.y-waypoints.poses.at(0).position.y,2)+pow(lidar_odom.position.z-waypoints.poses.at(0).position.z,2));
		for(int i=1;i<waypoints.poses.size();i++)
		{
			dist=sqrt(pow(lidar_odom.position.x-waypoints.poses.at(i).position.x,2)+pow(lidar_odom.position.y-waypoints.poses.at(i).position.y,2)+pow(lidar_odom.position.z-waypoints.poses.at(i).position.z,2));
			if (dist<temp)
			{
				temp=dist;
				closest_id=i;

			}
		}
		selec_waypoints.poses.push_back(waypoints.poses.at(closest_id));
		transform_and_sendtopixhawk(selec_waypoints);
		pub_waypoints.publish(selec_waypoints);
	}
	// printf ("getting points\n");
}

void Greedyplanner::transform_and_sendtopixhawk(const geometry_msgs::PoseArray & local_selec_waypoints)
{
	Eigen::Matrix3d Rx;
	Rx<< 1, 0, 0,
		 0,-1, 0,
		 0, 0,-1;
	Eigen::Vector3d point(local_selec_waypoints.poses.at(0).position.x,local_selec_waypoints.poses.at(0).position.y,local_selec_waypoints.poses.at(0).position.z);
    // Eigen::Vector3d point(5,5,5);
    Eigen::Vector3d heading(cos(local_selec_waypoints.poses.at(0).orientation.z),sin(local_selec_waypoints.poses.at(0).orientation.z),0);
    Eigen::Vector3d tranformed_point=Rx*point;
    Eigen::Vector3d tranformed_heading=Rx*heading;
    // std::cout << "Here is Transformed point:\n" << tranformed_point << std::endl;
    float a=0,b=0,c=0;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = tranformed_point[0]+a;
    pose.pose.position.y = tranformed_point[1]+b;
    pose.pose.position.z = tranformed_point[2]+c;
    pose.pose.orientation.x=tranformed_heading[0];
	pose.pose.orientation.y=tranformed_heading[1];
    pose.pose.orientation.z=tranformed_heading[2];
    pose.pose.orientation.w=0;
	if(controller=="pixhawk")
    {
	    local_pos_pub.publish(pose);
	}
	//p control velocity publishing
    if(controller=="dji")
	{
		float p_gain=1.0;
		float max_vel = 0.4;
        vel_and_heading_msg.velocity.x = p_gain*(pose.pose.position.x-lidar_odom.position.x);
        vel_and_heading_msg.velocity.y = p_gain*(pose.pose.position.y-lidar_odom.position.y);
        vel_and_heading_msg.velocity.z = p_gain*(pose.pose.position.z-lidar_odom.position.z);
        if(vel_and_heading_msg.velocity.x>max_vel)
        {
        	vel_and_heading_msg.velocity.x=max_vel;
        }
		if(vel_and_heading_msg.velocity.y>max_vel)
        {
        	vel_and_heading_msg.velocity.y=max_vel;
        }
		if(vel_and_heading_msg.velocity.z>max_vel)
        {
        	vel_and_heading_msg.velocity.z=max_vel;
        }
        if(vel_and_heading_msg.velocity.x<-max_vel)
        {
        	vel_and_heading_msg.velocity.x=-max_vel;
        }
		if(vel_and_heading_msg.velocity.y<-max_vel)
        {
        	vel_and_heading_msg.velocity.y=-max_vel;
        }
		if(vel_and_heading_msg.velocity.z<-max_vel)
        {
        	vel_and_heading_msg.velocity.z=-max_vel;
        }
        Eigen::Vector3d vel_map(vel_and_heading_msg.velocity.x,vel_and_heading_msg.velocity.y,vel_and_heading_msg.velocity.z);
        //Eigen::Quaterniond qi(lidar_odom.orientation.w,lidar_odom.orientation.x,lidar_odom.orientation.y,lidar_odom.orientation.z);
        //Eigen::Quaterniond qd(dji_odom.orientation.w,dji_odom.orientation.x,dji_odom.orientation.y,dji_odom.orientation.z);
        
        tf::Quaternion qi(lidar_odom.orientation.x,lidar_odom.orientation.y,lidar_odom.orientation.z,lidar_odom.orientation.w);
        tf::Quaternion qd(dji_odom.orientation.x,dji_odom.orientation.y,dji_odom.orientation.z,dji_odom.orientation.w);
        
        tf::Matrix3x3 Ri(qi);
        tf::Matrix3x3 Rd(qd);
        
        tf::Matrix3x3 R = Ri.transpose();
        
        Eigen::Matrix3d R_eigen;
        tf::matrixTFToEigen(R,R_eigen);
                                    
        Eigen::Vector3d vel_dji=R_eigen*vel_map;
        
        vel_and_heading_msg.velocity.x=vel_dji[0];
        vel_and_heading_msg.velocity.y=vel_dji[1];
        vel_and_heading_msg.velocity.z=vel_dji[2];
        
        vel_and_heading_msg.heading = local_selec_waypoints.poses.at(0).orientation.z;
        dji_msg.publish(vel_and_heading_msg);
	}

}
void Greedyplanner::pathtracking()
{
  
}

#endif
