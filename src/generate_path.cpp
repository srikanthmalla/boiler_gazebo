/*  file name: generate_path.cpp
 * Created on: Mar 8, 2015
 *      Author: Srikanth Malla
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "ca_nav_msgs/PathXYZVPsi.h"
#include <iostream> 
#include <vector>
#include <geometry_msgs/PoseArray.h>


nav_msgs::Odometry curr_pose;
geometry_msgs::PoseArray plist;

bool got_pose = false;
bool got_points =false;
void GetOdom(const nav_msgs::Odometry::ConstPtr &msg) 
{
  curr_pose = *msg;
  got_pose = true;
}


void waypoint_cb(const geometry_msgs::PoseArray& input)
{ plist.poses.clear();
  for(int i=0;i<input.poses.size();i++)
  {
    plist.poses.push_back(input.poses.at(i));
  }
  // printf ("getting points");
  if(plist.poses.size()>0)
  {
    got_points=true;
  }
  else
    got_points=false;

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_path_control");
  // ros::NodeHandle n("~");
  ros::NodeHandle n;
  ros::Publisher pub_command = n.advertise<ca_nav_msgs::PathXYZVPsi>("test_path_control/path", 10);
  ros::Publisher pub_marker = n.advertise<visualization_msgs::Marker>("test_path_control/path_marker",10);
  ros::Subscriber odometry_sub = n.subscribe<nav_msgs::Odometry>("test_path_control/pose", 10, GetOdom);
  ros::Subscriber waypoints_sub =n.subscribe("selec_waypoints", 1, waypoint_cb);
  ros::Rate loop_rate(10.0);

  while(!got_pose)
  {
    ros::spinOnce();
    loop_rate.sleep();
    std::cout<<"didnt get pose"<<std::endl;
  }
  std::cout<<"got pose"<<std::endl;
  while(!got_points)
  {
    ros::spinOnce();
    loop_rate.sleep();
    std::cout<<"didnt get points"<<std::endl;
  }
  std::cout<<plist.poses.size()<<std::endl;
  ca_nav_msgs::PathXYZVPsi path;
  ca_nav_msgs::XYZVPsi pt;

  pt.position.x = curr_pose.pose.pose.position.x+0.1*(curr_pose.pose.pose.position.x-plist.poses.at(0).position.x);  
  pt.position.y = curr_pose.pose.pose.position.y+0.1*(curr_pose.pose.pose.position.y-plist.poses.at(0).position.y); 
  pt.position.z = curr_pose.pose.pose.position.z+0.1*(curr_pose.pose.pose.position.z-plist.poses.at(0).position.z); 
  pt.heading = plist.poses.at(0).orientation.z;
  pt.vel = 0.3;
  path.waypoints.push_back(pt);
  for(int i=0; i<plist.poses.size(); i++)
  {
    pt.position.x = plist.poses.at(i).position.x;  
    pt.position.y = plist.poses.at(i).position.y; 
    pt.position.z = plist.poses.at(i).position.z; 
    ROS_INFO("%f  %f  %f",plist.poses.at(i).position.x,plist.poses.at(i).position.y,plist.poses.at(i).position.z);
    pt.vel = 0.3;
    pt.heading=plist.poses.at(i).orientation.z;
    path.waypoints.push_back(pt);
  }
  ros::Duration(0.5).sleep();
  pub_command.publish(path);

  visualization_msgs::Marker m;
  m.header.frame_id = "world";
  m.header.stamp = ros::Time::now();
  m.ns = "path";
  m.id = 0;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.1;
  m.type = visualization_msgs::Marker::LINE_STRIP; //correct marker type
  m.color.r = 0; m.color.g = 1; m.color.b = 0; m.color.a = 1;

  for (auto it : path.waypoints) 
  {
    geometry_msgs::Point ps;
    ps.x = it.position.x;
    ps.y = it.position.y;
    ps.z = it.position.z;
    m.points.push_back(ps);
  }


  pub_command.publish(path);
  while (ros::ok()) {
  ca_nav_msgs::PathXYZVPsi path;
  ca_nav_msgs::XYZVPsi pt;
  while(!got_points)
  {
    ros::spinOnce();
    loop_rate.sleep();
    std::cout<<"didnt got points"<<std::endl;
  }
  pt.position.x = curr_pose.pose.pose.position.x+0.1*(curr_pose.pose.pose.position.x-plist.poses.at(0).position.x);  
  pt.position.y = curr_pose.pose.pose.position.y+0.1*(curr_pose.pose.pose.position.y-plist.poses.at(0).position.y); 
  pt.position.z = curr_pose.pose.pose.position.z+0.1*(curr_pose.pose.pose.position.z-plist.poses.at(0).position.z); 
  pt.heading=plist.poses.at(0).orientation.z;
  pt.vel = 0.3;
  path.waypoints.push_back(pt);
  for(int i=0; i<plist.poses.size(); i++)
  {
    pt.position.x = plist.poses.at(i).position.x;  
    pt.position.y = plist.poses.at(i).position.y; 
    pt.position.z = plist.poses.at(i).position.z; 
    ROS_INFO("%f  %f  %f",plist.poses.at(i).position.x,plist.poses.at(i).position.y,plist.poses.at(i).position.z);
    pt.vel = 0.3;
    pt.heading=plist.poses.at(i).orientation.z;
    path.waypoints.push_back(pt);
  }
  pub_command.publish(path);

  visualization_msgs::Marker m;
  m.header.frame_id = "world";
  m.header.stamp = ros::Time::now();
  m.ns = "path";
  m.id = 0;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.1;
  m.type = visualization_msgs::Marker::LINE_STRIP; //correct marker type
  m.color.r = 0; m.color.g = 1; m.color.b = 0; m.color.a = 1;

  for (auto it : path.waypoints) {
    geometry_msgs::Point ps;
    ps.x = it.position.x;
    ps.y = it.position.y;
    ps.z = it.position.z;
    m.points.push_back(ps);
  }
    pub_marker.publish(m);
    pub_command.publish(path);
    ros::spinOnce();
    loop_rate.sleep();
  }
}



