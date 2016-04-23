#include <ros/ros.h>
#include "Greedyplanner.h"
#include <math.h>
#include "Markers.h"
geometry_msgs::PoseArray waypoints;

ros::Publisher pub_waypointlist, waypoint_marker_pub,path_marker_pub;
visualization_msgs::Marker waypoints_marker,path_marker;
void GetOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
	geometry_msgs::Pose ps;
	nav_msgs::Odometry curr_pose = *msg;
	ps.position = curr_pose.pose.pose.position;
	ps.orientation =curr_pose.pose.pose.orientation;
	if(waypoints.poses.size()>1)
	{
		float error=sqrt(pow((waypoints.poses.at(0).position.x-ps.position.x),2)+pow((waypoints.poses.at(0).position.y-ps.position.y),2)+ pow((waypoints.poses.at(0).position.z-ps.position.z),2));
		if(error<0.2)
		{
			waypoints.poses.erase(waypoints.poses.begin());
		}
	}
pub_waypointlist.publish(waypoints);
waypoint_marker_pub.publish(waypoints_marker);
path_marker_pub.publish(path_marker);
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "demo");
  ros::NodeHandle nh;

  pub_waypointlist= nh.advertise<geometry_msgs::PoseArray> ("waypoints", 10);
  //ros::NodeHandle nh;
  Greedyplanner gp;

  Markers marker;
  marker.points(waypoints_marker);
  marker.line_list(path_marker);
  waypoint_marker_pub= nh.advertise<visualization_msgs::Marker> ("waypoint_marker", 10);
  path_marker_pub= nh.advertise<visualization_msgs::Marker> ("path_marker", 10);

  ros::Subscriber odometry_sub = nh.subscribe<nav_msgs::Odometry>("lidar_ekf/imu_odom", 10, GetOdom);
  //give trajectoris to the waypoints 
  geometry_msgs::Pose ps;geometry_msgs::Point waypoint; waypoints_marker.color.a=1;waypoints_marker.color.b=1;
  ps.position.x=0; ps.position.y=0; ps.position.z=1.2; ps.orientation.z=0; waypoints.poses.push_back(ps);
  waypoint.x=ps.position.x;waypoint.y=ps.position.y;waypoint.z=ps.position.z;waypoints_marker.points.push_back(waypoint);
 // ps.position.x=1; ps.position.y=0; ps.position.z=1.2; ps.orientation.z=0; waypoints.poses.push_back(ps);
 // waypoint.x=ps.position.x;waypoint.y=ps.position.y;waypoint.z=ps.position.z;waypints_marker.points.push_back(waypoint);
 // ps.position.x=1; ps.position.y=1; ps.position.z=1.2; ps.orientation.z=0; waypoints.poses.push_back(ps);
// waypoint.x=ps.position.x;waypoint.y=ps.position.y;waypoint.z=ps.position.z;waypints_marker.points.push_back(waypoint);
  //ps.position.x=0; ps.position.y=1; ps.position.z=1.2; ps.orientation.z=0; waypoints.poses.push_back(ps);
  // waypoint.x=ps.position.x;waypoint.y=ps.position.y;waypoint.z=ps.position.z;waypints_marker.points.push_back(waypoint);
  //ps.position.x=0; ps.position.y=0; ps.position.z=1.2; ps.orientation.z=0; waypoints.poses.push_back(ps);
// waypoint.x=ps.position.x;waypoint.y=ps.position.y;waypoint.z=ps.position.z;waypints_marker.points.push_back(waypoint);
  ros::Rate loop_rate(10.0);

  path_marker.color.a=1;path_marker.color.g=1;
  for(int i=0;i<waypoints.poses.size()-1;i++)
  {
    path_marker.points.push_back(waypoints_marker.points.at(i));
    path_marker.points.push_back(waypoints_marker.points.at(i+1));
  }

  while(ros::ok())
  {   
    ros::spinOnce();
    loop_rate.sleep();
  }


}
