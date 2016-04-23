/*file name: image_area.cpp
Finding the image patch coordinates intersection with the plane given the pose and orientation of quadrotor
Author:srikanth malla
Date:November 30,2015
*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h> //conversions b/w ros pointcloud and pcl point cloud msgs
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>//for pcl types

#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/foreach.hpp>
#include <boiler_gazebo/Coorxyz.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <lemon/list_graph.h>
#include <lemon/insertion_tsp.h>
#include <lemon/greedy_tsp.h>
#include <tsp/tsp.h>
#include <lemon/nearest_neighbor_tsp.h>



typedef lemon::FullGraph FulGraph;
typedef FulGraph::EdgeIt EdgeIt;
typedef FulGraph::Edge Edge;
typedef FulGraph::NodeIt NodeIt;
typedef FulGraph::Node Node;
typedef FulGraph::EdgeMap<double> LengthMap;
FulGraph g;
LengthMap cost_map(g);
int huge_cost=100000;
pcl::PointCloud<pcl::PointXYZ> map_points;//msg is container of input cloud, and new_msg is manipulated cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr new_msg (new pcl::PointCloud<pcl::PointXYZ>); 
ros::Publisher pub1,pub2,pub_waypoint_marker;
boiler_gazebo::Coorxyz custom;
using namespace Eigen;
//using static map plane
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud msg,unsorted_msg;

visualization_msgs::Marker plane_marker,waypoint_marker_msg;


// plane, it passes through 3 points
Hyperplane<double,3> focalPlane ;

void coor_callback(const boiler_gazebo::Coorxyz& points)
{
  custom=points;
  new_msg->clear();
  map_points.clear();
   //plane
  printf ("markerx1 = %f, marker_y1 = %f, marker_z1=%f\n", custom.x[1],custom.y[1],custom.z[1]);
  Vector3d p1(custom.x[1],custom.y[1],custom.z[1]);
  Vector3d p2(custom.x[2],custom.y[2],custom.z[2]);
  Vector3d p3(custom.x[3],custom.y[3],custom.z[3]);
  Hyperplane<double,3>::Through( p1,p2,p3 );

  new_msg->header.frame_id = "world";
  new_msg->height= msg.height;
  new_msg->width = msg.width;
  map_points.height=msg.height;
  map_points.width=msg.width;
  map_points.header.frame_id="world";
  printf ("Cloud: width = %d, height = %d\n", msg.width, msg.height);
    int i=0;
    while(i<(msg.height*msg.width))
 	{ //constrained within the boundaries of camera ray projection
	  	// printf ("entered into boost");
	  	bool cx1,cx2,cy1,cy2,cz1,cz2;
	    cx1= ((custom.x[1]<=(msg.points.at(i).x)) | (custom.x[2]<=(msg.points.at(i).x)) | (custom.x[4]<=(msg.points.at(i).x)) | (custom.x[5]<=(msg.points.at(i).x))) ;
	    cx2= ((custom.x[1]>=(msg.points.at(i).x)) | (custom.x[2]>=(msg.points.at(i).x)) | (custom.x[4]>=(msg.points.at(i).x)) | (custom.x[5]>=(msg.points.at(i).x))) ;
	    cz1= ((custom.z[1]<=(msg.points.at(i).z)) | (custom.z[2]<=(msg.points.at(i).z)) | (custom.z[4]<=(msg.points.at(i).z)) | (custom.z[5]<=(msg.points.at(i).z))) ;
	    cz2= ((custom.z[1]>=(msg.points.at(i).z)) | (custom.z[2]>=(msg.points.at(i).z)) | (custom.z[4]>=(msg.points.at(i).z)) | (custom.z[5]>=(msg.points.at(i).z))) ;
	   //  //and should be lying on th plane

	  	cy1=cy2=1;
	    if((cx1 & cx2 & cy1 & cy2 & cz1 & cz2)&((abs(((focalPlane.coeffs()[0])*(msg.points.at(i).x))+((focalPlane.coeffs()[1])*(msg.points.at(i).y))+((focalPlane.coeffs()[2])*(msg.points.at(i).z))-(focalPlane.coeffs()[3]))<0.5))) 
	    {
	     ROS_ERROR_STREAM("Size::"<<msg.size()<<" i::"<<i<<" ID::"<<i<<msg.height);  

	     PointCloud::iterator it = msg.begin()+i; 	
	     msg.erase(it);//while erasing size changes
	     new_msg->resize(msg.size());
	     map_points.resize(msg.size());
	     i=i-1;
	    }
	    else
	    {  
	       new_msg->points.push_back (pcl::PointXYZ(msg.points.at(i).x, msg.points.at(i).y+2, msg.points.at(i).z));//not covered wp
	       map_points.points.push_back (pcl::PointXYZ(msg.points.at(i).x, msg.points.at(i).y, msg.points.at(i).z));//not covered map points
	      
	    }
	    i=i+1;
	};
  // std::cout<< "dasfasdfasf"<<std::endl;
  // // // Perform the actual filtering
  // pcl::VoxelGrid<pcl::PointXYZ> sor;
  // sor.setInputCloud (new_msg);
  // std::cout<< "dasfasdfasf"<<std::endl;
  // sor.setLeafSize (2, 2, 2);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
  // std::cout<< "dasfasdfasf"<<std::endl;
  // sor.filter (*tempCloud);
  // new_msg=tempCloud;


  std_msgs::ColorRGBA color_msg_local;
  geometry_msgs::Point point_local;
  waypoint_marker_msg.points.resize(new_msg->points.size());
  waypoint_marker_msg.colors.resize(new_msg->points.size());
  for(int i = 0; i < new_msg->points.size(); i++)
  { 
      waypoint_marker_msg.points.at(i).x = new_msg->points.at(i).x;       
      waypoint_marker_msg.points.at(i).y = new_msg->points.at(i).y;      
      waypoint_marker_msg.points.at(i).z = new_msg->points.at(i).z; 
      std::cout<<"marker.x"<<waypoint_marker_msg.points.at(i).x<<std::endl;
      waypoint_marker_msg.colors.at(i).r=0;
      waypoint_marker_msg.colors.at(i).g=0;
      waypoint_marker_msg.colors.at(i).b=1;
      waypoint_marker_msg.colors.at(i).a=1;
      
  }

  pub_waypoint_marker.publish(waypoint_marker_msg);

  pub1.publish(*new_msg);
  pub2.publish(msg);

}

int decrement_update(int x,int length)
{
  if(x==0)
  {
    x=length-1;
  }
  else
  {
    x=x-1;
  }
  return x;
}




    
int main (int argc, char** argv)
{
  msg.header.frame_id = "world";


  unsorted_msg.header.frame_id = "world";
  msg.height =  unsorted_msg.height =10;  msg.width =  unsorted_msg.width = 10;
  for(int j=0;j<unsorted_msg.height;j++)
  {
    for(int i=0; i<unsorted_msg.width;i++)
    {
      unsorted_msg.points.push_back (pcl::PointXYZ(i-5,1,j)); //y=1 plane
    }
  }
  //sorting based on full lenght tsp
  int tour_length=(unsorted_msg.height * unsorted_msg.width) +1;
  g.resize(tour_length);
  int start_position =9;
  int i=0,j=0;
  for (EdgeIt l(g); l!=lemon::INVALID; ++l)
  { 
    i=g.id(g.u(l)); j=g.id(g.v(l));
    if((i==tour_length-1)|(j==tour_length-1))
    {
      if((i==start_position)| (j==start_position))
      {
        cost_map[l]=0;
      }
      else
      {
        cost_map[l]=huge_cost;
      }
    }
    else
    { 
      cost_map[l] =  sqrt(pow((unsorted_msg.points.at(i).x-unsorted_msg.points.at(j).x),2)+pow((unsorted_msg.points.at(i).y-unsorted_msg.points.at(j).y),2)+(2*pow((unsorted_msg.points.at(i).z-unsorted_msg.points.at(j).z),2)));  
    }
  }
  lemon::NearestNeighborTsp<LengthMap> tsp_solver(g,cost_map);
  std::cout<<"TSP Cost::"<<tsp_solver.run()<<"\n";
  std::vector<Node> tour_t=tsp_solver.tourNodes();
  int start_index;
  // copy planned tour to tour vector
  for (i=0;i<tour_t.size();i++)
  {
    if (g.id(tour_t.at(i))==start_position)
    {
      start_index=i;
    }
  }
  std::vector<int> new_tour;
  int x=start_index;
  new_tour.clear();
  for( i=0;i<tour_t.size()-1;i++)
  {
    new_tour.push_back(g.id(tour_t.at(x)));     
    x=decrement_update(x,tour_length);
    std::cout<<new_tour.at(i)<<"->";
  }
  std::cout<<std::endl;
  i=0;
  while (i<new_tour.size())
  { std::cout<<g.id(tour_t.at(i))<<std::endl;
    msg.points.push_back(unsorted_msg.points.at(new_tour.at(i)));
    i++;
  }


  // ROS_ERROR_STREAM("Size::"<<msg.size());  
	 //     //printf ("\t(%f, %f, %f)\n", msg.points[i+(j*msg.height)].x, msg.points[i+(j*msg.height)].y, msg.points[i+(j*msg.height)].z);//covered
  // PointCloud::iterator it = msg.begin(); 	
  // msg.erase(it+1);
  // printf("%d\n",msg.points.at(0).x );

  // Initialize ROS
  ros::init (argc, argv, "uncovered_regions");
  ros::NodeHandle nh;
   // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe ("cloud_pcd", 1, cloud_cb);
  ros::Subscriber sub_coor = nh.subscribe ("coor", 10,coor_callback);
  pub1 = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("waypoints", 1);
  pub2 = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("uncovered", 1);
  ros::Publisher pub_plane = nh.advertise<visualization_msgs::Marker> ("plane_msg", 10);
  pub_waypoint_marker = nh.advertise<visualization_msgs::Marker> ("all_nodes", 10);


  //waypoints
  waypoint_marker_msg.header.frame_id = "/world"; // TODO
  waypoint_marker_msg.header.stamp = ros::Time();
  waypoint_marker_msg.ns = "marker_test_triangle_list";
  waypoint_marker_msg.id = 1;
  waypoint_marker_msg.type = visualization_msgs::Marker::POINTS;
  waypoint_marker_msg.action = visualization_msgs::Marker::ADD;
  waypoint_marker_msg.pose.position.x = 0.0;
  waypoint_marker_msg.pose.position.y = 0.0;
  waypoint_marker_msg.pose.position.z = 0.0;
  waypoint_marker_msg.pose.orientation.x = 0.0;
  waypoint_marker_msg.pose.orientation.y = 0.0;
  waypoint_marker_msg.pose.orientation.z = 0.0;
  waypoint_marker_msg.pose.orientation.w = 1.0;

  waypoint_marker_msg.scale.x = 0.1;
  waypoint_marker_msg.scale.y = 0.1;
  waypoint_marker_msg.scale.z = 0.1;



  //plane 
  plane_marker.header.frame_id = "/world"; // TODO
  plane_marker.header.stamp = ros::Time();
  plane_marker.ns = "marker_test_triangle_list";
  plane_marker.id = 0;
  plane_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  plane_marker.action = visualization_msgs::Marker::ADD;
  plane_marker.pose.position.x = 0.0;
  plane_marker.pose.position.y = -1;
  plane_marker.pose.position.z = 0.0;
  plane_marker.pose.orientation.x = 0.0;
  plane_marker.pose.orientation.y = 0.0;
  plane_marker.pose.orientation.z = 0.0;
  plane_marker.pose.orientation.w = 1.0;

  plane_marker.scale.x = 1.0;
  plane_marker.scale.y = 1.0;
  plane_marker.scale.z = 1.0;

std::cout << "publisher "<< std::endl;


std_msgs::ColorRGBA color_msg_local;
color_msg_local.r = 1;
color_msg_local.g = 1;
color_msg_local.b = 1;
color_msg_local.a = 1;

geometry_msgs::Point point_local;

point_local.x = -20;       
point_local.y = 0;      
point_local.z = -10; 
   
plane_marker.points.push_back(point_local);
plane_marker.colors.push_back(color_msg_local);

point_local.x = 20;       
point_local.y = 0;      
point_local.z = -10; 
   
plane_marker.points.push_back(point_local);
plane_marker.colors.push_back(color_msg_local);
point_local.x = -20;       
point_local.y = 0;      
point_local.z = 20; 
   
plane_marker.points.push_back(point_local);
plane_marker.colors.push_back(color_msg_local);
point_local.x = -20;       
point_local.y = 0;      
point_local.z = 20; 
   
plane_marker.points.push_back(point_local);
plane_marker.colors.push_back(color_msg_local);

point_local.x = 20;       
point_local.y = 0;      
point_local.z = -10; 
   
plane_marker.points.push_back(point_local);
plane_marker.colors.push_back(color_msg_local);

point_local.x = 20;       
point_local.y = 0;      
point_local.z = 20; 
   
plane_marker.points.push_back(point_local);
plane_marker.colors.push_back(color_msg_local);

while(ros::ok())
{  
  pub_plane.publish(plane_marker);
  // Spin
  ros::spinOnce();
}

}