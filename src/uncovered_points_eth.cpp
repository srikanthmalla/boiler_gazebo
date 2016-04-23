//this consists of markers and computation of which triangle 
//covered or not covered in the mesh file
//author: Srikanth Malla
#include "Markers.h"
#include "Filereader.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/Path.h>
#include <ros/package.h> //ros::package::getPath

using namespace std;

vector< vector<double> > csv_values,tour,mesh_coor;
Markers marker;
vector<int> covered_ids;
pcl::PointCloud<pcl::PointXYZ> waypoint_msg,dummy_msg;
visualization_msgs::Marker waypoint_marker_msg,mesh_marker, linelist,covered_regions;
//publishers declaration
ros::Publisher pub_waypoint,pub_covered_regions,pub_waypoint_marker,pub_mesh_marker, pub_linelist;
// int covered_no=0;
void GetOdom(const nav_msgs::Odometry::ConstPtr &msg) 
{
  nav_msgs::Odometry curr_pose = *msg;

  geometry_msgs::Point ps;
  ps.x = curr_pose.pose.pose.position.x;
  ps.y = curr_pose.pose.pose.position.y;
  ps.z = curr_pose.pose.pose.position.z;
  
  int i=0;
  while(i<(waypoint_msg.height*waypoint_msg.width))
  {
    float error=sqrt(pow((waypoint_msg.points.at(i).x-ps.x),2)+pow((waypoint_msg.at(i).y-ps.y),2)+ pow((waypoint_msg.at(i).z-ps.z),2));
    if (error<0.5)
    {
      waypoint_msg.erase(waypoint_msg.begin()+i);
    }
    i++;
  }
  i=0;
  while(i<(dummy_msg.height*dummy_msg.width))
  {
    float error=sqrt(pow((dummy_msg.points.at(i).x-ps.x),2)+pow((dummy_msg.at(i).y-ps.y),2)+ pow((dummy_msg.at(i).z-ps.z),2));
    if (error<0.5)
    {
      if (i==dummy_msg.width-1)
      {
        covered_ids.push_back(1);
        cout<<"id::"<<1<<endl;
      }
      else
      {
        covered_ids.push_back(csv_values[i][6]);
        cout<<"id::"<<csv_values[i][6]<<endl;
      }
    }
    i++;
  }
  
  cout<<"no of waypoints to be covered:"<<(waypoint_marker_msg.points.size()/2)<<endl;

  std_msgs::ColorRGBA color_msg_local,redcolor;
  geometry_msgs::Point point_local;
  waypoint_marker_msg.points.resize(waypoint_msg.height*waypoint_msg.width);
  waypoint_marker_msg.colors.resize(waypoint_msg.height*waypoint_msg.width);
  for (int i=0;i<(waypoint_msg.height*waypoint_msg.width);i++)
  {
      waypoint_marker_msg.points.at(i).x = waypoint_msg.points.at(i).x;       
      waypoint_marker_msg.points.at(i).y = waypoint_msg.points.at(i).y;      
      waypoint_marker_msg.points.at(i).z = waypoint_msg.points.at(i).z; 
      waypoint_marker_msg.colors.at(i).r=0;
      waypoint_marker_msg.colors.at(i).g=0;
      waypoint_marker_msg.colors.at(i).b=1;
      waypoint_marker_msg.colors.at(i).a=1;
  }

   //assigning each triangle of mesh to triangle list marker
  redcolor.r=1;redcolor.b=0;redcolor.g=0;redcolor.a=1;
  for(int i=0;i<covered_ids.size();i++)
  { int x=int(covered_ids.at(i)-2)*3;
    if(x>=0)
    { 
      marker.add_point(covered_regions,mesh_marker.points.at(x),redcolor);
      marker.add_point(covered_regions,mesh_marker.points.at(x+1),redcolor);
      marker.add_point(covered_regions,mesh_marker.points.at(x+2),redcolor);
    }
  }
  covered_ids.clear();

  pub_waypoint_marker.publish(waypoint_marker_msg);
  pub_waypoint.publish(waypoint_msg);
  pub_mesh_marker.publish(mesh_marker);
  pub_covered_regions.publish(covered_regions);

}

int main(int argc, char** argv)
{ 
  marker.line_list(linelist);
  ros::init (argc, argv, "uncovered_regions");
  ros::NodeHandle nh;
  //publishers
  pub_waypoint = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("waypoints", 1);
  pub_waypoint_marker = nh.advertise<visualization_msgs::Marker> ("all_nodes", 10);
  pub_mesh_marker= nh.advertise<visualization_msgs::Marker> ("mesh_marker", 10);
  pub_linelist = nh.advertise<visualization_msgs::Marker> ("line_list",10);
  pub_covered_regions=nh.advertise<visualization_msgs::Marker>("covered_regions",10);
  //subscribers
  ros::Subscriber odometry_sub = nh.subscribe<nav_msgs::Odometry>("quad_sim_sysid_node/pose", 1, GetOdom);
  //reading stl file
  Filereader file;
  string path=ros::package::getPath("koptplanner")+"/src/mesh.txt";
  file.read(path,mesh_coor);

  marker.triangle_list(mesh_marker);
  marker.triangle_list(covered_regions);
  std_msgs::ColorRGBA mesh_color;
  geometry_msgs::Point mesh_point;
  //define mesh color
  mesh_color.r = 0 ;
  mesh_color.g = 0.5 ;
  mesh_color.b = 0.5 ;
  mesh_color.a = 0.5;
  //assigning each triangle of mesh to triangle list marker
  for(int i=0;i<mesh_coor.size();i++)
  {

    mesh_point.x = mesh_coor[i][0];
    mesh_point.y = mesh_coor[i][1];
    mesh_point.z = mesh_coor[i][2];  
    mesh_marker.points.push_back(mesh_point);
    // marker.add_point(mesh_marker,mesh_point,mesh_color);
  }
  mesh_marker.color=mesh_color;
  path=ros::package::getPath("boiler_gazebo")+"/src/latestPath.csv";
  file.read(path,csv_values);

  // display results
  cout.precision(2);
  cout.setf(ios::fixed,ios::floatfield);
  path=ros::package::getPath("koptplanner")+"/src/tour.txt";
  file.read(path,tour);  


  // display results
  cout.precision(2);
  cout.setf(ios::fixed,ios::floatfield);

  //waypoint marker msg basically points
  marker.points(waypoint_marker_msg);
  //scaling down the size
  waypoint_marker_msg.scale.x = 0.1;
  waypoint_marker_msg.scale.y = 0.1;
  waypoint_marker_msg.scale.z = 0.1;

  std_msgs::ColorRGBA color_msg_local;
  waypoint_marker_msg.points.resize(csv_values.size());
  waypoint_marker_msg.colors.resize(csv_values.size());

  waypoint_msg.header.frame_id = "world"; //point cloud of positions
  waypoint_msg.height = 1; waypoint_msg.width =csv_values.size();
  for (int i=0;i<csv_values.size();i++)
  {
      for(int j=0;j<csv_values[i].size();j++)
      {
         cout<<csv_values[i][j]<<"  ";
      }
      cout<<endl;
      waypoint_msg.points.push_back(pcl::PointXYZ(csv_values[i][0],csv_values[i][1],csv_values[i][2]));
  }
  geometry_msgs::Point point_local;
  mesh_color.r=0.5;
  for(int i=0;i<csv_values.size();i++)
  { 
    int l=csv_values[i][6]-2;
    if(l>=0)
    {
      point_local.x=csv_values[i][0];point_local.y=csv_values[i][1];point_local.z=csv_values[i][2];
      marker.add_point(linelist,point_local,mesh_color);
     
      point_local.x=(mesh_marker.points.at(l*3).x+mesh_marker.points.at(l*3+1).x+mesh_marker.points.at(l*3+2).x)/3;
      point_local.y=(mesh_marker.points.at(l*3).y+mesh_marker.points.at(l*3+1).y+mesh_marker.points.at(l*3+2).y)/3;
      point_local.z=(mesh_marker.points.at(l*3).z+mesh_marker.points.at(l*3+1).z+mesh_marker.points.at(l*3+2).z)/3;
      marker.add_point(linelist,point_local,mesh_color);
    }
  }

  //dummy_msg because by erasing the waypoint we will not get correct id of covered triangle

  dummy_msg.header.frame_id = "world"; //point cloud of positions
  dummy_msg.height = 1; dummy_msg.width =csv_values.size();
  for (int i=0;i<csv_values.size();i++)
  {
    dummy_msg.points.push_back(pcl::PointXYZ(csv_values[i][0],csv_values[i][1],csv_values[i][2]));
  }
  ros::Rate loop_rate(10.0);
  while(ros::ok())
  {  
     pub_linelist.publish(linelist);
     ros::spinOnce();
     loop_rate.sleep();
  }
}

