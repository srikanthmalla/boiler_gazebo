/*
stl file reading, and slicing the mesh into planes (horizontal with the mesh)
*/
#include "Filereader.h"
#include "Markers.h"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h> //ros::package::getPath
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>

using namespace Eigen;

vector< vector<double> > mesh_coor;
pcl::PointCloud<pcl::PointXYZ> waypoint_msg;
visualization_msgs::Marker mesh_marker,planes,waypoint_marker,local_waypoints,local_path,cylinder_marker,cylinder_lines,camera_rays,coverage;
visualization_msgs::MarkerArray path_vec;
float z_min=0,z_max=0,x_min=0,x_max=0,y_min=0,y_max=0,steps=0,step_height=0;
ros::Publisher pub_waypointlist,pub_camera_rays,pub_coverage;
geometry_msgs::PoseArray plist;
bool sameside(Vector3d p1,Vector3d p2,Vector3d a,Vector3d b)
{
 Vector3d c1=(b-a).cross(p1-a);
 Vector3d c2=(b-a).cross(p2-a);

 if ((c1.dot(c2))>=0)
  return true;
 else
  return false;
}
void intersect(const nav_msgs::Odometry& curr_pose)
{ 
  camera_rays.points.clear();
  coverage.points.clear();
  geometry_msgs::Point ps,rays_local;
  ps= curr_pose.pose.pose.position;
  geometry_msgs::Quaternion orientation; 
  orientation=curr_pose.pose.pose.orientation;
  double fov=M_PI/9;//20 degrees 
  Vector3d ray1(sin((M_PI/2)-(fov/2))*cos(fov/2), sin((M_PI/2)-(fov/2))*sin(fov/2), cos((M_PI/2)-(fov/2)));
  Vector3d ray2(sin((M_PI/2)-(fov/2))*cos((2*M_PI)-(fov/2)), sin((M_PI/2)-(fov/2))*sin((2*M_PI)-(fov/2)),cos((M_PI/2)-(fov/2)));
  Vector3d ray3(sin((M_PI/2)+(fov/2))*cos(fov/2), sin((M_PI/2)+(fov/2))*sin(fov/2), cos((M_PI/2)+(fov/2)));
  Vector3d ray4(sin((M_PI/2)+(fov/2))*cos((2*M_PI)-(fov/2)), sin((M_PI/2)+(fov/2))*sin((2*M_PI)-(fov/2)),cos((M_PI/2)+(fov/2)));
  Vector3d a,b,c,pose(ps.x,ps.y,ps.z);Eigen::Quaterniond rot1,rot2,rot3,rot4;
  Eigen::Quaterniond p,q(orientation.w,orientation.x,orientation.y,orientation.z); p.w() = 0;
  p.vec() = ray1; rot1 = q * p * q.inverse(); p.vec() = ray2; rot2 = q * p * q.inverse(); 
  p.vec() = ray3; rot3 = q * p * q.inverse(); p.vec() = ray4; rot4 = q * p * q.inverse();

  int rl=5;//rl is for ray length
  camera_rays.points.push_back(ps);rays_local.x=(rl*rot1.vec()+pose)[0];rays_local.y=(rl*rot1.vec()+pose)[1];rays_local.z=(rl*rot1.vec()+pose)[2];camera_rays.points.push_back(rays_local);
  camera_rays.points.push_back(ps);rays_local.x=(rl*rot2.vec()+pose)[0];rays_local.y=(rl*rot2.vec()+pose)[1];rays_local.z=(rl*rot2.vec()+pose)[2];camera_rays.points.push_back(rays_local);
  camera_rays.points.push_back(ps);rays_local.x=(rl*rot3.vec()+pose)[0];rays_local.y=(rl*rot3.vec()+pose)[1];rays_local.z=(rl*rot3.vec()+pose)[2];camera_rays.points.push_back(rays_local);
  camera_rays.points.push_back(ps);rays_local.x=(rl*rot4.vec()+pose)[0];rays_local.y=(rl*rot4.vec()+pose)[1];rays_local.z=(rl*rot4.vec()+pose)[2];camera_rays.points.push_back(rays_local);
    
  Vector3d r1, r2, r3, r4;//intersected points
  for(int i=0;i<mesh_marker.points.size();i=i+3)
  {
    a[0]=(mesh_marker.points.at(i).x);a[1]=(mesh_marker.points.at(i).y);a[2]=(mesh_marker.points.at(i).z);
    b[0]=(mesh_marker.points.at(i+1).x);b[1]=(mesh_marker.points.at(i+1).y);b[2]=(mesh_marker.points.at(i+1).z);
    c[0]=(mesh_marker.points.at(i+2).x);c[1]=(mesh_marker.points.at(i+2).y);c[2]=(mesh_marker.points.at(i+2).z);
    // plane, it passes through 3 points
    Hyperplane<double,3> focalPlane = Hyperplane<double,3>::Through( a,b,c );

    ParametrizedLine<double,3> pline = ParametrizedLine<double,3>::Through(pose,pose+(rot1.vec()));
    double intersection1 = pline.intersection( focalPlane ) ;
    r1=(intersection1*( (rot1.vec()).normalized()) + pose);
    if( sameside(r1,a,b,c) & sameside(r1,b,a,c) & sameside(r1,c,a,b))
    {
      rays_local.x=r1[0];rays_local.y=r1[1];rays_local.z=r1[2];
      if ((r1-pose).norm()<=2)
        coverage.points.push_back(rays_local);
    }    
    pline = ParametrizedLine<double,3>::Through(pose,pose+(rot2.vec()));
    intersection1 = pline.intersection( focalPlane ) ;
    r2=(intersection1*( (rot2.vec()).normalized()) + pose);
    if( sameside(r2,a,b,c) & sameside(r2,b,a,c) & sameside(r2,c,a,b))
    {
      rays_local.x=r2[0];rays_local.y=r2[1];rays_local.z=r2[2];
      if ((r2-pose).norm()<=2)
        coverage.points.push_back(rays_local);
    }    
    pline = ParametrizedLine<double,3>::Through(pose,pose+(rot3.vec()));
    intersection1 = pline.intersection( focalPlane ) ;
    r3=(intersection1*( (rot3.vec()).normalized()) + pose);
    if( sameside(r3,a,b,c) & sameside(r3,b,a,c) & sameside(r3,c,a,b))
    {
      rays_local.x=r3[0];rays_local.y=r3[1];rays_local.z=r3[2];
      if ((r3-pose).norm()<=2)  
       coverage.points.push_back(rays_local);
    }    
    pline = ParametrizedLine<double,3>::Through(pose,pose+(rot4.vec()));
    intersection1 = pline.intersection( focalPlane ) ;
    r4=(intersection1*( (rot4.vec()).normalized()) + pose);
    if( sameside(r4,a,b,c) & sameside(r4,b,a,c) & sameside(r4,c,a,b))
    {
      rays_local.x=r4[0];rays_local.y=r4[1];rays_local.z=r4[2];
      if ( (r4-pose).norm()<=2)
        coverage.points.push_back(rays_local);
    }    
  }
  coverage.color.r=1;coverage.color.a=1;
  camera_rays.color.b=1;camera_rays.color.a=1;
  pub_camera_rays.publish(camera_rays);
  pub_coverage.publish(coverage);
}

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
    if (error<0.3)
    {
      waypoint_msg.erase(waypoint_msg.begin()+i);
    }
    i++;
  }
  intersect(curr_pose);
  pub_waypointlist.publish(waypoint_msg);
}
int main (int argc, char** argv)
{
  Markers marker; 
  ros::init (argc, argv, "mesh_slicing");
  ros::NodeHandle nh;
  //publishers
  ros::Publisher  pub_mesh_marker= nh.advertise<visualization_msgs::Marker> ("mesh_marker", 1);
  ros::Publisher  pub_plane= nh.advertise<visualization_msgs::Marker> ("plane_marker", 1);
  ros::Publisher  pub_waypoints= nh.advertise<visualization_msgs::Marker> ("waypoint_marker", 1);
  ros::Publisher  pub_cylinder= nh.advertise<visualization_msgs::Marker> ("cylinder_marker", 1);
  ros::Publisher  pub_cylinderlines= nh.advertise<visualization_msgs::Marker> ("cylinder_lines", 1);
  pub_coverage=nh.advertise<visualization_msgs::Marker> ("coverage", 1);
  pub_camera_rays= nh.advertise<visualization_msgs::Marker> ("camera_rays", 1);
  pub_waypointlist= nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("waypoints", 1);  
  //subscribers
  ros::Subscriber odometry_sub = nh.subscribe<nav_msgs::Odometry>("quad_sim_sysid_node/pose", 1, GetOdom);
  //reading stl file
  Filereader file;
  string path=ros::package::getPath("koptplanner")+"/src/mesh.txt";
  file.read(path,mesh_coor);

  marker.triangle_list(mesh_marker);
  marker.points(waypoint_marker);
  marker.points(local_waypoints);
  marker.points(local_path);
  marker.cylinder(cylinder_marker);
  marker.line_list(cylinder_lines);
  marker.line_list(camera_rays);
  marker.points(coverage);
  // marker.arraypoints(path_vec);
  std_msgs::ColorRGBA mesh_color,plane_color,waypoint_color;
  geometry_msgs::Point mesh_point,plane_points,waypoint;
  geometry_msgs::Pose plocal;
  //define mesh color
  mesh_color.r =0; mesh_color.g=0.5;mesh_color.b=0.5; mesh_color.a=0.3;
  plane_color.r=1;plane_color.a=0.5;  plane_color.b=0;plane_color.g=0;
  waypoint_color.r=0;waypoint_color.g=0;waypoint_color.b=1;waypoint_color.a=1;
  //assigning each triangle of mesh to triangle list marker
  for(int i=0;i<mesh_coor.size();i++)
  {

    mesh_point.x = mesh_coor[i][0];
    mesh_point.y = mesh_coor[i][1];
    mesh_point.z = mesh_coor[i][2];  
    // z min and max
    if(mesh_point.z<z_min)
    {
      z_min=mesh_point.z;
    }
    if(mesh_point.z>z_max)
    {
      z_max=mesh_point.z;
    }
    //y min and max
    if(mesh_point.y<y_min)
    {
      y_min=mesh_point.y;
    }
    if(mesh_point.y>y_max)
    {
      y_max=mesh_point.y;
    }
    // x min and max
    if(mesh_point.x<x_min)
    {
      x_min=mesh_point.x;
    }
    if(mesh_point.x>x_max)
    {
      x_max=mesh_point.x;
    }
    mesh_marker.points.push_back(mesh_point);
    // marker.add_point(mesh_marker,mesh_point,mesh_color);
  }
  mesh_marker.color=mesh_color;

  // these points are to create a slicing plane which will be shifted in steps
  // not for simple planes like z=0 but for oriented planes
  Vector3d p1(-10,10,1);
  Vector3d p2(0,0,1);
  Vector3d p3(10,10,1); // these are to create z=0 plane

  // plane, it passes through 3 points
  Hyperplane<double,3> focalPlane = Hyperplane<double,3>::Through( p1,p2,p3 );
  // ax +by+cz=d
  std::cout << focalPlane.coeffs()[0]<<" "<<focalPlane.coeffs()[1]<<" "<<focalPlane.coeffs()[2]<<" "<<focalPlane.coeffs()[3] << std::endl;
  std::cout<< "x_max::"<<x_max<<","<<"x_min::"<<x_min<<std::endl;
  std::cout<< "y_max::"<<y_max<<","<<"y_min::"<<y_min<<std::endl;
  std::cout<< "z_max::"<<z_max<<","<<"z_min::"<<z_min<<","<<"no of traingles"<<(mesh_marker.points.size()/3)<<std::endl;
  steps=10;
  step_height=(z_max-z_min)/10;
  bool above, below,intersect;
  for(int i=0;i<steps;i++)
  {
    focalPlane.coeffs()[3]=(z_min+(i*step_height)+step_height/2);
    marker.triangle_list(planes);
    //create planes ie two triangles form a polygon
    plane_points.x=-abs(1.5*x_min);  plane_points.y=-abs(1.5*y_min);  plane_points.z=focalPlane.coeffs()[3];planes.points.push_back(plane_points);
    plane_points.x= abs(1.5*x_max);  plane_points.y=-abs(1.5*y_min);  plane_points.z=focalPlane.coeffs()[3];planes.points.push_back(plane_points);
    plane_points.x= abs(1.5*x_max);  plane_points.y= abs(1.5*y_max);  plane_points.z=focalPlane.coeffs()[3];planes.points.push_back(plane_points);
   
    plane_points.x= abs(1.5*x_max);  plane_points.y= abs(1.5*y_max);  plane_points.z=focalPlane.coeffs()[3];planes.points.push_back(plane_points);
    plane_points.x=-abs(1.5*x_min);  plane_points.y= abs(1.5*y_max);  plane_points.z=focalPlane.coeffs()[3];planes.points.push_back(plane_points);
    plane_points.x=-abs(1.5*x_min);  plane_points.y=-abs(1.5*y_min);  plane_points.z=focalPlane.coeffs()[3];planes.points.push_back(plane_points);

    Vector3d i1,i2;
    local_waypoints.points.clear();
    for(int j=0;j<mesh_marker.points.size();j=j+3)
    {
      // std::cout<<mesh_marker.points.at(j).z<<std::endl;
      above=((mesh_marker.points.at(j).z>=focalPlane.coeffs()[3]) | (mesh_marker.points.at(j+1).z>=focalPlane.coeffs()[3]) | (mesh_marker.points.at(j+2).z>=focalPlane.coeffs()[3]));
      below=((mesh_marker.points.at(j).z<focalPlane.coeffs()[3]) | (mesh_marker.points.at(j+1).z<focalPlane.coeffs()[3]) | (mesh_marker.points.at(j+2).z<focalPlane.coeffs()[3]));
      intersect=above&below;
      
      if(intersect)
      { 

        Vector3d tri1(mesh_marker.points.at(j).x,mesh_marker.points.at(j).y,mesh_marker.points.at(j).z);
        Vector3d tri2(mesh_marker.points.at(j+1).x,mesh_marker.points.at(j+1).y,mesh_marker.points.at(j+1).z);
        Vector3d tri3(mesh_marker.points.at(j+2).x,mesh_marker.points.at(j+2).y,mesh_marker.points.at(j+2).z);
          //3c1 for below+ 3c1 for above cominations or possibilities
        if(mesh_marker.points.at(j).z>=focalPlane.coeffs()[3])//1 is above
        { 
          if(mesh_marker.points.at(j+1).z>=focalPlane.coeffs()[3])//2 is above
          {// then 3 is below
           
            ParametrizedLine<double,3> pline1 = ParametrizedLine<double,3>::Through(tri1,tri3);
            ParametrizedLine<double,3> pline2 = ParametrizedLine<double,3>::Through(tri2,tri3);
            double intersection1 = pline1.intersection( focalPlane ) ;
            double intersection2 = pline2.intersection( focalPlane ) ;
            i1=(intersection1*(tri3-tri1).normalized()+tri1);
            // std::cout <<"focalPlane" <<focalPlane.coeffs()[0]<<" "<<focalPlane.coeffs()[1]<<" "<<focalPlane.coeffs()[2]<<" "<<focalPlane.coeffs()[3] << std::endl;

            // std::cout<<i1[0]<<","<<i1[1]<<","<<i1[2]<<std::endl;
            waypoint.x=i1[0];waypoint.y=i1[1];waypoint.z=i1[2]; waypoint_marker.points.push_back(waypoint);
            i2=(intersection2*(tri3-tri2).normalized()+tri2);
            waypoint.x=i2[0];waypoint.y=i2[1];waypoint.z=i2[2]; waypoint_marker.points.push_back(waypoint);
            // std::cout<<i2[0]<<","<<i2[1]<<","<<i2[2]<<std::endl;
          }
          else
          {// 2 is below
            if(mesh_marker.points.at(j+2).z>=focalPlane.coeffs()[3])// 3 is above
            {
              ParametrizedLine<double,3> pline1 = ParametrizedLine<double,3>::Through(tri1,tri2);
              ParametrizedLine<double,3> pline2 = ParametrizedLine<double,3>::Through(tri3,tri2);
              double intersection1 = pline1.intersection( focalPlane ) ;
              double intersection2 = pline2.intersection( focalPlane ) ;
              i1=(intersection1*(tri2-tri1).normalized()+tri1);i2=(intersection2*(tri2-tri3).normalized()+tri3);
              // std::cout <<"focalPlane" <<focalPlane.coeffs()[0]<<" "<<focalPlane.coeffs()[1]<<" "<<focalPlane.coeffs()[2]<<" "<<focalPlane.coeffs()[3] << std::endl;
              // std::cout<<i1[0]<<","<<i1[1]<<","<<i1[2]<<std::endl;
              waypoint.x=i1[0];waypoint.y=i1[1];waypoint.z=i1[2]; waypoint_marker.points.push_back(waypoint);
              waypoint.x=i2[0];waypoint.y=i2[1];waypoint.z=i2[2]; waypoint_marker.points.push_back(waypoint);
            }
            else
            {//3 is below
              ParametrizedLine<double,3> pline1 = ParametrizedLine<double,3>::Through(tri1,tri3);
              ParametrizedLine<double,3> pline2 = ParametrizedLine<double,3>::Through(tri1,tri2);
              double intersection1 = pline1.intersection( focalPlane ) ;
              double intersection2 = pline2.intersection( focalPlane ) ;
              i1=(intersection1*(tri3-tri1).normalized()+tri1);i2=(intersection2*(tri2-tri1).normalized()+tri1);
              // std::cout <<"focalPlane" <<focalPlane.coeffs()[0]<<" "<<focalPlane.coeffs()[1]<<" "<<focalPlane.coeffs()[2]<<" "<<focalPlane.coeffs()[3] << std::endl;
              // std::cout<<i1[0]<<","<<i1[1]<<","<<i1[2]<<std::endl;
              waypoint.x=i1[0];waypoint.y=i1[1];waypoint.z=i1[2]; waypoint_marker.points.push_back(waypoint);
              waypoint.x=i2[0];waypoint.y=i2[1];waypoint.z=i2[2]; waypoint_marker.points.push_back(waypoint);
              
            }
          }
        }
        else// 1 is below
        {
          if(mesh_marker.points.at(j+1).z<focalPlane.coeffs()[3])//2 is below
          {// then 3 is above
            ParametrizedLine<double,3> pline1 = ParametrizedLine<double,3>::Through(tri3,tri1);
            ParametrizedLine<double,3> pline2 = ParametrizedLine<double,3>::Through(tri3,tri2);
            double intersection1 = pline1.intersection( focalPlane ) ;
            double intersection2 = pline2.intersection( focalPlane ) ;
            i1=(intersection1*(tri1-tri3).normalized()+tri3);i2=(intersection2*(tri2-tri3).normalized()+tri3);
            // std::cout<<i1<<","<<i2<<std::endl;          
            waypoint.x=i1[0];waypoint.y=i1[1];waypoint.z=i1[2]; waypoint_marker.points.push_back(waypoint);
            waypoint.x=i2[0];waypoint.y=i2[1];waypoint.z=i2[2]; waypoint_marker.points.push_back(waypoint);
          }
          else
          {// 2 is above
            if(mesh_marker.points.at(j+2).z>=focalPlane.coeffs()[3])// 3 is above
            {
              ParametrizedLine<double,3> pline1 = ParametrizedLine<double,3>::Through(tri2,tri1);
              ParametrizedLine<double,3> pline2 = ParametrizedLine<double,3>::Through(tri3,tri1);
              double intersection1 = pline1.intersection( focalPlane ) ;
              double intersection2 = pline2.intersection( focalPlane ) ;  
              i1=(intersection1*(tri1-tri2).normalized()+tri2);i2=(intersection2*(tri1-tri3).normalized()+tri3);
              // std::cout<<i1<<","<<i2<<std::endl;
              waypoint.x=i1[0];waypoint.y=i1[1];waypoint.z=i1[2]; waypoint_marker.points.push_back(waypoint);
              waypoint.x=i2[0];waypoint.y=i2[1];waypoint.z=i2[2]; waypoint_marker.points.push_back(waypoint);
            }
            else
            {//3 is below
              ParametrizedLine<double,3> pline1 = ParametrizedLine<double,3>::Through(tri2,tri1);
              ParametrizedLine<double,3> pline2 = ParametrizedLine<double,3>::Through(tri2,tri3);
              double intersection1 = pline1.intersection( focalPlane ) ;
              double intersection2 = pline2.intersection( focalPlane ) ; 
              i1=(intersection1*(tri1-tri2).normalized()+tri2);i2=(intersection2*(tri3-tri2).normalized()+tri2);
              // std::cout<<i1<<","<<i2<<std::endl;
              waypoint.x=i1[0];waypoint.y=i1[1];waypoint.z=i1[2]; waypoint_marker.points.push_back(waypoint);
              waypoint.x=i2[0];waypoint.y=i2[1];waypoint.z=i2[2]; waypoint_marker.points.push_back(waypoint);
            }
          }
        }
       //end of if_intersect;
      }
    // end of for loop mesh marker points size  
    }
    
    //end of for loop steps 
  }
  //create a cylinder to section the waypoints
  float r;
  for(int j=0;j<waypoint_marker.points.size();j++)  
  {
    waypoint_marker.points.at(j).x=waypoint_marker.points.at(j).x+2*((5-waypoint_marker.points.at(j).x)/sqrt(pow(5,2)+pow(waypoint_marker.points.at(j).x,2)));
    waypoint_marker.points.at(j).y=waypoint_marker.points.at(j).y+2*((5-waypoint_marker.points.at(j).y)/sqrt(pow(5,2)+pow(waypoint_marker.points.at(j).y,2)));
     
  }
  for(int j=0;j<waypoint_marker.points.size();j++)  
  {
    r=sqrt(pow(waypoint_marker.points.at(j).x,2)+pow(waypoint_marker.points.at(j).y,2)+pow(waypoint_marker.points.at(j).z,2));
    float theta=atan2(waypoint_marker.points.at(j).y-5,waypoint_marker.points.at(j).x-5);
    if ((r<(x_max+y_max))&&(theta>0)&&(theta<M_PI/4))
    {
      // std::cout<<theta<<std::endl;
      local_path.points.push_back(waypoint_marker.points.at(j));
    }
  } 
  // cylinder lines
  for (int j=0;j<8;j++)
  {
    mesh_point.x=5;mesh_point.y=5;mesh_point.z=3;
    cylinder_lines.points.push_back(mesh_point);
    mesh_point.x=5+(10*sin(j*(M_PI/4)));mesh_point.y=5+(10*cos(j*(M_PI/4)));
    cylinder_lines.points.push_back(mesh_point);
  }
  path_vec.markers.push_back(local_path);
  std::cout<<path_vec.markers.at(0).points.size()<<std::endl;
  // giving colors to plane and waypoints
  planes.color=plane_color;
  waypoint_marker.color=waypoint_color;
  local_path.color=waypoint_color;
  cylinder_marker.color=waypoint_color;
  cylinder_lines.color=waypoint_color;

  //waypoint msg
  waypoint_msg.header.frame_id = "world"; //point cloud of positions
  waypoint_msg.height = 1;
  waypoint_msg.width= path_vec.markers.at(0).points.size();
  plist.poses.clear();
  for(int j=0;j<local_path.points.size();j++)
  {
    waypoint_msg.points.push_back(pcl::PointXYZ(path_vec.markers.at(0).points.at(j).x,path_vec.markers.at(0).points.at(j).y,path_vec.markers.at(0).points.at(j).z));
    plocal.position.x=path_vec.markers.at(0).points.at(j).x;plocal.position.y=path_vec.markers.at(0).points.at(j).y;plocal.position.z=path_vec.markers.at(0).points.at(j).z;
    plocal.orientation.w=0;plocal.orientation.x=plocal.position.x-5;plocal.orientation.y=plocal.position.y-5;plocal.orientation.z=0;
    plist.poses.push_back(plocal);
  }

  ros::Rate loop_rate(10.0);
  while(ros::ok())
  {  
     pub_mesh_marker.publish(mesh_marker);
     pub_plane.publish(planes);
     pub_waypoints.publish(local_path);
     pub_cylinder.publish(cylinder_marker);
     pub_cylinderlines.publish(cylinder_lines);

     ros::spinOnce();
     loop_rate.sleep();
  }

}
