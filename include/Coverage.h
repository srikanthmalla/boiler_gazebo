//author:srikanth malla
// list of topics and messages and their use
// 

#ifndef _COVERAGE_H_
#define _COVERAGE_H_

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include "Markers.h"
#include "Filereader.h"
#include <geometry_msgs/Point.h>
#include <iostream>
#include <math.h>
#include <ros/package.h> //ros::package::getPath
#include <nav_msgs/Odometry.h>
class Coverage{
public:
  Coverage();//constructor
	bool sameside(Eigen::Vector3d& p1,Eigen::Vector3d& p2,Eigen::Vector3d& a,Eigen::Vector3d& b);
	void intersection_points(int& i,Eigen::Quaterniond& rot1,Eigen::Vector3d& pose);
	void slice(Eigen::Hyperplane<double,3>& focalPlane,Eigen::Vector3d& a,Eigen::Vector3d& b,Eigen::Vector3d& c,Eigen::Vector3d& d, const int j);
	void intersect(const nav_msgs::Odometry& curr_pose,visualization_msgs::Marker& camera_rays,visualization_msgs::Marker& coverage,ros::Publisher& pub_camera_rays,ros::Publisher& pub_coverage, const int n_part);
	void GetOdom(const nav_msgs::Odometry::ConstPtr &msg);
	void create_plane(float x_min,float y_min,float x_max,float y_max,float zcoeff);
	void waypoint_generation(Eigen::Hyperplane<double,3>& focalPlane);
	void readstl(string path);
	void movewp_inside(visualization_msgs::Marker waypoint_marker,int dist);
  void check_uncovered(const int j);
private:
  ros::NodeHandle nh_;
  //publishers
  ros::Publisher  pub_mesh_marker, pub_plane,  pub_waypoints, pub_cylinder,  pub_cylinderlines,pub_covered,pub_uncovered;
  ros::Publisher pub_plist,pub_pzlist,pub_waypointlist,pub_camera_rays,pub_coverage, pub_path_travelled, pub_surf_normals, pub_mesh_lines;
  //subscriber
  ros::Subscriber odometry_sub;
	//markers
  visualization_msgs::Marker mesh_marker,planes, camera_rays, coverage, path_travelled, local_path, waypoint_marker;
  visualization_msgs::MarkerArray path_vec,dummy_path_vec;
  visualization_msgs::Marker cylinder_marker,cylinder_lines,normals,covered_regions,mesh_lines,uncovered_mesh;
  geometry_msgs::PoseArray plist,pzlist;int next_part=1;

  float z_min=0,z_max=0,x_min=0,x_max=0,y_min=0,y_max=0,steps=0,step_height=0;//bounds of stl file
  boiler_gazebo::triangularmesh mesh;//for reading stl file
  std::vector<float> yaw_angles;
  std::vector<int> wp_mesh_ids;
  std::vector<std::vector<int>> pathvec_mesh_ids,flags_mesh;
  std::vector<std::vector<float>> yaw_vector;
  float z_widow_min=0,z_widow_max=0;

  //these are to test check_uncovered
  bool recovery_mode=0;
  visualization_msgs::Marker mid_points,plist_marker,pzlist_marker; 
  ros::Publisher pub_mid;
  std::string mode;
};
void Coverage::check_uncovered(const int j)
{
//used messages wp_mesh_ids, dummy_path_vec, j is id of path_vec to check
//coverage are triangles marker for camera footprint to flag uncovered surface
//uncovered mesh to get uncovered surface in each section

  Eigen::Vector3d p,a,b,c;bool flag=0;
  geometry_msgs::Point local_point; int count=0;
  // std::cout<<coverage.points.size()<<std::endl;
  for(int i=0;i<pathvec_mesh_ids.at(j).size();i=i+1)
  {//for each triangle
    p[0]=(mesh.tri.at(pathvec_mesh_ids.at(j).at(i)).p1.x+mesh.tri.at(pathvec_mesh_ids.at(j).at(i)).p2.x+mesh.tri.at(pathvec_mesh_ids.at(j).at(i)).p1.x)/3;
    p[1]=(mesh.tri.at(pathvec_mesh_ids.at(j).at(i)).p1.y+mesh.tri.at(pathvec_mesh_ids.at(j).at(i)).p2.y+mesh.tri.at(pathvec_mesh_ids.at(j).at(i)).p2.y)/3;
    p[2]=(mesh.tri.at(pathvec_mesh_ids.at(j).at(i)).p1.z+mesh.tri.at(pathvec_mesh_ids.at(j).at(i)).p2.z+mesh.tri.at(pathvec_mesh_ids.at(j).at(i)).p3.z)/3;
    local_point.x=p[0];local_point.y=p[1];local_point.z=p[2];
    // mid_points.points.push_back(local_point);
    if (coverage.points.size()==6)
    {
      for(int k=0;k<coverage.points.size();k=k+3)
      {//for each camera footprint which is two triangles we can go triangle by triangle
        a[0]=coverage.points.at(k).x;a[1]=coverage.points.at(k).y;a[2]=coverage.points.at(k).z;
        b[0]=coverage.points.at(k+1).x;b[1]=coverage.points.at(k+1).y;b[2]=coverage.points.at(k+1).z;
        c[0]=coverage.points.at(k+2).x;c[1]=coverage.points.at(k+2).y;c[2]=coverage.points.at(k+2).z;
    
         // plane, it passes through 3 points
        Eigen::Hyperplane<double,3> focalPlane2 = Eigen::Hyperplane<double,3>::Through( a,b,c );
        float error=(focalPlane2.coeffs()[0]*p[0])+(focalPlane2.coeffs()[1]*p[1])+(focalPlane2.coeffs()[2]*p[2])-(focalPlane2.coeffs()[3]);
        if( sameside(p,a,b,c) & sameside(p,b,a,c) & sameside(p,c,a,b))
        {
          flag=1;//that mesh facet is covered
          flags_mesh[j][i]=1;
          // std::cout<<i<<","<<j<<std::endl;
          // count++;
        }
      }
    }      
  }
}


void Coverage::readstl(string path)
{
	//reading stl file
	Filereader file;
  // file.read(path);
  std::string directory=ros::package::getPath("boiler_gazebo")+"/meshes/boiler5000.stl";
	file.ReadSTL(directory.c_str(),mesh,false);
    geometry_msgs::Point mesh_point;
	 //assigning each triangle of mesh to triangle list marker
	for(int i=0;i<mesh.tri.size();i++)
	{
		mesh_point.x = mesh.tri.at(i).p1.x;		mesh_point.y = mesh.tri.at(i).p1.y;		mesh_point.z = mesh.tri.at(i).p1.z;  mesh_marker.points.push_back(mesh_point);
    mesh_point.x = mesh.tri.at(i).p2.x;   mesh_point.y = mesh.tri.at(i).p2.y;   mesh_point.z = mesh.tri.at(i).p2.z;  mesh_marker.points.push_back(mesh_point);
    mesh_point.x = mesh.tri.at(i).p3.x;   mesh_point.y = mesh.tri.at(i).p3.y;   mesh_point.z = mesh.tri.at(i).p3.z;  mesh_marker.points.push_back(mesh_point);
  }
  //meshlines
  for(int i=0;i<mesh.tri.size();i++)
  {
    mesh_lines.points.push_back(mesh.tri.at(i).p1); mesh_lines.points.push_back(mesh.tri.at(i).p2);
    mesh_lines.points.push_back(mesh.tri.at(i).p2); mesh_lines.points.push_back(mesh.tri.at(i).p3);
    mesh_lines.points.push_back(mesh.tri.at(i).p3); mesh_lines.points.push_back(mesh.tri.at(i).p1);
  } 

  //normals marker
  geometry_msgs::Point normal,normal1;
  for(int i=0;i<mesh.tri.size();i++)
  {
    normal.x=(mesh.tri.at(i).p1.x+mesh.tri.at(i).p2.x+mesh.tri.at(i).p3.x)/3;
    normal.y=(mesh.tri.at(i).p1.y+mesh.tri.at(i).p2.y+mesh.tri.at(i).p3.y)/3;
    normal.z=(mesh.tri.at(i).p1.z+mesh.tri.at(i).p2.z+mesh.tri.at(i).p3.z)/3;
    normal1.x=normal.x+mesh.tri.at(i).n.x;normal1.y=normal.y+mesh.tri.at(i).n.y;normal1.z=normal.z+mesh.tri.at(i).n.z;
    normals.points.push_back(normal);
    normals.points.push_back(normal1);   
  }
  //finding bounds
  for(int i=0;i<mesh_marker.points.size();i++)
  { // z min and max
		if(mesh_marker.points.at(i).z<z_min)
		{
		  z_min=mesh_marker.points.at(i).z;
		}
		if(mesh_marker.points.at(i).z>z_max)
		{
		  z_max=mesh_marker.points.at(i).z;
		}
		//y min and max
		if(mesh_marker.points.at(i).y<y_min)
		{
		  y_min=mesh_marker.points.at(i).y;
		}
		if(mesh_marker.points.at(i).y>y_max)
		{
		  y_max=mesh_marker.points.at(i).y;
		}
		// x min and max
		if(mesh_marker.points.at(i).x<x_min)
		{
		  x_min=mesh_marker.points.at(i).x;
		}
		if(mesh_marker.points.at(i).x>x_max)
		{
		  x_max=mesh_marker.points.at(i).x;
		}
	}
}
Coverage::Coverage()
{   //constructor
	Markers marker;

  nh_.param<std::string>("mode", mode, "sim");
//publishers
	pub_mesh_marker= nh_.advertise<visualization_msgs::Marker> ("mesh_marker", 1);
  pub_plane= nh_.advertise<visualization_msgs::Marker> ("plane_marker", 1);
  pub_waypoints= nh_.advertise<visualization_msgs::Marker> ("waypoint_marker", 1);
	pub_cylinder= nh_.advertise<visualization_msgs::Marker> ("cylinder_marker", 1);
  pub_cylinderlines= nh_.advertise<visualization_msgs::Marker> ("cylinder_lines", 1);

  pub_path_travelled = nh_.advertise<visualization_msgs::Marker> ("path_travelled", 10);
  pub_coverage=nh_.advertise<visualization_msgs::Marker> ("coverage", 1);
  pub_camera_rays= nh_.advertise<visualization_msgs::Marker> ("camera_rays", 1);
  pub_waypointlist= nh_.advertise<geometry_msgs::PoseArray> ("waypoints", 1);
  pub_surf_normals= nh_.advertise<visualization_msgs::Marker> ("normals",1);
  pub_covered= nh_.advertise<visualization_msgs::Marker> ("covered_regions",1);
  pub_mesh_lines=nh_.advertise<visualization_msgs::Marker> ("mesh_lines",1);
  pub_uncovered=nh_.advertise<visualization_msgs::Marker> ("uncovered_mesh",1);
  pub_plist=nh_.advertise<visualization_msgs::Marker>("plist",1);
  pub_pzlist=nh_.advertise<visualization_msgs::Marker>("pzlist",1);

//these are to test check_uncovered
  pub_mid=nh_.advertise<visualization_msgs::Marker>("mid",1);
//subscribers
  if(mode=="real")
    odometry_sub = nh_.subscribe<nav_msgs::Odometry>("lidar_ekf/imu_odom", 1, &Coverage::GetOdom, this);
  else
    odometry_sub = nh_.subscribe<nav_msgs::Odometry>("quad_sim_sysid_node/pose", 1, &Coverage::GetOdom, this);

  cylinder_marker.color.b=1;cylinder_marker.color.a=0.5;

  marker.triangle_list(mesh_marker);
  marker.cylinder(cylinder_marker);
  marker.line_list(camera_rays);
	marker.triangle_list(coverage);
	marker.line_strip(path_travelled);
	marker.points(local_path);
	marker.triangle_list(planes);	
  marker.line_list(normals);
  marker.triangle_list(covered_regions);
	marker.points(waypoint_marker);
	marker.line_list(cylinder_lines);
  marker.line_list(mesh_lines);
  marker.triangle_list(uncovered_mesh);

  marker.points(mid_points);
  marker.points(plist_marker);
  marker.points(pzlist_marker);
  
	// giving colors to plane and waypoints
	std_msgs::ColorRGBA mesh_color,plane_color,waypoint_color;
	//define mesh color
	mesh_color.r =0; mesh_color.g=0.5;mesh_color.b=0.5; mesh_color.a=0.3;//cement
	plane_color.r=1;plane_color.a=0.5;  plane_color.b=0;plane_color.g=0;//red
	waypoint_color.r=0;waypoint_color.g=0;waypoint_color.b=1;waypoint_color.a=1;//blue
	path_travelled.color.r = 0; path_travelled.color.g = 0; path_travelled.color.b = 0; path_travelled.color.a = 1;path_travelled.scale.x = 0.02;path_travelled.scale.y = 0.02;path_travelled.scale.z = 0.02;
	planes.color=plane_color;
	waypoint_marker.color=waypoint_color;
	local_path.color=waypoint_color;
	cylinder_lines.color=waypoint_color;
	mesh_marker.color=mesh_color;
  normals.color=plane_color;
  mesh_lines.color=path_travelled.color;mesh_lines.scale.x=0.01;mesh_lines.scale.y=0.01;mesh_lines.scale.z=0.01;
  covered_regions.color.a=0.7;covered_regions.color.r=1;//camera covered regions
  uncovered_mesh.color.a=1;uncovered_mesh.color.b=1;//uncovered check after covering each section
  
mid_points.color.a=1;mid_points.color.r=1;
pzlist_marker.color.a=1;pzlist_marker.color.g=1;pzlist_marker.color.r=1;
plist_marker.color.a=1;plist_marker.color.b=1;plist_marker.color.r=1;
	// string path=ros::package::getPath("koptplanner")+"/src/mesh.txt";
string path="mesh.txt";
  readstl(path);
	// these points are to create a slicing plane which will be shifted in steps
	// not for simple planes like z=0 but for oriented planes
	Eigen::Vector3d p1(-10,10,1);
	Eigen::Vector3d p2(0,0,1);
	Eigen::Vector3d p3(10,10,1); // these are to create z=0 plane

	// plane, it passes through 3 points
	Eigen::Hyperplane<double,3> focalPlane = Eigen::Hyperplane<double,3>::Through( p1,p2,p3 );
	// ax +by+cz=d
	std::cout << focalPlane.coeffs()[0]<<" "<<focalPlane.coeffs()[1]<<" "<<focalPlane.coeffs()[2]<<" "<<focalPlane.coeffs()[3] << std::endl;
	std::cout<< "x_max::"<<x_max<<","<<"x_min::"<<x_min<<std::endl;
	std::cout<< "y_max::"<<y_max<<","<<"y_min::"<<y_min<<std::endl;
	std::cout<< "z_max::"<<z_max<<","<<"z_min::"<<z_min<<","<<"no of traingles"<<(mesh_marker.points.size()/3)<<std::endl;

  geometry_msgs::Point mesh_point;
  cylinder_marker.pose.position.x=5;
  cylinder_marker.pose.position.y=5;
  // cylinder lines
  for (int j=0;j<8;j++)
  {
  mesh_point.x=cylinder_marker.pose.position.x;mesh_point.y=cylinder_marker.pose.position.y;mesh_point.z=3;
  cylinder_lines.points.push_back(mesh_point);
  mesh_point.x=cylinder_marker.pose.position.x+(2*cylinder_marker.pose.position.x*sin(j*(M_PI/4)));mesh_point.y=(cylinder_marker.pose.position.x)+(2*cylinder_marker.pose.position.x*cos(j*(M_PI/4)));
  cylinder_lines.points.push_back(mesh_point);
  }

  steps=40;
	step_height=(z_max-z_min)/steps;
  z_widow_min=z_min+step_height/4;z_widow_max=z_min+(3*step_height/4);
	for(int i=0;i<steps;i++)
	{
		focalPlane.coeffs()[3]=(z_min+(i*step_height)+step_height/2);
		create_plane(x_min,y_min,x_max,y_max,focalPlane.coeffs()[3]);
		waypoint_generation(focalPlane);
	}
	movewp_inside(waypoint_marker,1);

}
void Coverage::intersect(const nav_msgs::Odometry& curr_pose ,visualization_msgs::Marker& camera_rays,visualization_msgs::Marker& coverage,ros::Publisher& pub_camera_rays,ros::Publisher& pub_coverage, const int n_part)
{ 
//casting the rays on the mesh
  //camera footprint
  camera_rays.points.clear();
  coverage.points.clear();
  geometry_msgs::Point ps,rays_local;
  ps= curr_pose.pose.pose.position;
  geometry_msgs::Quaternion orientation; 
  orientation=curr_pose.pose.pose.orientation;
  double fov=M_PI/9;//20 degrees 
  Eigen::Vector3d ray1(sin((M_PI/2)-(fov/2))*cos(fov/2), sin((M_PI/2)-(fov/2))*sin(fov/2), cos((M_PI/2)-(fov/2)));
  Eigen::Vector3d ray2(sin((M_PI/2)-(fov/2))*cos((2*M_PI)-(fov/2)), sin((M_PI/2)-(fov/2))*sin((2*M_PI)-(fov/2)),cos((M_PI/2)-(fov/2)));
  Eigen::Vector3d ray3(sin((M_PI/2)+(fov/2))*cos(fov/2), sin((M_PI/2)+(fov/2))*sin(fov/2), cos((M_PI/2)+(fov/2)));
  Eigen::Vector3d ray4(sin((M_PI/2)+(fov/2))*cos((2*M_PI)-(fov/2)), sin((M_PI/2)+(fov/2))*sin((2*M_PI)-(fov/2)),cos((M_PI/2)+(fov/2)));
  Eigen::Vector3d a,b,c,pose(ps.x,ps.y,ps.z);Eigen::Quaterniond rot1,rot2,rot3,rot4;
  Eigen::Quaterniond p,q(orientation.w,orientation.x,orientation.y,orientation.z); p.w() = 0;
  p.vec() = ray1; rot1 = q * p * q.inverse(); p.vec() = ray2; rot2 = q * p * q.inverse(); 
  p.vec() = ray3; rot3 = q * p * q.inverse(); p.vec() = ray4; rot4 = q * p * q.inverse();

  int rl=5;//rl is for ray length
  camera_rays.points.push_back(ps);rays_local.x=(rl*rot1.vec()+pose)[0];rays_local.y=(rl*rot1.vec()+pose)[1];rays_local.z=(rl*rot1.vec()+pose)[2];camera_rays.points.push_back(rays_local);
  camera_rays.points.push_back(ps);rays_local.x=(rl*rot2.vec()+pose)[0];rays_local.y=(rl*rot2.vec()+pose)[1];rays_local.z=(rl*rot2.vec()+pose)[2];camera_rays.points.push_back(rays_local);
  camera_rays.points.push_back(ps);rays_local.x=(rl*rot3.vec()+pose)[0];rays_local.y=(rl*rot3.vec()+pose)[1];rays_local.z=(rl*rot3.vec()+pose)[2];camera_rays.points.push_back(rays_local);
  camera_rays.points.push_back(ps);rays_local.x=(rl*rot4.vec()+pose)[0];rays_local.y=(rl*rot4.vec()+pose)[1];rays_local.z=(rl*rot4.vec()+pose)[2];camera_rays.points.push_back(rays_local);
    
  Eigen::Vector3d r1, r2, r3, r4;//intersected points
  for(int i=0;i<mesh_marker.points.size();i=i+3)
  {
    intersection_points(i, rot1, pose);
  }
  for(int i=0;i<mesh_marker.points.size();i=i+3)
  {
    intersection_points( i, rot2, pose);
  }
  for(int i=0;i<mesh_marker.points.size();i=i+3)
  {
    intersection_points( i, rot3, pose);
  }  
  for(int i=0;i<mesh_marker.points.size();i=i+3)
  {
    intersection_points( i, rot4, pose); 
  }
  //if area is less than something then accept else we dont need that coverage

  if (coverage.points.size()==4)
  {    //create polygon from the intersection points
    Eigen::Vector3d a(coverage.points.at(1).x-coverage.points.at(0).x,coverage.points.at(1).y-coverage.points.at(0).y,coverage.points.at(1).z-coverage.points.at(0).z);
    Eigen::Vector3d b(coverage.points.at(1).x-coverage.points.at(3).x,coverage.points.at(1).y-coverage.points.at(3).y,coverage.points.at(1).z-coverage.points.at(3).z);
    float a1=sqrt(abs((a.cross(b)).dot((a.cross(b)))))/2;
    Eigen::Vector3d c(coverage.points.at(0).x-coverage.points.at(1).x,coverage.points.at(0).y-coverage.points.at(1).y,coverage.points.at(0).z-coverage.points.at(1).z);
    Eigen::Vector3d d(coverage.points.at(0).x-coverage.points.at(2).x,coverage.points.at(0).y-coverage.points.at(2).y,coverage.points.at(0).z-coverage.points.at(2).z);
    float a2=sqrt(abs((c.cross(d)).dot((c.cross(d)))))/2;
    // std::cout<<"area::"<<(a1+a2);
    if ((a1+a2)<0.3)
    {
      coverage.points.push_back(coverage.points.at(1));
      coverage.points.push_back(coverage.points.at(2));
      for(int i=0;i<coverage.points.size();i++)
      {
        covered_regions.points.push_back(coverage.points.at(i));
      }
    }
    else
    {
      coverage.points.clear();
    }
  }
  // std::cout<<"camera_footPrint_size::"<<coverage.points.size()<<std::endl;
  // float roll  = atan2(2*orientation.y*orientation.w - 2*orientation.x*orientation.z, 1 - 2*orientation.y*orientation.y - 2*orientation.z*orientation.z);
  // float pitch = atan2(2*orientation.x*orientation.w - 2*orientation.y*orientation.z, 1 - 2*orientation.x*orientation.x - 2*orientation.z*orientation.z);
  // float yaw=asin(2*orientation.x*orientation.y + 2*orientation.z*orientation.w)*180/M_PI;
  coverage.color.r=1;coverage.color.a=1;coverage.color.b=0;coverage.color.g=0;
  camera_rays.color.b=1;camera_rays.color.a=1;
  pub_camera_rays.publish(camera_rays);
  pub_coverage.publish(coverage);
  pub_covered.publish(covered_regions);

  // flag uncovered triangles
  check_uncovered(n_part);


}
void Coverage::GetOdom(const nav_msgs::Odometry::ConstPtr &msg) 
{
  nav_msgs::Odometry curr_pose = *msg;
  geometry_msgs::Point ps;
  ps.x = curr_pose.pose.pose.position.x;
  ps.y = curr_pose.pose.pose.position.y;
  ps.z = curr_pose.pose.pose.position.z; 
  path_travelled.points.push_back(ps);
  pub_path_travelled.publish(path_travelled);

//normal mode
  float localz_min=z_max;
  //find lowest z value
  plist_marker.points.clear();
  pzlist_marker.points.clear();
  geometry_msgs::Point local_pt;
  for (int i = 0; i < plist.poses.size(); i++)
  {
    if(plist.poses.at(i).position.z<localz_min)
    {
      localz_min=plist.poses.at(i).position.z;
    }

  }

  if((pzlist.poses.size()==0)&&(recovery_mode==0))  
  { 
    for (int i = 0; i < plist.poses.size(); i++)
    {
      if ((plist.poses.at(i).position.z<(localz_min+step_height/2))&&(plist.poses.at(i).position.z>(localz_min-step_height/2)))
      {
        pzlist.poses.push_back(plist.poses.at(i));
      }
    }
  }
  int i=0;  
  while(i<plist.poses.size())
  {
    local_pt.x=plist.poses.at(i).position.x;local_pt.y=plist.poses.at(i).position.y;local_pt.z=plist.poses.at(i).position.z;
    plist_marker.points.push_back(local_pt);
    float error=sqrt(pow((plist.poses.at(i).position.x-ps.x),2)+pow((plist.poses.at(i).position.y-ps.y),2)+ pow((plist.poses.at(i).position.z-ps.z),2));
    if (error<0.2)
    {
      plist.poses.erase(plist.poses.begin()+i);
      i--;
    }
    i++;  
  }
  i=0;
  while(i<pzlist.poses.size())
  {
    local_pt.x=pzlist.poses.at(i).position.x;local_pt.y=pzlist.poses.at(i).position.y;local_pt.z=pzlist.poses.at(i).position.z;
    pzlist_marker.points.push_back(local_pt);
    float error=sqrt(pow((pzlist.poses.at(i).position.x-ps.x),2)+pow((pzlist.poses.at(i).position.y-ps.y),2)+ pow((pzlist.poses.at(i).position.z-ps.z),2));
    if (error<0.2)
    {
      pzlist.poses.erase(pzlist.poses.begin()+i);
      i--;
    }
    i++;
  }

  if((plist.poses.size()==0)&&(recovery_mode==0))
  {
    // check_uncovered(next_part-1);
    uncovered_mesh.points.clear();

// recovery mode (fill pzlist with uncovered_mesh.points)
    recovery_mode=1;
    geometry_msgs::Pose plocal;
    for(int j=0;j<flags_mesh.at(next_part-1).size();j++)
    {
      if (flags_mesh.at(next_part-1).at(j)==0)
      {
        std::cout<<"uncovered triangles"<<next_part-1<<","<<j<<std::endl;
        uncovered_mesh.points.push_back(mesh.tri.at(pathvec_mesh_ids.at(next_part-1).at(j)).p1);
        uncovered_mesh.points.push_back(mesh.tri.at(pathvec_mesh_ids.at(next_part-1).at(j)).p2);
        uncovered_mesh.points.push_back(mesh.tri.at(pathvec_mesh_ids.at(next_part-1).at(j)).p3);


        plocal.position.x=dummy_path_vec.markers.at(next_part-1).points.at(j).x;plocal.position.y=dummy_path_vec.markers.at(next_part-1).points.at(j).y;plocal.position.z=dummy_path_vec.markers.at(next_part-1).points.at(j).z;
        plocal.orientation.z=yaw_vector[next_part-1][j];
        pzlist.poses.push_back(plocal);
      }
    }
  }
  if((pzlist.poses.size()==0)&&(recovery_mode==1))
  {
    geometry_msgs::Pose plocal;
    for(int j=0;j<path_vec.markers.at(next_part).points.size();j++)
    {
      plocal.position.x=dummy_path_vec.markers.at(next_part).points.at(j).x;plocal.position.y=dummy_path_vec.markers.at(next_part).points.at(j).y;plocal.position.z=dummy_path_vec.markers.at(next_part).points.at(j).z;
      // plocal.orientation.z=atan2((plocal.position.y-5),(plocal.position.x-5));//giving Psi which is yaw angle;
      plocal.orientation.z=yaw_vector[next_part][j];
      plist.poses.push_back(plocal);
    }
    next_part++;
    // set recovery mode off
    recovery_mode=0;
  }





  intersect(curr_pose,camera_rays,coverage,pub_camera_rays,pub_coverage,next_part-1);
  pub_waypointlist.publish(pzlist);
  //these dooesnot belong here
  pub_mesh_marker.publish(mesh_marker);
  pub_plane.publish(planes);

  local_path=path_vec.markers.at(next_part);
  pub_waypoints.publish(local_path);
  pub_cylinder.publish(cylinder_marker);
  pub_cylinderlines.publish(cylinder_lines);
  pub_surf_normals.publish(normals);
  pub_mesh_lines.publish(mesh_lines);
  pub_plist.publish(plist_marker);
  pub_pzlist.publish(pzlist_marker);
  pub_uncovered.publish(uncovered_mesh);
}

void Coverage::create_plane(float x_min,float y_min,float x_max,float y_max,float zcoeff)
{

  geometry_msgs::Point plane_points;
  //create planes ie two triangles form a polygon
  plane_points.x=-abs(1.5*x_min);  plane_points.y=-abs(1.5*y_min);  plane_points.z=zcoeff;planes.points.push_back(plane_points);
  plane_points.x= abs(1.5*x_max);  plane_points.y=-abs(1.5*y_min);  plane_points.z=zcoeff;planes.points.push_back(plane_points);
  plane_points.x= abs(1.5*x_max);  plane_points.y= abs(1.5*y_max);  plane_points.z=zcoeff;planes.points.push_back(plane_points);
 
  plane_points.x= abs(1.5*x_max);  plane_points.y= abs(1.5*y_max);  plane_points.z=zcoeff;planes.points.push_back(plane_points);
  plane_points.x=-abs(1.5*x_min);  plane_points.y= abs(1.5*y_max);  plane_points.z=zcoeff;planes.points.push_back(plane_points);
  plane_points.x=-abs(1.5*x_min);  plane_points.y=-abs(1.5*y_min);  plane_points.z=zcoeff;planes.points.push_back(plane_points);
}

bool Coverage::sameside(Eigen::Vector3d& p1,Eigen::Vector3d& p2,Eigen::Vector3d& a,Eigen::Vector3d& b)
{
//ref:	http://www.blackpawn.com/texts/pointinpoly/
 Eigen::Vector3d c1=(b-a).cross(p1-a);
 Eigen::Vector3d c2=(b-a).cross(p2-a);

 if ((c1.dot(c2))>=0)
  return true;
 else
  return false;
}
void Coverage::intersection_points( int& i,Eigen::Quaterniond& rot1,Eigen::Vector3d& pose)
{     
  // rot1 is ray that is rotated
    geometry_msgs::Point rays_local;
    Eigen::Vector3d r1, a,b,c;
    a[0]=(mesh_marker.points.at(i).x);a[1]=(mesh_marker.points.at(i).y);a[2]=(mesh_marker.points.at(i).z);
    b[0]=(mesh_marker.points.at(i+1).x);b[1]=(mesh_marker.points.at(i+1).y);b[2]=(mesh_marker.points.at(i+1).z);
    c[0]=(mesh_marker.points.at(i+2).x);c[1]=(mesh_marker.points.at(i+2).y);c[2]=(mesh_marker.points.at(i+2).z);
    // plane, it passes through 3 points
    Eigen::Hyperplane<double,3> focalPlane1 = Eigen::Hyperplane<double,3>::Through( a,b,c );

    Eigen::ParametrizedLine<double,3> pline = Eigen::ParametrizedLine<double,3>::Through(pose,pose+(rot1.vec()));
    double intersection1 = pline.intersection( focalPlane1 ) ;
    r1=(intersection1*( (rot1.vec()).normalized()) + pose);
    if( sameside(r1,a,b,c) & sameside(r1,b,a,c) & sameside(r1,c,a,b))
    {
     //direction constraint
      // std::cout<<intersection1<<std::endl;  
      if(intersection1>=0)  
      {
        rays_local.x=r1[0];rays_local.y=r1[1];rays_local.z=r1[2];
        coverage.points.push_back(rays_local);
      }
    }  
}
void Coverage::slice(Eigen::Hyperplane<double,3>& focalPlane,Eigen::Vector3d& a,Eigen::Vector3d& b,Eigen::Vector3d& c,Eigen::Vector3d& d,const int j)
{
  
  float yaw_y=0,yaw_x=0; bool flag=0;
  //focal plane is the 2d plane that cuts the triangle
  Eigen::Vector3d i1,i2;geometry_msgs::Point waypoint;
  Eigen::ParametrizedLine<double,3> pline1 = Eigen::ParametrizedLine<double,3>::Through(a,b);
  Eigen::ParametrizedLine<double,3> pline2 = Eigen::ParametrizedLine<double,3>::Through(c,d);
  double intersection1 = pline1.intersection( focalPlane ) ;
  double intersection2 = pline2.intersection( focalPlane ) ;
 

  i1=(intersection1*(b-a).normalized()+a);
  waypoint.x=i1[0];waypoint.y=i1[1];waypoint.z=i1[2]; 
  //moving the waypoint inside using the mesh normal
  waypoint.x=waypoint.x+1*((mesh.tri.at(int(j/3)).n.x)/sqrt(pow((mesh.tri.at(int(j/3)).n.x),2)+pow((mesh.tri.at(int(j/3)).n.y),2)));
  waypoint.y=waypoint.y+1*((mesh.tri.at(int(j/3)).n.y)/sqrt(pow((mesh.tri.at(int(j/3)).n.x),2)+pow((mesh.tri.at(int(j/3)).n.y),2)));
  yaw_y=mesh.tri.at(int(j/3)).n.y;yaw_x=mesh.tri.at(int(j/3)).n.x;
  //vector flow to move away from walls
  // while()
  // {
    for(int i=0;i<mesh.tri.size();i++)  
    { 
      float dist1=sqrt(pow(mesh.tri.at(i).p1.x-waypoint.x,2)+pow(mesh.tri.at(i).p1.y-waypoint.y,2)+pow(mesh.tri.at(i).p1.z-waypoint.z,2));
      float dist2=sqrt(pow(mesh.tri.at(i).p2.x-waypoint.x,2)+pow(mesh.tri.at(i).p2.y-waypoint.y,2)+pow(mesh.tri.at(i).p2.z-waypoint.z,2));
      float dist3=sqrt(pow(mesh.tri.at(i).p3.x-waypoint.x,2)+pow(mesh.tri.at(i).p3.y-waypoint.y,2)+pow(mesh.tri.at(i).p3.z-waypoint.z,2));
      if((dist1<0.7)||(dist2<0.7)||(dist3<0.7))
      {
        flag=1;//to not to add those wp
      }
    }
  // }
  if (flag==0)
  {
    yaw_angles.push_back(atan2(-yaw_y,-yaw_x));  
    waypoint_marker.points.push_back(waypoint);
    wp_mesh_ids.push_back(int(j/3));
  }
  else
    flag=0;
  
  i2=(intersection2*(d-c).normalized()+c);
  waypoint.x=i2[0];waypoint.y=i2[1];waypoint.z=i2[2];  
  waypoint.x=waypoint.x+1*((mesh.tri.at(int(j/3)).n.x)/sqrt(pow((mesh.tri.at(int(j/3)).n.x),2)+pow((mesh.tri.at(int(j/3)).n.y),2)));
  waypoint.y=waypoint.y+1*((mesh.tri.at(int(j/3)).n.y)/sqrt(pow((mesh.tri.at(int(j/3)).n.x),2)+pow((mesh.tri.at(int(j/3)).n.y),2)));
  yaw_y=mesh.tri.at(int(j/3)).n.y;yaw_x=mesh.tri.at(int(j/3)).n.x;
  //vector flow
  for(int i=0;i<mesh.tri.size();i++)  
  { 
    float dist1=sqrt(pow(mesh.tri.at(i).p1.x-waypoint.x,2)+pow(mesh.tri.at(i).p1.y-waypoint.y,2)+pow(mesh.tri.at(i).p1.z-waypoint.z,2));
    float dist2=sqrt(pow(mesh.tri.at(i).p2.x-waypoint.x,2)+pow(mesh.tri.at(i).p2.y-waypoint.y,2)+pow(mesh.tri.at(i).p2.z-waypoint.z,2));
    float dist3=sqrt(pow(mesh.tri.at(i).p3.x-waypoint.x,2)+pow(mesh.tri.at(i).p3.y-waypoint.y,2)+pow(mesh.tri.at(i).p3.z-waypoint.z,2));
    if((dist1<0.7)||(dist2<0.7)||(dist3<0.7))
    {
      flag=1;//to not to add those wp
    }
  }
  if (flag==0)
  {
    yaw_angles.push_back(atan2(-yaw_y,-yaw_x));  
    waypoint_marker.points.push_back(waypoint);
    wp_mesh_ids.push_back(int(j/3));
  }
  else
    flag=0;

}
void Coverage::waypoint_generation(Eigen::Hyperplane<double,3>& focalPlane)
{
	  bool above, below,intersect;
    for(int j=0;j<mesh_marker.points.size();j=j+3)
    {
      // std::cout<<mesh_marker.points.at(j).z<<std::endl;
      above=((mesh_marker.points.at(j).z>=focalPlane.coeffs()[3]) | (mesh_marker.points.at(j+1).z>=focalPlane.coeffs()[3]) | (mesh_marker.points.at(j+2).z>=focalPlane.coeffs()[3]));
      below=((mesh_marker.points.at(j).z<focalPlane.coeffs()[3]) | (mesh_marker.points.at(j+1).z<focalPlane.coeffs()[3]) | (mesh_marker.points.at(j+2).z<focalPlane.coeffs()[3]));
      intersect=above&below;
      
      if(intersect)
      { 

        Eigen::Vector3d tri1(mesh_marker.points.at(j).x,mesh_marker.points.at(j).y,mesh_marker.points.at(j).z);
        Eigen::Vector3d tri2(mesh_marker.points.at(j+1).x,mesh_marker.points.at(j+1).y,mesh_marker.points.at(j+1).z);
        Eigen::Vector3d tri3(mesh_marker.points.at(j+2).x,mesh_marker.points.at(j+2).y,mesh_marker.points.at(j+2).z);
          //3c1 for below+ 3c1 for above cominations or possibilities
        if(mesh_marker.points.at(j).z>=focalPlane.coeffs()[3])//1 is above
        { 
          if(mesh_marker.points.at(j+1).z>=focalPlane.coeffs()[3])//2 is above
          {// then 3 is below
            slice(focalPlane,tri1,tri3,tri2,tri3,j);
          }
          else
          {// 2 is below
            if(mesh_marker.points.at(j+2).z>=focalPlane.coeffs()[3])// 3 is above
            {
              slice(focalPlane,tri1,tri2,tri3,tri2,j);
            }
            else
            {//3 is below
              slice(focalPlane,tri1,tri3,tri1,tri2,j);              
            }
          }
        }
        else// 1 is below
        {
          if(mesh_marker.points.at(j+1).z<focalPlane.coeffs()[3])//2 is below
          {// then 3 is above
            slice(focalPlane,tri3,tri1,tri3,tri2,j);
          }
          else
          {// 2 is above
            if(mesh_marker.points.at(j+2).z>=focalPlane.coeffs()[3])// 3 is above
            {
              slice(focalPlane,tri2,tri1,tri3,tri1,j);
            }
            else
            {//3 is below
              slice(focalPlane,tri2,tri1,tri2,tri3,j);
            }
          }
        }
       //end of if_intersect;
      }
    // end of for loop mesh marker points size  
    }
    //end of for loop steps 
}
void Coverage::movewp_inside(visualization_msgs::Marker waypoint_marker,int dist)
{
	geometry_msgs::Pose plocal;
  std::vector<int> local_mesh_ids;
	float r;
  plist.poses.clear();
  std::vector<float>local_yaw;
  // for(int i=0;i<8;i++)
  // {
  //   for(int j=0;j<mesh.tri.size();j++)
  //   {
  //     float x=(mesh.tri.at(j).p1.x+mesh.tri.at(j).p1.x+mesh.tri.at(j).p1.x),y=,z=;
  //   }
  // }

	for(int i=0;i<8;i++)
	{
		local_path.points.clear();
    local_yaw.clear();
    local_mesh_ids.clear();
		for(int j=0;j<waypoint_marker.points.size();j++)  
		{
		  r=sqrt(pow(waypoint_marker.points.at(j).x,2)+pow(waypoint_marker.points.at(j).y,2)+pow(waypoint_marker.points.at(j).z,2));
		  float theta=atan2(waypoint_marker.points.at(j).y- cylinder_marker.pose.position.x ,waypoint_marker.points.at(j).x- cylinder_marker.pose.position.y )+M_PI;
		  if ((r<(x_max+y_max))&&(theta>(i*(M_PI/4)))&&(theta<((i+1)*M_PI/4)))
		  {
		    local_path.points.push_back(waypoint_marker.points.at(j));
        local_yaw.push_back(yaw_angles.at(j));
        local_mesh_ids.push_back(wp_mesh_ids.at(j));
      }
		} 
    yaw_vector.push_back(local_yaw);
		path_vec.markers.push_back(local_path);
    pathvec_mesh_ids.push_back(local_mesh_ids);
		// std::cout<<path_vec.markers.at(i).points.size()<<std::endl;
	}
	plist.poses.clear();
	for(int j=0;j<path_vec.markers.at(0).points.size();j++)
	{
		plocal.position.x=path_vec.markers.at(0).points.at(j).x;plocal.position.y=path_vec.markers.at(0).points.at(j).y;plocal.position.z=path_vec.markers.at(0).points.at(j).z;
		// plocal.orientation.z=atan2((plocal.position.y-5),(plocal.position.x-5));//giving Psi which is yaw angle;
    plocal.orientation.z=yaw_vector[0][j];
    //need to give according to z level
		plist.poses.push_back(plocal);
	}
  dummy_path_vec.markers.clear();
  for(int j=0;j<path_vec.markers.size();j++)
  {
    dummy_path_vec.markers.push_back(path_vec.markers.at(j));
  }

  //setting flags_mesh size
  flags_mesh.resize(dummy_path_vec.markers.size());
  for(int i=0;i<dummy_path_vec.markers.size();i++)
  {
    flags_mesh.at(i).resize(dummy_path_vec.markers.at(i).points.size());
  }
  for(int i=0;i<dummy_path_vec.markers.size();i++)
  {
    for(int j=0;j<dummy_path_vec.markers.at(i).points.size();j++)
    {
        flags_mesh[i][j]=0;
    }
  }

}
#endif