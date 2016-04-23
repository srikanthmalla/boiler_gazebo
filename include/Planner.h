//author:srikanth malla

#ifndef _PLANNER_H_
#define _PLANNER_H_
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include "Markers.h"
#include <ros/ros.h>
#include <lemon/list_graph.h>
#include <lemon/greedy_tsp.h>

typedef lemon::FullGraph FulGraph;
typedef FulGraph::EdgeIt EdgeIt;
typedef FulGraph::Edge Edge;
typedef FulGraph::NodeIt NodeIt;
typedef FulGraph::Node Node;
typedef FulGraph::EdgeMap<double> LengthMap;
FulGraph g;
LengthMap cost_map(g);
class Planner{
public:
	Planner();
	int increment_update(int x, int length);
	int decrement_update(int x, int length);
	void resize( std::vector <std::vector <float>>& m, unsigned rows, unsigned cols, float value = 0.0  );//not used
	void GetOdom(const nav_msgs::Odometry::ConstPtr &msg); 
	void graph_gen(int tour_length,geometry_msgs::PoseArray waypoints);//not used
	void find_robot_position(geometry_msgs::PoseArray waypoints);
	void find_start_position(geometry_msgs::PoseArray waypoints,geometry_msgs::PoseArray prev_waypoints);// not used
	void waypoint_cb(const geometry_msgs::PoseArray& input);
	void breakthisdown();
	nav_msgs::Odometry curr_pose;
	bool got_pose = false;//move it private
	visualization_msgs::Marker waypoint_marker_msg;
	std::vector<int> tour,prev_toor,new_toor;// final optimal tour
	const int window_size=800;
	int tour_length=window_size;//window size
	int robot_position=4,robot_index;
	geometry_msgs::PoseArray waypoints,prev_waypoints,selec_waypoints;
	int c = 0,cost = 999;
	int huge_cost=100000;

	ros::Publisher pub;
	ros::Publisher pub_waypoint_marker;

private:
    ros::NodeHandle n_;
	std::vector<std::vector<float>> graph;
	ros::Subscriber waypoints_sub,odometry_sub;
};
Planner::Planner()
{
  waypoints_sub =n_.subscribe("waypoints", 1, &Planner::waypoint_cb,this);
  odometry_sub = n_.subscribe<nav_msgs::Odometry>("quad_sim_sysid_node/pose", 1, &Planner::GetOdom,this);
  pub_waypoint_marker = n_.advertise<visualization_msgs::Marker> ("tsp_waypoint", 10);
  pub=n_.advertise<geometry_msgs::PoseArray>("selec_waypoints",10);
    //--- waypoint marker ---//
  Markers marker;
  marker.points(waypoint_marker_msg);
  waypoint_marker_msg.scale.x = 0.2;  waypoint_marker_msg.scale.y = 0.2;  waypoint_marker_msg.scale.z = 0.2;
}
void Planner::breakthisdown()
{  
	int i, j;
	if (waypoints.poses.size()<3)   
    {
      std::cout<<"waiting..."<<std::endl;
      pub.publish(selec_waypoints);
      pub_waypoint_marker.publish(waypoint_marker_msg);
    }
    else
    {
      g.resize(tour_length);
      
      if(waypoints.poses.size()>0)
      {
        for (EdgeIt l(g); l!=lemon::INVALID; ++l)
        { 
          i=g.id(g.u(l)); j=g.id(g.v(l));
          if((i==tour_length-1)|(j==tour_length-1))
          {
            if((i==tour_length-2)| (j==tour_length-2))
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
            cost_map[l] =  sqrt(pow((waypoints.poses.at(i).position.x-waypoints.poses.at(j).position.x),2)+pow((waypoints.poses.at(i).position.y-waypoints.poses.at(j).position.y),2)+(2*pow((waypoints.poses.at(i).position.z-waypoints.poses.at(j).position.z),2)));  
          }
        }
        lemon::GreedyTsp  <LengthMap> tsp_solver(g,cost_map);
        std::cout<<"TSP Cost::"<<tsp_solver.run()<<"\n";
        std::vector<Node> tour_t=tsp_solver.tourNodes();

        // copy planned pl.tour to pl.tour vector
        tour.clear();
        std::cout<<"planned_pl.tour_by_tsp_pl:"<<std::endl;
        for (int i=0;i<tour_t.size();i++)
        { 
          std::cout<<g.id(tour_t.at(i))<<"->";
          tour.push_back(g.id(tour_t.at(i)));
        }
      std::cout<<std::endl;
      }
      prev_toor.clear();
      std::cout<<"previous_tour:"<<std::endl;
      //copy to previous pl.tour
      for (int i=0;i<new_toor.size();i++)
      {  
        std::cout<< new_toor.at(i) <<"->";
        prev_toor.push_back(new_toor.at(i));
      }

      find_robot_position(waypoints);
      int end_pos;
      for(int i=0;i<tour.size();i++)
      {
        if(tour.at(i)==tour_length-1)
        {
          end_pos=i;
        }
      }
      std::cout<<"waypoint_marker_msg"<<std::endl;
      waypoint_marker_msg.points.resize(tour.size());
      waypoint_marker_msg.colors.resize(tour.size());
      std::cout<<"waypoint_marker_msg"<<std::endl;
     
      int x=robot_index;
      new_toor.clear();
      bool flag=0;
      for(i=0;i<tour.size();i++)// finding which direction is the endpoint is
      { if(end_pos>robot_index)
          flag=0;
        else if(end_pos<robot_index)
          flag=1;
        else
          std::cout<<"REPORT ERROR"<<std::endl;
      }

      for( i=0;i<tour.size();i++)
      {
        new_toor.push_back(tour.at(x));     
        if (flag==0)
          x=decrement_update(x,tour_length);
        else 
          x=increment_update(x,tour_length);
      }
      for(i=0;i<int(new_toor.size());i++)
      {
        std::cout<<new_toor[i]<<"->";
      }
      new_toor.erase(new_toor.begin());
      // resizing according to new path
      selec_waypoints.poses.resize(new_toor.size());

      for(i=0;i<int(new_toor.size());i++)
      { 
        selec_waypoints.poses[i]=waypoints.poses.at(new_toor[i]);

      }
      pub.publish(selec_waypoints); // publish new tour as path


      for(i = 0; i < new_toor.size(); i++)
      { 
        waypoint_marker_msg.points.at(i).x = waypoints.poses.at(new_toor.at(i)).position.x;       
        waypoint_marker_msg.points.at(i).y = waypoints.poses.at(new_toor.at(i)).position.y;      
        waypoint_marker_msg.points.at(i).z = waypoints.poses.at(new_toor.at(i)).position.z; 
        if( i==0 |i==1) //pink for heading direction
        {
          waypoint_marker_msg.colors.at(i).r=1;
          waypoint_marker_msg.colors.at(i).g=0;
          waypoint_marker_msg.colors.at(i).b=1;
          waypoint_marker_msg.colors.at(i).a=1;   
        }
        else // cement color
        { 
          waypoint_marker_msg.colors.at(i).r=0.5;
          waypoint_marker_msg.colors.at(i).g=0.5;
          waypoint_marker_msg.colors.at(i).b=0.5;
          waypoint_marker_msg.colors.at(i).a=1;   
        }
      }
      std::cout<<"waypoint_marker_msg"<<std::endl;
      pub_waypoint_marker.publish(waypoint_marker_msg); //publish new tour as marker
    }
}
void Planner::find_start_position(geometry_msgs::PoseArray waypoints,geometry_msgs::PoseArray prev_waypoints)
{ 
  int flag=0;float x,y,z,distance[tour.size()];
  std::cout<<"prev_toor.size()"<<prev_toor.size()<<std::endl;
  robot_index=0;
  if (prev_toor.size()==0)
  {
    x=waypoints.poses.at(0).position.x;
    y=waypoints.poses.at(0).position.y;
    z=waypoints.poses.at(0).position.z;
  }
  else
  {
    x=prev_waypoints.poses.at(prev_toor.at(0)).position.x;
    y=prev_waypoints.poses.at(prev_toor.at(0)).position.y;
    z=prev_waypoints.poses.at(prev_toor.at(0)).position.z;
  }

  for (int i=0;i<tour.size();i++)
  {
   distance[i]= sqrt(pow((waypoints.poses.at(tour.at(i)).position.x-x),2)+pow((waypoints.poses.at(tour.at(i)).position.y-y),2)+ 2*pow((waypoints.poses.at(tour.at(i)).position.z-z),2));
   // cout <<"dist"<<i<<distance[i]<<endl;
  }  
  for (int i=0;i< tour.size();i++)
  {
    if (distance[robot_index]>distance[i])
    { 
      robot_index=i;
    }
  }
  robot_position=tour.at(robot_index);

  std::cout<<"robot_position"<<robot_position<<std::endl;
  std::cout<<"robot_index"<<robot_index<<std::endl;
}

void Planner::find_robot_position(geometry_msgs::PoseArray waypoints)
{int i;robot_index=0;
  float x=curr_pose.pose.pose.position.x, y=curr_pose.pose.pose.position.y, z=curr_pose.pose.pose.position.z, distance[tour_length];
  for (i=0;i<tour.size();i++)
  {
   distance[i]= sqrt(pow((waypoints.poses.at(tour.at(i)).position.x-x),2)+pow((waypoints.poses.at(tour.at(i)).position.y-y),2)+ 2*pow((waypoints.poses.at(tour.at(i)).position.z-z),2));
   // cout <<"dist"<<i<<distance[i]<<endl;
  }
  for (i=0;i< tour_length;i++)
  {
    if (distance[robot_index]>distance[i])
    { robot_index=i;
    }
  }
  robot_position=tour[robot_index];
}
void Planner::graph_gen(int tour_length,geometry_msgs::PoseArray waypoints)
{
  resize(graph,tour_length,tour_length);
  int i,j;
 
  for(i=0;i<tour_length;i++)
  {  for(j=0;j<tour_length;j++)
    {  //make a graph with euqlidean distance as cost 
       graph[i][j]=sqrt(pow((waypoints.poses.at(i).position.x-waypoints.poses.at(j).position.x),2)+pow((waypoints.poses.at(i).position.y-waypoints.poses.at(j).position.y),2)+ 2*pow((waypoints.poses.at(i).position.z-waypoints.poses.at(j).position.z),2));
    }
  }
}
void Planner::GetOdom(const nav_msgs::Odometry::ConstPtr &msg) 
{
  curr_pose = *msg;
  got_pose = true;

  geometry_msgs::Point ps;
  ps.x = curr_pose.pose.pose.position.x;
  ps.y = curr_pose.pose.pose.position.y;
  ps.z = curr_pose.pose.pose.position.z;
}

void Planner::resize( std::vector <std::vector <float>>& m, unsigned rows, unsigned cols, float value )
{
// first, resize all the available columns
unsigned min = (m.size() < rows) ? m.size() : rows;

for (unsigned row = 0; row < min; row++)
{
  m[ row ].resize( cols, value );
}

// next, resize the rows -- adding complete new columns if necessary
m.resize( rows, std::vector <float> ( cols, value ) );
}
int Planner::decrement_update(int x,int length)
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
int Planner::increment_update(int x,int length)
{
  if(x+1==length)
  {
    x=0;
  }
  else
  {
    x=x+1;
  }
  return x;
}

void Planner::waypoint_cb(const geometry_msgs::PoseArray& input)
{ 
  //copy to prev poses
  prev_waypoints.poses.clear();
  std::cout<<"prev_waypoints:"<<std::endl;
  for(int i=0;i<waypoints.poses.size();i++)
  {
    prev_waypoints.poses.push_back(waypoints.poses.at(i));
    std::cout<<prev_waypoints.poses.at(i).position.x<<" "<<prev_waypoints.poses.at(i).position.y<<" "<<prev_waypoints.poses.at(i).position.z<<std::endl;
  }
  if (input.poses.size()<window_size)
  { 
    tour_length=input.poses.size()+2;
    waypoints.poses.resize(tour_length);
    for (int i = 0; i < input.poses.size(); ++i)
    {
      waypoints.poses.at(i)=input.poses.at(i);
    }
  }
  else
  {
    waypoints.poses.resize(tour_length);

    for (int i = 0; i <window_size; ++i)
    { 
      waypoints.poses.at(i)=input.poses.at(i);
    }
  }

    waypoints.poses.at(tour_length-1).position.x=15;
    waypoints.poses.at(tour_length-1).position.y=15; 
    waypoints.poses.at(tour_length-1).position.z=-8;

    waypoints.poses.at(tour_length-2).position.x=curr_pose.pose.pose.position.x;
    waypoints.poses.at(tour_length-2).position.y=curr_pose.pose.pose.position.y; 
    waypoints.poses.at(tour_length-2).position.z=curr_pose.pose.pose.position.z;

  printf ("getting points\n");
}
#endif