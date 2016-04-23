#include <iostream>
#include <cmath>
#include <tuple>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h> //conversions b/w ros pointcloud and pcl point cloud msgs
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>//for pcl types
#include <pcl_conversions/pcl_conversions.h>
#include <boiler_gazebo/Coorxyz.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <lemon/list_graph.h>
#include <lemon/insertion_tsp.h>
#include <lemon/greedy_tsp.h>
#include <tsp/tsp.h>
using namespace std;
int c = 0,cost = 999;
const int window_size=20;
int tour_length=window_size;//window size
nav_msgs::Odometry curr_pose;
bool got_pose = false;
int huge_cost=100000;
typedef lemon::FullGraph FulGraph;
typedef FulGraph::EdgeIt EdgeIt;
typedef FulGraph::Edge Edge;
typedef FulGraph::NodeIt NodeIt;
typedef FulGraph::Node Node;
typedef FulGraph::EdgeMap<double> LengthMap;
FulGraph g;
LengthMap cost_map(g);
float start[3]={0,3,0}, pre_start[3]={0,3,0};// needed for routng manipulation

int robot_position=4,robot_index;
std::vector<int> tour,prev_tour;// final optimal tour
std::vector<std::vector<float>> waypoints,graph,prev_waypoints;

boiler_gazebo::Coorxyz selec_waypoints;
ros::Publisher pub;
ros::Publisher pub_waypoint_marker, pub_path_travelled;
std::vector<int> new_tour, direction;
visualization_msgs::Marker waypoint_marker_msg, path_travelled;


typedef std::vector <float> row_t;
typedef std::vector <row_t> matrix_t;
void resize( matrix_t& m, unsigned rows, unsigned cols, float value = 0.0 )
{
// first, resize all the available columns
unsigned min = (m.size() < rows) ? m.size() : rows;

for (unsigned row = 0; row < min; row++)
{
  m[ row ].resize( cols, value );
}

// next, resize the rows -- adding complete new columns if necessary
m.resize( rows, row_t( cols, value ) );
}

void GetOdom(const nav_msgs::Odometry::ConstPtr &msg) 
{
  curr_pose = *msg;
  got_pose = true;

  geometry_msgs::Point ps;
  ps.x = curr_pose.pose.pose.position.x;
  ps.y = curr_pose.pose.pose.position.y;
  ps.z = curr_pose.pose.pose.position.z;
  path_travelled.points.push_back(ps);
  pub_path_travelled.publish(path_travelled);


}
void graph_gen()
{
  resize(graph,tour_length,tour_length);
  direction.resize(tour_length);
  int i,j;
 
  for(i=0;i<tour_length;i++)
  {  for(j=0;j<tour_length;j++)
    {  //make a graph with euqlidean distance as cost 
       graph[i][j]=sqrt(pow((waypoints[i][0]-waypoints[j][0]),2)+pow((waypoints[i][1]-waypoints[j][1]),2)+ 2*pow((waypoints[i][2]-waypoints[j][2]),2));
    }
  }
}
int increment_update(int x,int length)
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
int cicular_addition(int a, int b,int max)
{
  if((a+b)>max)
  {
    return a+b-max;
  }
  else
  {
    return a+b;
  }

}

int open_the_loop()
{ 
  int i,j,x,index;


  float d1=0,d2=0,d[tour_length];
  for(j=0;j<tour_length;j++)
  {   x=robot_index;d1=0;d2=0;

    //traveling one way --incrementing from robot index
    for(i=0;i<j;i++)
    {
      d1=d1+graph[tour.at(x)][tour[increment_update(x,tour_length)]];
      x=increment_update(x,tour_length);
      
    };
    //traveling the other way  --decrementing from robot index
    for(i=0;i<tour_length-j-1;i++)
    {
      d2=d2+graph[tour.at(x)][tour[decrement_update(x,tour_length)]];
      x=decrement_update(x,tour_length);
      
    };
    if(d1<d2)
    {
      d[j]=(2*d1)+d2;
      direction[j]=1;//which means incrementing loop has small cost
    }
    else
    {
      d[j]=(2*d2)+d1;
      direction[j]=0;//which means decrementing loop has small cost
    };
  };
  
  index=0;
  for(j=1;j<tour_length;j++)
  { if(d[j]<d[index]) 
    {
      index=j;
    }
  }
return index;
}


void create_new_route(int index)
{   
  new_tour.clear();
  int i=0,c_index,x=robot_index; //c_index is only used to check if the index is either sides of the robot_position
  c_index=cicular_addition(index, robot_index,tour_length-1);//this is because in the open_the_loop function we are considering from the robot_position

  waypoint_marker_msg.points.resize(tour.size());
  waypoint_marker_msg.colors.resize(tour.size());


  for(i = 0; i < tour.size(); i++)
  { 
    waypoint_marker_msg.points.at(i).x = waypoints[tour.at(i)][0];       
    waypoint_marker_msg.points.at(i).y = waypoints[tour.at(i)][1];      
    waypoint_marker_msg.points.at(i).z = waypoints[tour.at(i)][2]; 
    if(( i==c_index )| (i==increment_update(c_index,tour_length)))
    {
      waypoint_marker_msg.colors.at(i).r=1;
      waypoint_marker_msg.colors.at(i).g=0;
      waypoint_marker_msg.colors.at(i).b=1;
      waypoint_marker_msg.colors.at(i).a=1;   
    }
    else
    {
      waypoint_marker_msg.colors.at(i).r=0.5;
      waypoint_marker_msg.colors.at(i).g=0.5;
      waypoint_marker_msg.colors.at(i).b=0.5;
      waypoint_marker_msg.colors.at(i).a=1;   
    }
  }
  pub_waypoint_marker.publish(waypoint_marker_msg);


  if(c_index==robot_index)
  { 
    cout<<"index==robot_index"<<endl;

    if (direction[index]==0)
    { 
      x=robot_index;
      for(i=0;i<tour.size();i++)
      {
        new_tour.push_back(tour.at(x));     
        x=increment_update(x,tour_length);
      }
    }
    else
    {
      x=robot_index;
      for(i=0;i<tour.size();i++)
      {
        new_tour.push_back(tour.at(x));     
        x=decrement_update(x,tour_length);
      }
    }
  }
  else if(decrement_update(tour.at(robot_index),tour_length)==c_index)
  {
    if (direction[index]==0)
    { 
      x=robot_index;
      for(i=0;i<tour_length;i++)
      {
        new_tour.push_back(tour.at(x));     
        x=increment_update(x,tour_length);
      }
    }
    else
    {
      x=robot_index;
      for(i=0;i<tour_length;i++)
      {
        new_tour.push_back(tour.at(x));     
        x=decrement_update(x,tour_length);
      }
    }
  }
  else
  { 
    
    x=robot_index;
    if(direction[index]==0)
    {
      cout<<"direction[index]=0 and c_index<robot_index"<<endl;
      for(i=0;i<tour_length-index;i++)
      {
        new_tour.push_back(tour.at(x));
        x=decrement_update(x,tour_length);          
      }
      
      for(i=0;i<tour_length-index;i++)
      {
        new_tour.push_back(tour.at(x));
        x=increment_update(x,tour_length);          
      }
     
      for(i=0;i<=index;i++)
      {
        new_tour.push_back(tour.at(x));
        x=increment_update(x,tour_length);          
      }      
    }
    else
    {  
      x=robot_index;
      cout<<"direction[index]=1 and c_index<robot_index"<<endl;
      for(i=0;i<index;i++)
      {
       new_tour.push_back(tour.at(x));
       x=increment_update(x,tour_length);  
      }
      for(i=0;i<=index;i++)
      {
       new_tour.push_back(tour.at(x));
       x=decrement_update(x,tour_length);  
      }
      
      for(i=0;i<tour_length-1-index;i++)
      {
       new_tour.push_back(tour.at(x));
       x=decrement_update(x,tour_length);  
      }
    }
  }

  i=0;
  // robot_position=tour_length-1;
  while(i<new_tour.size())
  {
    if(new_tour.at(i)==robot_position)
    {
      new_tour.erase(new_tour.begin()+i);
      // waypoints.erase(waypoints.begin()+i);
      i--;
    }
    i++;
  }
  for(i=0;i<int(new_tour.size());i++)
  {
    cout<<new_tour[i]<<"->";
  }
  // resizing according to new path
  selec_waypoints.x.resize(new_tour.size());
  selec_waypoints.y.resize(new_tour.size());
  selec_waypoints.z.resize(new_tour.size());
  for(i=0;i<int(new_tour.size());i++)
  { 
    selec_waypoints.x[i]=waypoints[new_tour[i]][0];
    selec_waypoints.y[i]=waypoints[new_tour[i]][1];
    selec_waypoints.z[i]=waypoints[new_tour[i]][2];
  }

}

void waypoint_cb(const sensor_msgs::PointCloud2& input)
{ //convert to pcl(if its point cloud sensor msg then it will be converted to point cloud pcl msg) data type
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(input, pcl_pc);
  //from the pcl point cloud to our pcl template <xyz> or <xyzrgb> to do manipulations
  pcl::PointCloud<pcl::PointXYZ> msg;
  pcl::fromPCLPointCloud2(pcl_pc, msg);
  prev_waypoints.clear();
  cout<<"prev_waypoints:"<<endl;
  for(int i=0;i<waypoints.size();i++)
  {
    prev_waypoints.push_back(waypoints.at(i));
    cout<<prev_waypoints[i][0]<<" "<<prev_waypoints[i][1]<<" "<<prev_waypoints[i][2]<<endl;
  }


  if (msg.points.size()<window_size)
  { 
    tour_length=msg.points.size()+1;
    resize( waypoints, tour_length, 3);
    for (int i = 0; i < msg.points.size(); ++i)
    {
      waypoints[i][0]=msg.points[i].x;
      waypoints[i][1]=msg.points[i].y;
      waypoints[i][2]=msg.points[i].z;
    }
  }
  else
  {
    tour_length=window_size+1;
    resize(waypoints, tour_length, 3);
   

    for (int i = 0; i <window_size; ++i)
    { 
      waypoints[i][0]=msg.points[i].x;
      waypoints[i][1]=msg.points[i].y;
      waypoints[i][2]=msg.points[i].z;
    }
  }
    //robot curr position is also included for tour calculation
    // waypoints[tour_length-2][0]=curr_pose.pose.pose.position.x;
    // waypoints[tour_length-2][1]=curr_pose.pose.pose.position.y; 
    // waypoints[tour_length-2][2]=curr_pose.pose.pose.position.z;

    //endpoint is also inclcuded for tour calculation
    waypoints[tour_length-1][0]=-10;
    waypoints[tour_length-1][1]=0; 
    waypoints[tour_length-1][2]=0;
  printf ("getting points\n");
}

// this find robot position is findng nearest node
void find_robot_position()
{int i;robot_index=0;
  float x=curr_pose.pose.pose.position.x, y=curr_pose.pose.pose.position.y, z=curr_pose.pose.pose.position.z, distance[tour_length];
  for (i=0;i<tour_length;i++)
  {
   distance[i]= sqrt(pow((waypoints[tour.at(i)][0]-x),2)+pow((waypoints[tour.at(i)][1]-y),2)+ 2*pow((waypoints[tour.at(i)][2]-z),2));
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
void find_start_position()
{ 
  int flag=0;float x,y,z,distance[tour.size()];
  cout<<"prev_tour.size()"<<prev_tour.size()<<endl;
  robot_index=0;
  if (prev_tour.size()==0)
  {
    x=waypoints[0][0];
    y=waypoints[0][1];
    z=waypoints[0][2];
  }
  else
  {
    x=prev_waypoints[prev_tour.at(0)][0];
    y=prev_waypoints[prev_tour.at(0)][1];
    z=prev_waypoints[prev_tour.at(0)][2];
  }

  for (int i=0;i<tour.size();i++)
  {
   distance[i]= sqrt(pow((waypoints[tour.at(i)][0]-x),2)+pow((waypoints[tour.at(i)][1]-y),2)+ 2*pow((waypoints[tour.at(i)][2]-z),2));
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

  cout<<"robot_position"<<robot_position<<endl;
  cout<<"robot_index"<<robot_index<<endl;
}

int main(int argc , char **argv)
{ 
  ros::init(argc, argv, "heuristic_planner");
  ros::NodeHandle n;
  ros::Subscriber waypoints_sub =n.subscribe("waypoints", 1, waypoint_cb);
  ros::Subscriber odometry_sub = n.subscribe<nav_msgs::Odometry>("quad_sim_sysid_node/pose", 10, GetOdom);
  pub_waypoint_marker = n.advertise<visualization_msgs::Marker> ("tsp_waypoint", 10);
  pub_path_travelled = n.advertise<visualization_msgs::Marker> ("path_travelled", 10);
  pub=n.advertise<boiler_gazebo::Coorxyz>("selec_waypoints",10);
  int i, j;
  //--- waypoint marker ---//
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

  waypoint_marker_msg.scale.x = 0.2;
  waypoint_marker_msg.scale.y = 0.2;
  waypoint_marker_msg.scale.z = 0.2;
  //--path travelled marker ---//

  path_travelled.header.frame_id = "world";
  path_travelled.header.stamp = ros::Time::now();
  path_travelled.ns = "path";
  path_travelled.id = 0;
  path_travelled.action = visualization_msgs::Marker::ADD;
  path_travelled.scale.x = 0.1;
  path_travelled.type = visualization_msgs::Marker::LINE_STRIP; //correct marker type
  path_travelled.color.r = 0; path_travelled.color.g = 0; path_travelled.color.b = 0; path_travelled.color.a = 1;


    //---------------------------TSP --------------------//
  


  ros::Rate loop_rate(10.0);
  while(!got_pose)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  while(n.ok())
  {  
    ros::spinOnce(); 
    g.resize(tour_length);
    
    if(waypoints.size()>0)
    {
      for (EdgeIt l(g); l!=lemon::INVALID; ++l)
      { 
        i=g.id(g.u(l)); j=g.id(g.v(l));
        if((i==tour_length-1)|(j==tour_length-1))
        {
          if((i==robot_position)| (j==robot_position))
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
          cost_map[l] =  sqrt(pow((waypoints[i][0]-waypoints[j][0]),2)+pow((waypoints[i][1]-waypoints[j][1]),2)+(2*pow((waypoints[i][2]-waypoints[j][2]),2)));  
        }
      }
      lemon::GreedyTsp<LengthMap> tsp_solver(g,cost_map);
      std::cout<<"TSP Cost::"<<tsp_solver.run()<<"\n";
      std::vector<Node> tour_t=tsp_solver.tourNodes();

      // copy planned tour to tour vector
      tour.clear();
      cout<<"planned_tour_by_tsp_planner:"<<endl;
      for (int i=0;i<tour_t.size();i++)
      { 
        std::cout<<g.id(tour_t.at(i))<<"->";
        tour.push_back(g.id(tour_t.at(i)));
      }
    std::cout<<std::endl;
    }
    prev_tour.clear();
    cout<<"previous_tour:"<<endl;
    //copy to previous tour
    for (int i=0;i<new_tour.size();i++)
    {  
      std::cout<< new_tour.at(i) <<"->";
      prev_tour.push_back(new_tour.at(i));
    }
    find_start_position();
    cout<<"waypoint_marker_msg"<<endl;
    waypoint_marker_msg.points.resize(tour.size());
    waypoint_marker_msg.colors.resize(tour.size());
    cout<<"waypoint_marker_msg"<<endl;
    for(i = 0; i < tour.size(); i++)
    { 
      waypoint_marker_msg.points.at(i).x = waypoints[tour.at(i)][0];       
      waypoint_marker_msg.points.at(i).y = waypoints[tour.at(i)][1];      
      waypoint_marker_msg.points.at(i).z = waypoints[tour.at(i)][2]; 
      if( i==robot_position ) /////
      {
        waypoint_marker_msg.colors.at(i).r=1;
        waypoint_marker_msg.colors.at(i).g=0;
        waypoint_marker_msg.colors.at(i).b=1;
        waypoint_marker_msg.colors.at(i).a=1;   
      }
      else
      {
        waypoint_marker_msg.colors.at(i).r=0.5;
        waypoint_marker_msg.colors.at(i).g=0.5;
        waypoint_marker_msg.colors.at(i).b=0.5;
        waypoint_marker_msg.colors.at(i).a=1;   
      }
    }
    cout<<"waypoint_marker_msg"<<endl;
    pub_waypoint_marker.publish(waypoint_marker_msg);


   
    int x=robot_index;
    new_tour.clear();
    bool flag=0;
    for(i=0;i<tour.size();i++)// finding which direction is the endpoint is
    { if(tour.at(increment_update(robot_index,tour_length))==tour_length-1)
        flag=0;
      else if(tour.at(decrement_update(robot_index,tour_length))==tour_length-1)
        flag=1;
      else
        cout<<"REPORT ERROR"<<endl;
    }

    for( i=0;i<tour.size();i++)
    {
      new_tour.push_back(tour.at(x));     
      if (flag==0)
        x=decrement_update(x,tour_length);
      else 
        x=increment_update(x,tour_length);
    }
    for(i=0;i<int(new_tour.size());i++)
    {
      cout<<new_tour[i]<<"->";
    }
    // resizing according to new path
    selec_waypoints.x.resize(new_tour.size());
    selec_waypoints.y.resize(new_tour.size());
    selec_waypoints.z.resize(new_tour.size());
    for(i=0;i<int(new_tour.size());i++)
    { 
      selec_waypoints.x[i]=waypoints[new_tour[i]][0];
      selec_waypoints.y[i]=waypoints[new_tour[i]][1];
      selec_waypoints.z[i]=waypoints[new_tour[i]][2];
    }
    // if(waypoints.size()>2)
    // {
    //   //removing fake node from tour
    //   i=0;
    //   while(i<tour.size())
    //   {
    //     if(tour.at(i)==tour_length-1)
    //     {
    //       tour.erase(tour.begin()+i);
    //       i--;
    //     }
    //     i++;
    // }
    // waypoints.erase(waypoints.begin()+tour_length-1);
    // tour_length--;

    // cout<<"tour_after_removing_fake_node or end point:"<<endl;
    // for(i=0;i<int(tour.size());i++)
    // {
    //   cout<<tour.at(i)<<"->";
    // }
    // }
    // std::cout<<std::endl;
    // find_robot_position();
    // graph_gen();
    // int ind=open_the_loop();
    
    pub.publish(selec_waypoints);
    loop_rate.sleep();
  }
  return 0;  
}
