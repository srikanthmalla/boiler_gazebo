//author:: srikanth Malla

#include "Planner.h"
int main(int argc , char **argv)
{ 
  Markers marker;
  ros::init(argc, argv, "heuristic_planner");
  Planner pl;
  ros::Rate loop_rate(10.0);
  while(!pl.got_pose)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  while(ros::ok())
  {  
    pl.breakthisdown();
    ros::spinOnce(); 
    loop_rate.sleep();
  }
  return 0;  
}
