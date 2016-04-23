/*
author: srikanth Malla
stl file reading, and slicing the mesh into planes (horizontal with the mesh)
*/

#include "Coverage.h"
#include "Greedyplanner.h"
int main (int argc, char** argv)
{
  ros::init (argc, argv, "coverage");
  Coverage cvr;
  Greedyplanner gp;
  ros::Rate loop_rate(10.0);
  while(ros::ok())
  {   
    ros::spinOnce();
    loop_rate.sleep();
  }
}

