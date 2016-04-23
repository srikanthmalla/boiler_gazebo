//author: srikanth malla
//reference: https://ravehgonen.wordpress.com/tag/stl-file-format/

#include <ros/ros.h>
#include "Filereader.h"
boiler_gazebo::triangularmesh mesh;
using namespace std;
int main(int argc , char **argv)
{ 
	Filereader file;
 	ros::init(argc, argv, "test");
 	ros::NodeHandle n;
 	file.ReadSTL("/home/aeroscout2/boiler_ws/src/boiler_gazebo/meshes/boiler5000.stl",mesh,false);
 	for(int i=0;i<mesh.tri.size();i++)
 	{
 		std::cout<<mesh.tri.at(i)<<std::endl;
 	}
 	std::cout<<mesh.tri.at(0)<<std::endl;
	std::cout<<"\n"<<"meshsize::"<<mesh.tri.size()<<std::endl;
	return 0;
}