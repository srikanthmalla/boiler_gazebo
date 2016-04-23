#include <ros/ros.h>
#include <ros/package.h>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <ros/ros.h>
#include <math.h>
using namespace std;

int main(int argc, char** argv)
{
    vector< vector<double> > csv_values;//tsp output viewpoints
    vector< vector<double> > lpath_values;//sampled viewpoints
    vector< vector<double> > distances;
    //load the tsp plan with coordinates to csv_values
    fstream file(ros::package::getPath("koptplanner")+"/data/latestPath.csv", ios::in);
    if (file)
    {
        typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
        boost::char_separator<char> sep(",");
        string line;

        while (getline(file, line))
        {
            Tokenizer info(line, sep);   // tokenize the line of data
            vector<double> values;

            for (Tokenizer::iterator it = info.begin(); it != info.end(); ++it)
            {
                // convert data into double value, and store
                values.push_back(strtod(it->c_str(), 0));
            }

            // store array of values
            csv_values.push_back(values);
        }
    }
    else
    {
        cerr << "Error: Unable to open file " << argv[1] << endl;
        return -1;
    }
    // display results
    cout.precision(4);
    cout.setf(ios::fixed,ios::floatfield);
    //load the tour lpath_values
    fstream vpfile((ros::package::getPath("koptplanner")+"/src/tour.txt").c_str(), ios::in);
    if (vpfile)
    {
        typedef boost::tokenizer< boost::char_separator<char> > Tokenizer;
        boost::char_separator<char> sep(",");
        string line;

        while (getline(vpfile, line))
        {
            Tokenizer info(line, sep);   // tokenize the line of data
            vector<double> values;

            for (Tokenizer::iterator it = info.begin(); it != info.end(); ++it)
            {
                // convert data into double value, and store
                values.push_back(strtod(it->c_str(), 0));
            }

            // store array of values
            lpath_values.push_back(values);
        }
    }
    else
    {
        cerr << "Error: Unable to open file " << argv[1] << endl;
        return -1;
    }
    // display results
    cout.precision(4);
    cout.setf(ios::fixed,ios::floatfield);
    // for (int i=0;i<lpath_values.size();i++)
    // {
    //   for(int j=0;j<lpath_values[i].size();j++)
    //      cout<<lpath_values[i][j]<<"  ";
    //   cout<<endl;
    // }     
  
    cout<<"latestPathsize:"<<lpath_values.size()<<"x"<<lpath_values[0].size()<<endl;
    cout<<"csv_values_size:"<<csv_values.size()<<"x"<<csv_values[0].size()<<endl;
    int low_val;double dist;
    //writing to file along with 7th column saying which VP belongs to which mesh
    std::ofstream myfile;
    myfile.open(ros::package::getPath("boiler_gazebo")+"/src/latestPath.csv", ios::out| ios::trunc);

    for(int i=0;i<csv_values.size();i++)
    { 
      for (int j=0;j<csv_values[i].size();j++)
      {
      	myfile <<csv_values[i][j]<<",";
      } 
      if(i==csv_values.size()-1)
      {
      	myfile<<lpath_values[0][0]<<"\n";;
      	cout<<lpath_values[0][0]<<endl;
      }
	  else
      {
	    myfile<<lpath_values[int((i+1)/2)][0]<<"\n";;
	    cout<<lpath_values[(i+1)/2][0]<<endl;
	  }
    }
    myfile.close();

}