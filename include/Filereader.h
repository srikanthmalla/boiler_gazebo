//author:srikanth malla

#ifndef _FILE_READER_H_
#define _FILE_READER_H_

#include <boost/tokenizer.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <cstdlib>
#include "boiler_gazebo/triangularmesh.h"
#include "boiler_gazebo/trianglewithnormal.h"

using namespace std;
class Filereader
{
public:
	void read(const string& path, vector< vector<double> >& csv_values);
  void ReadCSV(const string& path, vector< vector<double> >& csv_values);
  int ReadSTL(const char *stlFile, boiler_gazebo::triangularmesh& mesh, bool isBinaryFormat);
};
int Filereader::ReadSTL(const char *stlFile, boiler_gazebo::triangularmesh& mesh, bool isBinaryFormat)
{    
    mesh.tri.clear();
    if (!isBinaryFormat) {
        ifstream in(stlFile);
        // if (!in.good()) return 1;
        std::string s0, s1;
        float n0, n1, n2, f0, f1, f2, f3, f4, f5, f6, f7, f8;
        // in.read(title, 80);
        in >> s0;                                // remove solid and title
        if (s0=="solid")
        {
          in >> s0;//for title
        }
        while (!in.eof()) {
            in >> s0;                                // facet || endsolid
            if (s0=="facet") {
                
                in >> s1 >> n0 >> n1 >> n2;            // normal x y z
                in >> s0 >> s1;                        // outer loop
                in >> s0 >> f0 >> f1 >> f2;         // vertex x y z
                in >> s0 >> f3 >> f4 >> f5;         // vertex x y z
                in >> s0 >> f6 >> f7 >> f8;         // vertex x y z
                in >> s0;                             // endloop
                in >> s0;                            // endfacet
                boiler_gazebo::trianglewithnormal triangle;
                triangle.n.x =n0;triangle.n.y =n1;triangle.n.z =n2;
                triangle.p1.x=f0;triangle.p1.y=f1;triangle.p1.z=f2;
                triangle.p2.x=f3;triangle.p2.y=f4;triangle.p2.z=f5;
                triangle.p3.x=f6;triangle.p3.y=f7;triangle.p3.z=f8;

                mesh.tri.push_back(triangle);
            }
            else if (s0=="endsolid") {
                break;
            }
        }
        in.close();
    }
    else {
        FILE *f = fopen(stlFile, "rb");
        if (!f) return 1;
        char title[80];
        int nFaces;
        fread(title, 80, 1, f);
        fread((void*)&nFaces, 4, 1, f);
        float v[12]; // normal=3, vertices=3*3 = 12
        unsigned short uint16;
        // Every Face is 50 Bytes: Normal(3*float), Vertices(9*float), 2 Bytes Spacer
        for (size_t i=0; i<nFaces; ++i) {
            for (size_t j=0; j<12; ++j) {
                fread((void*)&v[j], sizeof(float), 1, f);
            }
            fread((void*)&uint16, sizeof(unsigned short), 1, f); // spacer between successive faces
            boiler_gazebo::trianglewithnormal triangle;
            triangle.n.x =v[0];triangle.n.y =v[1];triangle.n.z =v[2];
            triangle.p1.x=v[3];triangle.p1.y=v[4];triangle.p1.z=v[5];
            triangle.p2.x=v[6];triangle.p2.y=v[7];triangle.p2.z=v[8];
            triangle.p3.x=v[9];triangle.p3.y=v[10];triangle.p3.z=v[11];
            
            mesh.tri.push_back(triangle);
        }
        fclose(f);
    }    
    return 0;
}
void Filereader::ReadCSV(const string& path, vector< vector<double> >& csv_values )
{
    fstream file(path, ios::in);
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
    cerr << "Error: Unable to open file " << "latestPath.csv" << endl;
  }
}
void Filereader::read(const string& path, vector< vector<double> >& csv_values )
{
    fstream file(path, ios::in);
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
	  cerr << "Error: Unable to open file " << "latestPath.csv" << endl;
	}
}

#endif
