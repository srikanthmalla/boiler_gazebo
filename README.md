## what is this project for? ###
To simulate the coverage planning for boiler like 3D model using gazebo
## How do I setup? ##
```
git clone git@bitbucket.org:castacks/boiler_gazebo.git
```
### Dependecies: ###
[gazebo_examples](https://bitbucket.org/castacks/gazebo_examples ) **(for simulation purpose only)**

## Working with it: ##
Start gazebo simulation (**not needed when working with real robot**)
```
roslaunch boiler_gazebo boiler_gazebo.launch
```
Generage waypoints for coverage of 3d model and start planner
```
rosrun boiler_gazebo coverage
```
To generate and follow the path( **not needed when working with real robot**)
```
roslaunch boiler_gazebo path_tracking.launch 
```

When working with real time robot pose subscription topic should be changed..

### who should i talk to ? ###
Srikanth Malla (smalla@andrew.cmu.edu)

### What Project generated this code? ###
Boiler Inspection