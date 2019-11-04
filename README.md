# Udacity-Lidar-Obstacle-Dectection

<img src="https://github.com/schottb85/Udacity-Lidar-Obstacle-Dectection/blob/master/2019-11-04_19h36_43.png" width="700" height="400" />

<img src="https://github.com/schottb85/Udacity-Lidar-Obstacle-Dectection/blob/master/2019-11-04_19h37_05.png" width="700" height="400" />

### Welcome to this Lidar obstacle detection project.

In this project, high resolution sensor data from **LIDAR** data are postprocessed taking into the following steps:

* _RANSAC plane segmentation_ - segment the road surface from the entire LIDAR _point cloud_

* perform an _Euclidean Clustering_ of the environmental point cloud to separate different obstacles from each other

* Implementation of a _kd-tree_ for efficient set up and search within spatially distributed point clouds 

* Downsampling strategies like _voxel grid filtering_, _box cropping_, _clippling_ and removel of ego-car vehicle roof points

The different algorithms are combined and applied to realistic real-world point cloud data of either a:

* urban scenario with different passing cars along a straight road

* urban scenario following a bicycle driving on a curvy road



## Installation

### Windows subsystem for Linux (WSL)

To work best on a windows machine, we recommend to use the _windows subsystem for linux_ (WSL)

Install PCL

* _sudo apt-get install ..._

* 

Clone repository

Build project

* `mkdir build && cd build`

* `cmake ..`

* `make -j5``

where the number specifies the number of cores for compiling the project

Install xming (optional)

For showing graphical content within the WSL environment, on windows side a listener needs to be installed and started

* install e.g. *XMing*

* start XMing

* within WSL: `export DISPLAY=:0`

* run application `./environment`

Export DISPLAY settings


Coding with VSCode within WSL (remote)

`code .` will open visual studio

Install plugins for 'CMake' and C++ coding

Install PCL, C++

The link here is very helpful, 
https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/

A few updates to the instructions above were needed.

* libvtk needed to be updated to libvtk6-dev instead of (libvtk5-dev). The linker was having trouble locating libvtk5-dev while building, but this might not be a problem for everyone.

* BUILD_visualization needed to be manually turned on, this link shows you how to do that,
http://www.pointclouds.org/documentation/tutorials/building_pcl.php

