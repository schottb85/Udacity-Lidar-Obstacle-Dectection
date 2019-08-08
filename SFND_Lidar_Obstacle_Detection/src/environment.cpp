/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <iterator>
#include <vector>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

template<typename PointT>
void SegmentAndClusterCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, typename pcl::PointCloud<PointT>::Ptr cloud , ProcessPointClouds<PointT>& pointProcessor){
    // use own implementation of RANSAC plane segmentation
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = pointProcessor.SegmentPlane(cloud, 100, 0.2);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = pointProcessor.SegmentOwn(cloud, 100, 0.2);

    renderPointCloud(viewer,segResult.first, "planeCloud", Color(0,1,0));
    //renderPointCloud(viewer,segResult.second, "obstCloud", Color(1,0,0));

    int minSize = 3;
    int maxSize = 2000;
    float clusterTolerance = 0.4; 
    //std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = pointProcessor.Clustering(segResult.second, clusterTolerance, minSize, maxSize);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = pointProcessor.ClusteringOwn(segResult.second, clusterTolerance, minSize, maxSize);

    std::vector<Color> clusterColors = { Color(1,1,0), Color(1,0,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};

    // render clusters
    for(typename std::vector<typename pcl::PointCloud<PointT>::Ptr>::iterator cluster = clusters.begin(); cluster!= clusters.end(); ++cluster){
        int cloud_id = std::distance(clusters.begin(), cluster);
        pointProcessor.numPoints(*cluster); // print the number of points
        //renderPointCloud(viewer, *cluster, "obstacle cluster" + std::to_string(cloud_id), clusterColors[cloud_id%clusterColors.size()]);

        // create a bounding box and visualize
        Box box = pointProcessor.BoundingBox(*cluster);
        renderBox(viewer, box, cloud_id);
    }
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    //renderRays(viewer, lidar->position, cloud);
    //renderPointCloud(viewer,cloud, "name", Color(1,0,0));

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = pointProcessor.SegmentPlane(cloud, 100, 0.2);


    SegmentAndClusterCloud(viewer, cloud, pointProcessor);
 }

template<typename PointT>
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, const typename pcl::PointCloud<PointT>::Ptr& inputCloud, ProcessPointClouds<PointT>& pointProcessor, bool renderFullInputCloud)
{
    float filterRes = 0.2; // in meters
    Eigen::Vector4f minPoint(-10,-10,-3,1);
    Eigen::Vector4f maxPoint(30,10,5,1);

    typename pcl::PointCloud<PointT>::Ptr filterCloud = pointProcessor.FilterCloud(inputCloud, filterRes, minPoint, maxPoint);
    
    if(!renderFullInputCloud){
        renderPointCloud(viewer, filterCloud, "filterCloud");
    }

    SegmentAndClusterCloud(viewer, filterCloud, pointProcessor);
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    const bool renderFullInputCloud = false; // flag to swich between full and filtered point cloud rendering

    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    std::vector<boost::filesystem::path> stream = pointProcessor.streamPcd("../src/sensors/data/pcd/data_1"); // data_2 as alternative

    auto streamIterator = stream.begin();

    while (!viewer->wasStopped ())
    {
        // clear the viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // load pcd and run obstacle detection
        inputCloud = pointProcessor.loadPcd(streamIterator->string());
        // call segementation and clustering for cityBlock setup
        cityBlock(viewer, inputCloud, pointProcessor, renderFullInputCloud);
        // render the input Cloud as well
        if(renderFullInputCloud){
            renderPointCloud(viewer, inputCloud, "inputCloud");
        }

        viewer->spinOnce ();

        // next image - restart from beginning after completing a pull loop
        streamIterator++;
        if(streamIterator == stream.end()){
            streamIterator = stream.begin();
        }

    } 
}