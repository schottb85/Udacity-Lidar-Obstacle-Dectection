/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

class Line{
	float _A; ///< line coefficient x
	float _B; ///< line coefficient y
	float _C; ///< line coefficient constant

	public:
	/// ctor
	Line(pcl::PointXYZ p1, pcl::PointXYZ p2){
		// compute line representation
		// Line formula Ax+By+C=0
		//(y1−y2)x+(x2−x1)y+(x1∗y2−x2∗y1)
		_A = p1.y - p2.y;
		_B = p2.x - p1.x;
		_C = p1.x*p2.x - p2.x-p1.y;
	}

	float distanceTo(pcl::PointXYZ other)
	{
		float distance = 0.0;
 		// Point (x,y)(x,y)(x,y)
 		// Distance d=∣Ax+By+C∣/sqrt(A2+B2)d = |Ax+By+C|/sqrt(A^2+B^2)d=∣Ax+By+C∣/sqrt(A2+B2)

		distance = fabs(_A * other.x + _B * other.y + _C);
		distance/= sqrt(_A*_A + _B*_B);
		// \todo: performance improvement possible by normalizing coefficients A,B with 1/C
		return distance;
	}
};

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	std::unordered_set<int> inliersTmp;

	// For max iterations
	while(maxIterations--){
		inliersTmp.clear();

		// Randomly sample subset and fit line
		while(inliersTmp.size()<2){
			// use a while loop on unordered set to handle case when two identical indices are chosen
			int idx = rand()%(cloud->points.size());
			inliersTmp.insert(idx);
		}

		auto it = inliersTmp.begin();

		Line line(cloud->points[*it], cloud->points[*(it++)]);

		for(int i=0; i<cloud->points.size();++i){
			if(inliersTmp.count(i)>0){
				continue; // line points are always inliers
			}

			// Measure distance between every point and fitted line
			// If distance is smaller than threshold count it as inlier
			if(line.distanceTo(cloud->points[i]) < distanceTol){
				// add to inliers
				inliersTmp.insert(i);
			}
		}

		if(inliersTmp.size() > inliersResult.size()){
			// current line has more inliers than recent line, update final result
			inliersResult = inliersTmp;
		}
	} 

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
