/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include <vector>

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

class IRansacFitObject{
	public:
	IRansacFitObject(){};
	virtual ~IRansacFitObject(){};

	virtual const std::vector<pcl::PointXYZ>& getFitPoints() const = 0;
	virtual void fitTo(const std::vector<pcl::PointXYZ> & points) = 0;
	virtual float distanceTo(pcl::PointXYZ other) = 0;
	virtual int numRequiredFitPoints() = 0;
};

class RansacFitObjectBase : public IRansacFitObject{
	public:
	RansacFitObjectBase() : IRansacFitObject(){};
	virtual ~RansacFitObjectBase(){};
	virtual const std::vector<pcl::PointXYZ>& getFitPoints() const { return _points; }
	virtual void fitTo(const std::vector<pcl::PointXYZ> & points) = 0;
	virtual float distanceTo(pcl::PointXYZ other) = 0;
	virtual int numRequiredFitPoints() = 0;

	protected:
	std::vector<pcl::PointXYZ> _points;
};


class Line : public RansacFitObjectBase{
	float _A; ///< line coefficient x
	float _B; ///< line coefficient y
	float _C; ///< line coefficient constant

	public:
	/// ctor
	Line() : RansacFitObjectBase(){}

	/// ctor
	Line(const std::vector<pcl::PointXYZ>& points) : RansacFitObjectBase(){
		fitTo(points);
	}

	void fitTo(const std::vector<pcl::PointXYZ> & points) override{
		if(points.size() != numRequiredFitPoints()){
			throw std::runtime_error("invalid number of points of a RANSAC line fit");
		}
		_points = points;

		computeCoefficients(_points[0], _points[1]);
	}

	int numRequiredFitPoints() override { return 2; }

	float distanceTo(pcl::PointXYZ other) override
	{
		float distance = 0.0;
 		// Point (x,y)(x,y)(x,y)
 		// Distance d=∣Ax+By+C∣/sqrt(A2+B2)d = |Ax+By+C|/sqrt(A^2+B^2)d=∣Ax+By+C∣/sqrt(A2+B2)

		distance = fabs(_A * other.x + _B * other.y + _C);
		distance/= sqrt(_A*_A + _B*_B);
		// \todo: performance improvement possible by normalizing coefficients A,B with 1/C
		return distance;
	}
	
	private:
	void computeCoefficients(pcl::PointXYZ p1, pcl::PointXYZ p2){
		// compute line representation
		// Line formula Ax+By+C=0
		//(y1−y2)x+(x2−x1)y+(x1∗y2−x2∗y1)
		_A = p1.y - p2.y;
		_B = p2.x - p1.x;
		_C = p1.x*p2.y - p2.x*p1.y;
	}
};

class Plane : public RansacFitObjectBase{
	float _A; ///< line coefficient x
	float _B; ///< line coefficient y
	float _C; ///< line coefficient z
	float _D; ///< line coefficient constant

	public:
	/// ctor
	Plane() : RansacFitObjectBase(){}

	/// ctor
	Plane(const std::vector<pcl::PointXYZ>& points) : RansacFitObjectBase(){
		fitTo(points);
	}

	void fitTo(const std::vector<pcl::PointXYZ> & points) override{
		if(points.size() != numRequiredFitPoints()){
			throw std::runtime_error("invalid number of points of a RANSAC line fit");
		}
		_points = points;

		computeCoefficients(_points[0], _points[1], _points[2]);
	}

	int numRequiredFitPoints() override { return 3; }

	float distanceTo(pcl::PointXYZ other) override
	{
		float distance = 0.0;
		// d=∣A∗x+B∗y+C∗z+D∣/sqrt(A2+B2+C2).

		distance = fabs(_A * other.x + _B * other.y + _C * other.z + _D);
		distance/= sqrt(_A*_A + _B*_B + _C*_C);
		// \todo: performance improvement possible by normalizing coefficients A,B,C with 1/D
		return distance;
	}
	
	private:
	void computeCoefficients(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3){
		// v1 = p2-p1
		// v2 = p3-p1
		// n = v1 \times v2
		float i = (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y);
		float j = (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z);
		float k = (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x);

		// compute line representation
		// Line formula Ax+By+C=0
		//(y1−y2)x+(x2−x1)y+(x1∗y2−x2∗y1)
		_A = i;
		_B = j;
		_C = k;
		_D = -1.*(i*p1.x + j*p1.y + k*p1.z);
	}
};

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, IRansacFitObject& obj, int maxIterations, float distanceTol)
{
	srand(time(NULL));

	std::unordered_set<int> inliersResult; // holding inliers indices for the best RANSAC fit
	std::vector<pcl::PointXYZ> fitpointsResult; // holding the fit points of the best RANSAC fit

	std::unordered_set<int> inliers;

	// For max iterations
	while(maxIterations--){
		inliers.clear();

		// Randomly sample subset and fit obj
		while(inliers.size()<obj.numRequiredFitPoints()){
			// use a while loop on unordered set to handle case when two identical indices are chosen
			int idx = rand()%(cloud->points.size());
			inliers.insert(idx);
		}

		std::vector<pcl::PointXYZ> fitpoints;
		for(const auto & idx: inliers){
			fitpoints.push_back(cloud->points[idx]);
		}

		obj.fitTo(fitpoints); // compute new coefficients according to fit

		for(int i=0; i<cloud->points.size(); ++i){
			if(inliers.count(i)>0){
				continue; // obj points are always inliers
			}

			// Measure distance between every point and fitted obj
			// If distance is smaller than threshold count it as inlier
			if(obj.distanceTo(cloud->points[i]) < distanceTol){
				// add to inliers
				inliers.insert(i);
			}
		}

		if(inliers.size() > inliersResult.size()){
			// current obj has more inliers than recent obj, update final result
			inliersResult = inliers;
			fitpointsResult = fitpoints;
		}
	} 

	obj.fitTo(fitpointsResult); // set again the best fit to the object for later use int the obj

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	//IRansacFitObject * obj = new Line();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	IRansacFitObject * obj = new Plane();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, *obj, 50, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFitPoints(new pcl::PointCloud<pcl::PointXYZ>());

	// store the fit points in an extra cloud
	for(const auto& point : obj->getFitPoints()){
		cloudFitPoints->points.push_back(point);
	}

	// store the inliers (including fit points) and outliers in separate clouds
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
		renderPointCloud(viewer,cloudFitPoints,"fitpoints",Color(0,0,1));
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
