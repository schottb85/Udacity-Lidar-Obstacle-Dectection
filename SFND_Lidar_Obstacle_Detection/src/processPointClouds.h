// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Segment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};

template<typename PointT>
class IRansacFitObject{
	public:
	IRansacFitObject(){};
	virtual ~IRansacFitObject(){};

	virtual const std::vector<PointT>& getFitPoints() const = 0;
	virtual void fitTo(const std::vector<PointT> & points) = 0;
	virtual float distanceTo(PointT other) = 0;
	virtual int numRequiredFitPoints() = 0;
};

template<typename PointT>
class RansacFitObjectBase : public IRansacFitObject<PointT>{
	public:
	RansacFitObjectBase() : IRansacFitObject<PointT>(){};
	virtual ~RansacFitObjectBase(){};
	virtual const  std::vector<PointT>& getFitPoints() const { return _points; }
	virtual void fitTo(const std::vector<PointT> & points) = 0;
	virtual float distanceTo(PointT other) = 0;
	virtual int numRequiredFitPoints() = 0;

	protected:
	std::vector<PointT> _points;
};

template<typename PointT>
class Line : public RansacFitObjectBase<PointT>{
	float _A; ///< line coefficient x
	float _B; ///< line coefficient y
	float _C; ///< line coefficient constant

	public:
	/// ctor
	Line() : RansacFitObjectBase<PointT>(){}

	/// ctor
	Line(const std::vector<PointT>& points) : RansacFitObjectBase<PointT>(){
		fitTo(points);
	}

	void fitTo(const std::vector<PointT> & points) override{
		if(points.size() != numRequiredFitPoints()){
			throw std::runtime_error("invalid number of points of a RANSAC line fit");
		}
		this->_points = points;

		computeCoefficients(this->_points[0], this->_points[1]);
	}

	int numRequiredFitPoints() override { return 2; }

	float distanceTo(PointT other) override
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
	void computeCoefficients(PointT p1, PointT p2){
		// compute line representation
		// Line formula Ax+By+C=0
		//(y1−y2)x+(x2−x1)y+(x1∗y2−x2∗y1)
		_A = p1.y - p2.y;
		_B = p2.x - p1.x;
		_C = p1.x*p2.y - p2.x*p1.y;
	}
};

template<typename PointT>
class Plane : public RansacFitObjectBase<PointT>{
	float _A; ///< line coefficient x
	float _B; ///< line coefficient y
	float _C; ///< line coefficient z
	float _D; ///< line coefficient constant

	public:
	/// ctor
	Plane() : RansacFitObjectBase<PointT>(){}

	/// ctor
	Plane(const std::vector<PointT>& points) : RansacFitObjectBase<PointT>(){
		fitTo(points);
	}

	void fitTo(const std::vector<PointT> & points) override{
		if(points.size() != numRequiredFitPoints()){
			throw std::runtime_error("invalid number of points of a RANSAC line fit");
		}
		this->_points = points;

		computeCoefficients(this->_points[0], this->_points[1], this->_points[2]);
	}

	int numRequiredFitPoints() override { return 3; }

	float distanceTo(PointT other) override
	{
		float distance = 0.0;
		// d=∣A∗x+B∗y+C∗z+D∣/sqrt(A2+B2+C2).

		distance = fabs(_A * other.x + _B * other.y + _C * other.z + _D);
		distance/= sqrt(_A*_A + _B*_B + _C*_C);
		// \todo: performance improvement possible by normalizing coefficients A,B,C with 1/D
		return distance;
	}
	
	private:
	void computeCoefficients(PointT p1, PointT p2, PointT p3){
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

template<typename PointT>
static std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, IRansacFitObject<PointT>& obj, int maxIterations, float distanceTol);

#endif /* PROCESSPOINTCLOUDS_H_ */