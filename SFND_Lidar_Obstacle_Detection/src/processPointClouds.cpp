// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr plane_points(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstacle_points(new pcl::PointCloud<PointT>);


    for(int i : inliers->indices){
        plane_points->points.push_back(cloud->points[i]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_points);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(plane_points, obstacle_points);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // create a segmentation object
    pcl::SACSegmentation<PointT> seg;

    // set RANSAC-plane segmentation options
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // set the input point cloud
    seg.setInputCloud(cloud);

    // do the segmentation
    seg.segment(*inliers, *coefficients);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
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

static std::unordered_set<int> Ransac(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, IRansacFitObject& obj, int maxIterations, float distanceTol)
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


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Segment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    IRansacFitObject* obj = new Plane();
    std::unordered_set<int> inliers_map = Ransac(cloud, *obj, maxIterations, distanceThreshold);

    for(const auto idx : inliers_map){
        inliers->indices.push_back(idx);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation (own implementation) took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}