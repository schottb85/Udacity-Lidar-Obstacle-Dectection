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

	typename pcl::PointCloud<PointT>::Ptr downsampled_cloud(new pcl::PointCloud<PointT>());

    // do voxel grid point reduction
	pcl::VoxelGrid<PointT> voxelgrid;
	voxelgrid.setInputCloud(cloud);
	voxelgrid.setLeafSize(filterRes,filterRes,filterRes);
	voxelgrid.filter(*downsampled_cloud);

	// do region based filtering
	typename pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>());
	pcl::CropBox<PointT> region(true);
	region.setMin(minPoint);
	region.setMax(maxPoint);
	region.setInputCloud(downsampled_cloud);
	region.filter(*cropped_cloud);

	// remove points on roof using crop_box
	std::vector<int> indices;
	pcl::CropBox<PointT> roof(true);
	roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
	roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
	roof.setInputCloud(cropped_cloud);
	roof.filter(indices);

    std::cout << "number of roof points " << indices.size() << std::endl;

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	for(int point : indices){
		inliers->indices.push_back(point);
	}

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cropped_cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cropped_cloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cropped_cloud;
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



template<typename PointT>
static std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, IRansacFitObject<PointT>& obj, int maxIterations, float distanceTol)
{
	srand(time(NULL));

	std::unordered_set<int> inliersResult; // holding inliers indices for the best RANSAC fit
	typename std::vector<PointT> fitpointsResult; // holding the fit points of the best RANSAC fit

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

		typename std::vector<PointT> fitpoints;
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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentOwn(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    IRansacFitObject<PointT>* obj = new Plane<PointT>();
    std::unordered_set<int> inliers_map = Ransac(cloud, *obj, maxIterations, distanceThreshold);

    for(const auto idx : inliers_map){
        inliers->indices.push_back(idx);
    }

	delete obj;

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
   
    // create a new KdTree instance
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud); // set the original point cloud for clustering

    // vector of PointIncides - each vector entry defines a cluster
    std::vector<pcl::PointIndices> cluster_indices;
   
    // create an instance for extracting euclidean clusters from point cloud
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setMinClusterSize(minSize); // to distinguish noise from real clusters
    ec.setMaxClusterSize(maxSize); // for proper separation of clusters, break up large clusters
    ec.setClusterTolerance(clusterTolerance); // max tolerance for better identifying neighboring points within a cluser
    ec.setSearchMethod(tree);   // speed up neighbor seach using a kdtree instance based on the same point cloud
    ec.setInputCloud(cloud); // set the point cloud to the extraction object as well
    ec.extract(cluster_indices); // fill the vector of cluster_indices from which clusters can be created afterwards

    // fill the clusters based on the grouped indices
    for(const auto& c_indices : cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cluster_point_cloud(new pcl::PointCloud<PointT>);
        for(const auto & point_cloud_idx : c_indices.indices){
            cluster_point_cloud->points.push_back(cloud->points[point_cloud_idx]);
        }
        cluster_point_cloud->width = cluster_point_cloud->points.size();
        cluster_point_cloud->height = 1;
        cluster_point_cloud->is_dense = true;

        clusters.push_back(cluster_point_cloud);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

#include "kdtree.h"
#include "cluster.h"



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringOwn(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	std::vector<std::vector<float>> cloud_points;

	for(const auto& point : cloud->points){
		std::vector<float> coords{point.x, point.y, point.z};
		cloud_points.push_back(coords);
	}
   
    // create a new KdTree instance
	KdTree* tree = new KdTree();
  
    for (int i=0; i<cloud_points.size(); i++){
    	tree->insert(cloud_points[i],i);
	}

  	std::vector<std::vector<int>> cluster_indices = euclideanCluster(cloud_points, tree, clusterTolerance, minSize, maxSize);

    // fill the clusters based on the grouped indices
    for(const auto& c_indices : cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cluster_point_cloud(new pcl::PointCloud<PointT>);
        for(const auto & point_cloud_idx : c_indices){
            cluster_point_cloud->points.push_back(cloud->points[point_cloud_idx]);
        }
        cluster_point_cloud->width = cluster_point_cloud->points.size();
        cluster_point_cloud->height = 1;
        cluster_point_cloud->is_dense = true;

        clusters.push_back(cluster_point_cloud);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

	delete tree;

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