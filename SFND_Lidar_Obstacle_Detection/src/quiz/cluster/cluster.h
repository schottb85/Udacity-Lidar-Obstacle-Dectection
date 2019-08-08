#pragma once


#include <unordered_set>
#include <vector>

class KdTree;


// return when no further unprocessed nearby points are remaining
static void proximity(int point_index, KdTree* tree, float distanceTol, std::vector<int>& cluster, std::unordered_set<int>& processed_points, const std::vector<std::vector<float>>& points){
	cluster.push_back(point_index); // add point to cluster
	processed_points.insert(point_index); // mark point as processed

	std::vector<int> nearby_points = tree->search(points[point_index], distanceTol);

	for(auto near_point : nearby_points){
		if(processed_points.count(near_point) > 0){
			continue; // point already processed
		}
		proximity(near_point, tree, distanceTol, cluster, processed_points, points);
	}
}


static std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<std::vector<int>> clusters;

	std::unordered_set<int> processed_points;

	for(int point_index = 0; point_index<points.size(); ++point_index){
		if(processed_points.count(point_index) > 0){
			continue; // point already processed
		}

		// create new cluster and mark point as processed
		std::vector<int> cluster;
			
		// search for all points in the proximity of the start point recursively
		proximity(point_index, tree, distanceTol, cluster, processed_points, points);

		// add new cluster to list of clusters
		if(cluster.size() >= minSize && cluster.size() <= maxSize){
			clusters.push_back(cluster);
		}
	}
 
	return clusters;
}