/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	int dimension; ///< split dimension of node in kdTree
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId, int dimension)
	:	point(arr), id(setId), left(NULL), right(NULL), dimension(dimension)
	{}

	~Node(){
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree(){
		delete root;
	}

	void insertHelperRecursive(Node** node, uint depth, const std::vector<float>& point, int id){
		//TODO: fill this function
		// which dimension is split by this node?
		int dimension = depth % point.size(); // x,y,z, x, y,z, ... (for octree), x, y, x, y, ... (for quadtree)

		if(*node == NULL){
			*node = new Node(point, id, dimension);
			return;
		}

		// insert point into next leaf - left/right if respective dimension (based on depth) </>= node's dimension
		Node ** next_leaf = (point[dimension] < (*node)->point[dimension]) ? &(*node)->left : &(*node)->right;

		insertHelperRecursive(next_leaf, depth+1, point, id);

		return;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		// TODO: call insertHelperRecursive
		insertHelperRecursive(&root, 0, point, id);
	}

	float euclideanDistanceSquaredBetween(const std::vector<float>& source, const std::vector<float>& target){
		if(source.size() != target.size()){
			throw std::runtime_error("source and target point dimensions are not equal!");
		}
		float distance = 0.0;
		for(int idim=0; idim< source.size(); ++idim)
		{
			distance += (target[idim] - source[idim])*(target[idim] - source[idim]);
		}
		return distance;
	}

	float euclideanDistanceBetween(const std::vector<float>& source, const std::vector<float>& target){
		return sqrt(euclideanDistanceSquaredBetween(source, target));
	}

	// \todo: improvement: use tolerances when split point exactly matches limits of bounding box
	class AxisAlignedBoundingBox{
		public:
		AxisAlignedBoundingBox(const std::vector<float>& center, float distanceTol) : _center(center), _distanceTol(distanceTol){}

		bool intersectsHalfspace(int idim, float splitValue, bool left){
			if(left){
				return (_center[idim]-_distanceTol - 1e-4< splitValue) ? true : false;
			}
			else{
				return (_center[idim]+_distanceTol + 1e-4> splitValue) ? true : false;
			}
		}

		bool contains(const std::vector<float>& point){
			for(int idim=0; idim<_center.size(); ++idim){
				if(point[idim]>_center[idim] + _distanceTol || point[idim]<_center[idim] - _distanceTol){
					return false;
				}
			}

			return true;
		}

		const std::vector<float> &center(){
			return _center;
		}

		private:
		std::vector<float> _center;
		float _distanceTol;
	};

	// the fundamental idea to speed up search is to find halfspaces which can be neglected, since it is not
	// possible that they contain any points which are closer than distanceTol
	// -> since a hypersphere is not represented by a finite number of limiting corners,
	// we speed up this by using axis-aligned bounding-box overlap checks. If a bbox will contain the point, then it
	// is worth to compute the exact euclidean distance for an exact check
	void searchHelperRecursive(std::vector<int> & ids, Node* node, const std::vector<float>& target, float distanceTol, AxisAlignedBoundingBox& bbox){
		if(node == NULL){
			return;
		}

		// check if bounding box around target intersects with half space in node's split direction
		// if node is within the bounding box, do the exact euclidean distance computation
		if(bbox.contains(node->point)){
			// compute the distance of the current node to the target based on sqrt
			if(euclideanDistanceSquaredBetween(bbox.center(), node->point)< distanceTol*distanceTol){
				ids.push_back(node->id); // the current point lies within the distance
			}	
		}

		// decide if to search left or/and right leaf for potential points within hypersphere
		// to speed up, we kick off irrelevant halfspaces based on cheaper bounding box checks
		int splitDimension = node->dimension;
		bool search_left = bbox.intersectsHalfspace(splitDimension, node->point[splitDimension], true);
		bool search_right = bbox.intersectsHalfspace(splitDimension, node->point[splitDimension], false);

		if(search_left){
			searchHelperRecursive(ids, node->left, target, distanceTol, bbox);
		}
		if(search_right){
			searchHelperRecursive(ids, node->right, target, distanceTol, bbox);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		// basic idea:
		// for the current tree node -> do first a bounding box check in which leaf/or leafs have to searched for
		// -> in case that an intersection target point bounding box with the splitregion is found -> seach in the respective leaf
		// -> in case that the split line crosses the bounding box -> search in both leafs
		// -> in case that the current node lies within the targets bounding box -> do the actual check based on euclidean distance
		AxisAlignedBoundingBox bbox(target, distanceTol);

		searchHelperRecursive(ids, root, target, distanceTol, bbox);

		return ids;
	}
	

};




