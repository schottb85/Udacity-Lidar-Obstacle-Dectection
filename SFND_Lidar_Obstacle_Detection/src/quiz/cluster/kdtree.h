/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelperRecursive(Node** node, uint depth, const std::vector<float>& point, int id){
		//TODO: fill this function

		if(*node == NULL){
			*node = new Node(point, id);
			return;
		}

		// which dimension is split by this node?
		int dimension = depth % point.size(); // x,y,z, x, y,z, ... (for octree), x, y, x, y, ... (for quadtree)

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

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




