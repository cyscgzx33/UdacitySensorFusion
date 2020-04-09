/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of KD-tree
struct Node
{
	std::vector<float> point; // first one is x, second one is y
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(nullptr)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		Node* curr = root;
		int depth = 0;

		while (curr->point.size() > 0)
		{
			if (depth % 2 == 0) // check x split
			{
				if (point[0] > curr->point[0])
					curr = curr->right;
				else
					curr = curr->left;
			}
			else // check y split
			{
				if (point[1] > curr->point[1])
					curr = curr->right;
				else
					curr = curr->left;
			}
			depth++;
		}
		curr = new Node(point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
};




