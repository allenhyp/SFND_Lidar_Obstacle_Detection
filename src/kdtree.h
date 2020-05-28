/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


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
	int dimension;

	KdTree(int dimension)
	: root(NULL)
	{this->dimension = dimension;}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insert(root, point, id, 0);
	}

	void insert(Node *&node, std::vector<float> point, int id, int level)
	{
		if (node == NULL)
		{
			node = new Node(point, id);
		}
		else if (node->point[level % this->dimension] > point[level % this->dimension])
		{
			insert(node->left, point, id, level + 1);
		}
		else
		{
			insert(node->right, point, id, level + 1);
		}
	}

	void search(Node *node, int level, std::vector<int>& ids, std::vector<float>target, float distanceTol)
	{
		if (node == NULL) return;
		bool isWithinBoundary = true;
		for (int i = 0; i < this->dimension; ++i)
			isWithinBoundary |= fabs(node->point[i] - target[i]) <= distanceTol;
		if (isWithinBoundary)
		{
			float distance = 0;
			for (int i = 0; i < this->dimension; ++i)
				distance += (node->point[i] - target[i]) * (node->point[i] - target[i]);
			distance = sqrt(distance);
			if (distance <= distanceTol)
				ids.push_back(node->id);
		}
		if ((target[level % this->dimension] - distanceTol) < node->point[level % this->dimension])
			search(node->left, level + 1, ids, target, distanceTol);
		if ((target[level % this->dimension] + distanceTol) > node->point[level % this->dimension])
			search(node->right, level + 1, ids, target, distanceTol);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search(root, 0, ids, target, distanceTol);
		return ids;
	}
};




