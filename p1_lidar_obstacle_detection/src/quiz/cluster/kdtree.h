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

  void insertHelper(Node* &node, std::vector<float> point, int id, size_t depth)
	{
    if(node==nullptr)
    {
			node = new Node(point, id);
    }
		else
		{
			if(point[depth%2] < node->point[depth%2])
      {
				insertHelper(node->left,point,id,depth+1);
      }
			else
      {
				insertHelper(node->right,point,id,depth+1);
      }
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
    insertHelper(root,point,id,0);
	}

  void searchHelper(std::vector<int>& ids, Node* node, const std::vector<float>& target, size_t depth, float distanceTol)
	{
		if (node!=nullptr)
		{
			float nodeX = node->point[0];
			float nodeY = node->point[1];
			float targetX = target[0];
			float targetY = target[1];
			if ((std::fabs(nodeX-targetX)<=distanceTol) && (std::fabs(nodeY-targetY)<=distanceTol))
			{
				if (std::hypot(nodeX-targetX,nodeY-targetY)<=distanceTol)
        {
					ids.push_back(node->id);
        }
			}

			// recursive iterate
			if (target[depth%2]-distanceTol < node->point[depth%2])
				searchHelper(ids,node->left,target,depth+1,distanceTol);
			if (target[depth%2]+distanceTol > node->point[depth%2])
				searchHelper(ids,node->right,target,depth+1,distanceTol);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
    searchHelper(ids,root,target,0,distanceTol);
		return ids;
	}
	

};
