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

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			uint cd = depth%point.size();
			if(point[cd] < ((*node)->point[cd]))
			{
				insertHelper(&((*node)->left),depth+1, point, id);
			}
			else
			{
				insertHelper(&((*node)->right),depth+1, point, id);
			}
		}
		
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root,0,point, id);
	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node!=NULL)
		{

		
		if(node->point[0]>=(target[0]-distanceTol) && node->point[0]<=(target[0]+distanceTol) && node->point[1]>=(target[1]-distanceTol) && node->point[1]<=(target[1]+distanceTol))
		{
			float x = node->point[0]-target[0];
			float y = node->point[1]-target[1];
			float distance = sqrt(x*x + y*y);
			if(distance <=distanceTol)
			{
				ids.push_back(node->id);
			}
		}
		uint depth_c = depth%2;
		if((target[depth_c]-distanceTol) < node->point[depth_c])
			searchHelper(target,node->left, depth+1, distanceTol,ids);
		if((target[depth_c]+distanceTol) > node->point[depth_c])
			searchHelper(target,node->right, depth+1, distanceTol,ids);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target,root,0,distanceTol,ids);
		return ids;
	}
	

};




