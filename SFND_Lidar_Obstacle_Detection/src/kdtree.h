/* \author Aaron Brown */
// Quiz on implementing kd tree


#ifndef KDTREE_H
#define KDTREE_H

#include "./render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<double> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<double> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, uint depth, std::vector<double> point, int id)
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

	void insert(std::vector<double> point, int id)
	{
		insertHelper(&root,0,point, id);
	}

	void searchHelper(std::vector<double> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node!=NULL)
		{

			bool toleranceChecker = true;
			for(int i= 0; i<node->point.size(); i++)
			{
				bool checkOnEachAxis = node->point[i]>=(target[i]-distanceTol) && node->point[i]<=(target[i]+distanceTol);
				toleranceChecker = toleranceChecker && checkOnEachAxis;
				if(!toleranceChecker)
					break;
			}
		
		if(toleranceChecker)
		{
			double x = node->point[0]-target[0];
			double y = node->point[1]-target[1];
			double z = node->point[2]-target[2];
			double distance = sqrt(x*x + y*y + z*z);
			if(distance <=distanceTol)
			{
				ids.push_back(node->id);
			}
		}
		uint depth_c = depth%node->point.size();
		if((target[depth_c]-distanceTol) < node->point[depth_c])
			searchHelper(target,node->left, depth+1, distanceTol,ids);
		if((target[depth_c]+distanceTol) > node->point[depth_c])
			searchHelper(target,node->right, depth+1, distanceTol,ids);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<double> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target,root,0,distanceTol,ids);
		return ids;
	}
	

};
#endif



