/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

    void insertHelper(Node<PointT>** node, uint depth, PointT point, int id)
	{
		// Tree is empty
        if (*node==NULL)
			*node = new Node<PointT>(point, id);
		else
		{
			// Calculate current dim
			uint cd = depth % 3;// 0 for even (x split), 1 for odd (y split)

			// if (point[cd] < ((*node)->point[cd])) // will select x-component to compare for even, y-component cd is odd
			// 	insertHelper(&((*node)->left), depth+1, point, id);
			// else
			// 	insertHelper(&((*node)->right), depth+1, point, id);

		    if ((cd == 0 && point.x < ((*node)->point.x)) ||
                (cd == 1 && point.y < ((*node)->point.y)) ||
                (cd == 2 && point.z < ((*node)->point.z)))
                insertHelper(&((*node)->left), depth + 1, point, id);
            else
                insertHelper(&((*node)->right), depth + 1, point, id);
		}
	}
		

    void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
	    insertHelper(&root,0,point,id);	
	}


    void searchHelper(PointT target, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids)
	{

		if(node!=NULL)
		{
			// check if in target box range
			if((node->point.x >= (target.x - distanceTol)) && (node->point.x <= (target.x + distanceTol)) \
			&& (node->point.y >= (target.y - distanceTol)) && (node->point.y <= (target.y + distanceTol)) \
			&& (node->point.z >= (target.z - distanceTol)) && (node->point.z <= (target.z + distanceTol)))
			{
				float distance = sqrt(((node->point.x-target.x)*(node->point.x-target.x)) \
				+ ((node->point.y-target.y)*(node->point.y-target.y))
				+ ((node->point.z-target.z)*(node->point.z-target.z)));
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}

			if((target.data[depth%3]-distanceTol) < node->point.data[depth%3])
			{
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
          	if((target.data[depth%3]+distanceTol) > node->point.data[depth%3])
			{	
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};
    



