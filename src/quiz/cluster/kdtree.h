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

	~Node()
	{
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

	~KdTree()
	{
		delete root;
	}

    void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		// Tree is empty
        if (*node==NULL)
			*node = new Node(point, id);
		else
		{
			// Calculate current dim
			uint cd = depth % 2;// 0 for even (x split), 1 for odd (y split)

			if (point[cd] < ((*node)->point[cd])) // will select x-component to compare for even, y-component cd is odd
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);
		}
	}
		

    void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
	    insertHelper(&root,0,point,id);	
	}


    void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{

		if(node!=NULL)
		{
			// check if in target box range
			if((node->point[0] >= (target[0]- distanceTol)) && (node->point[0] <= (target[0] + distanceTol)) \
			&& (node->point[1] >= (target[1] - distanceTol)) && (node->point[1] <= (target[1] + distanceTol)))
			{
				float distance = sqrt(((node->point[0]-target[0])*(node->point[0]-target[0])) \
				+ ((node->point[1]-target[1])*(node->point[1]-target[1])));
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}

			// in outside target box, determine left or right child node
			// depending on depth even or odd, check x or y coords to determine branch direction
			if((target[depth%2]-distanceTol) < node->point[depth%2])
			{
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
          	if((target[depth%2]+distanceTol) > node->point[depth%2])
			{	
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};
    



