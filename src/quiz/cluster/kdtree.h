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
private:
	void insert_helper(Node** p_node, std::vector<float>& point, int id, int depth)
	{
		if(*p_node == nullptr)
		{
			*p_node = new Node{point, id};
		} else
		{
			if (depth % 2 == 0) // split x-axis
			{
				if(point[0] > (*p_node)->point[0])
				{
					insert_helper(&((*p_node)->right), point, id, depth+1);
				}
				else
				{
					insert_helper(&((*p_node)->left), point, id, depth+1);
				}
			} else // split y-axis
			{
				if(point[1] > (*p_node)->point[1])
				{
					insert_helper(&((*p_node)->right), point, id, depth+1);
				}
				else
				{
					insert_helper(&((*p_node)->left), point, id, depth+1);
				}
			}
		}
	}

public:
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert_helper(&root, point, id, 0);
	}

private:
	bool check_within_box(std::vector<float> &target, std::vector<float> &current, float d)
    {
	    if ((current[0] >= (target[0] - d) && current[0] <= (target[0] + d)) and
            (current[1] >= (target[1] - d) && current[1] <= (target[1] + d)))
        {
	        float distance = sqrt((current[0] - target[0]) * (current[0] - target[0])
            	                + (current[1] - target[1]) * (current[1] - target[1]));
	        if (distance <= d)
                return true;
        }
    }

	void search_helper(std::vector<int>& ids, std::vector<float> &target,
	        float distanceTol, Node* p_node, int depth)
    {
	    if (p_node != nullptr)
        {
            if(check_within_box(target, p_node->point, distanceTol))
            {
                ids.push_back(p_node->id);
                search_helper(ids, target,distanceTol, p_node->right, depth + 1);
                search_helper(ids, target,distanceTol, p_node->left, depth + 1);
            } else
            {
                if( (target[depth%2] - distanceTol) < p_node->point[depth%2])
                    search_helper(ids, target, distanceTol, p_node->left, depth + 1);
                if( (target[depth%2] - distanceTol) > p_node->point[depth%2])
                    search_helper(ids, target, distanceTol, p_node->right, depth + 1);
            }
        }
    }

public:
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(ids, target, distanceTol, root, 0);
		return ids;
	}
	

};




