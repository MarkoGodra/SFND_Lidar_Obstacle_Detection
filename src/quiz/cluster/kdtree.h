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
	const uint8_t dimensions{3};

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		insert_helper(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

        searchHelper(&root, target, distanceTol, ids, 0);

		return ids;
	}
	
private:
    void insert_helper(Node** node, uint8_t depth, const std::vector<float>& point, int id)
    {
        if(*node == NULL)
        {
            *node = new Node(point, id);
        }
        else
        {
            const auto comparison_dim = depth % dimensions;
            if(point[comparison_dim] < (*node)->point[comparison_dim])
            {
                // Left subtree
                insert_helper(&((*node)->left), ++depth, point, id);
            }
            else
            {
                // Right subtree
                insert_helper(&((*node)->right), ++depth, point, id);
            }
        }
    }

    bool isPointInTargetBox(const std::vector<float>& target, float distanceTol, const std::vector<float>& point)
    {
        double x_lower = target[0] - distanceTol;
        double x_upper = target[0] + distanceTol;
        double y_lower = target[1] - distanceTol;
        double y_upper = target[1] + distanceTol;
        double z_lower = target[2] - distanceTol;
        double z_upper = target[2] + distanceTol;

        bool ret = false;
        if ((point[0] >= x_lower && point[0] <= x_upper) &&
            (point[1] >= y_lower && point[1] <= y_upper) &&
            (point[2] >= z_lower && point[2] <= z_upper))
        {
            ret = true;
        }

        return ret;
    }

    void searchHelper(Node** root, const std::vector<float>& target, float distanceTol, std::vector<int>& ids, uint8_t depth)
    {
	    if(*root == NULL)
        {
	        return;
        }
	    else
        {
            const auto& currentPoint = (*root)->point;
            const bool isPointInBox = isPointInTargetBox(target, distanceTol, currentPoint);
            if(isPointInBox)
            {
                // Point is in the box, check distance and branch out to both sides
                double d = sqrt((target[0] - currentPoint[0]) * (target[0] - currentPoint[0])
                                + (target[1] - currentPoint[1]) * (target[1] - currentPoint[1])
                                + (target[2] - currentPoint[2]) * (target[2] - currentPoint[2]));

                if(d <= distanceTol)
                {
                    ids.push_back((*root)->id);
                }
            }

            // Check where to branch out (based on depth we know which dim are we comparing)
            const auto comparison_dim = depth % dimensions;

            if(target[comparison_dim] - distanceTol < currentPoint[comparison_dim])
            {
                searchHelper(&((*root)->left), target, distanceTol, ids, depth + 1);
            }

            if(target[comparison_dim] + distanceTol > currentPoint[comparison_dim])
            {
                searchHelper(&((*root)->right), target, distanceTol, ids, depth + 1);
            }
        }
    }
};




