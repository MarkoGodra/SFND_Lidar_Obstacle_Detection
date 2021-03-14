// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

namespace EuclideanCluster
{

void Proximity(const std::vector<std::vector<float>> &points, int index, KdTree *tree, float distanceTol,
               std::vector<bool> &processed, std::vector<int> &cluster)
{
    processed[index] = true;
    cluster.push_back(index);

    std::vector<int> neighbours = tree->search(points[index], distanceTol);
    for (int id : neighbours)
    {
        if (processed[id] == false)
        {
            Proximity(points, id, tree, distanceTol, processed, cluster);
        }
    }
}

std::vector<std::vector<int>>
EuclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;

    std::vector<bool> processed(points.size(), false);
    for (auto i = 0; i < points.size(); i++)
    {
        if (processed[i] == false)
        {
            std::vector<int> cluster;
            Proximity(points, i, tree, distanceTol, processed, cluster);
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

}
