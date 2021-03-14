// PCL lib Functions for processing point cloud;s

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"
#include "quiz/cluster/kdtree.h"

namespace EuclideanCluster
{
void Proximity(const std::vector<std::vector<float>> &points, int index, KdTree *tree, float distanceTol,
               std::vector<bool> &processed, std::vector<int> &cluster);

std::vector<std::vector<int>>
EuclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol);

template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function

    // For max iterations
    for(auto iter = 0; iter < maxIterations; iter++)
    {
        std::unordered_set<int> tempInliers;

        // Randomly sample subset and fit line
        auto point1 = cloud->at(rand() % cloud->points.size());
        auto point2 = cloud->at(rand() % cloud->points.size());
        auto point3 = cloud->at(rand() % cloud->points.size());

        float i = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
        float j = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
        float k = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point2.y) * (point3.x - point1.x);

        float A = i;
        float B = j;
        float C = k;
        float D = -1.0 * (i * point1.x + j * point1.y + k * point1.z);

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        for(auto j = 0; j <  cloud->points.size(); j++)
        {
            PointT point = cloud->points[j];
            float d = fabs((A * point.x + B * point.y + C * point.z + D) / sqrt((A * A) + (B * B) + (C * C)));

            if(d <= distanceTol)
            {
                tempInliers.insert(j);
            }
        }

        if(tempInliers.size() > inliersResult.size())
        {
            inliersResult = std::move(tempInliers);
        }
    }

    // Return indicies of inliers from fitted line with most inliers
    return inliersResult;
}
}

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();

    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr
    FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint,
                Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
    SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
    SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
    CustomSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr>
    Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
    std::vector<typename pcl::PointCloud<PointT>::Ptr>
    CustomClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr voxelCloud{new pcl::PointCloud<PointT>};
    typename pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter(*voxelCloud);

    typename pcl::PointCloud<PointT>::Ptr croppedCloud{new pcl::PointCloud<PointT>};
    typename pcl::CropBox<PointT> cropBox(true);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(voxelCloud);
    cropBox.filter(*croppedCloud);

    std::vector<int> indices;
    pcl::CropBox<pcl::PointXYZI> cropBoxRoof(true);
    cropBoxRoof.setInputCloud(croppedCloud);
    cropBoxRoof.setMin(Eigen::Vector4f(-2.0, -2.0, -3.0, 1.0));
    cropBoxRoof.setMax(Eigen::Vector4f(3.0, 2.0, 3.0, 1.0));
    cropBoxRoof.filter(indices); // here we have indices of points belonging to the car's roof

    pcl::PointIndices::Ptr point_indices(new pcl::PointIndices());
    for (auto ind : indices)
    {
        point_indices->indices.push_back(ind);
    }

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    typename pcl::PointCloud<PointT>::Ptr outputCloud{new pcl::PointCloud<PointT>};
    // Extract all the points that DO NOT belong to car's roof
    extract.setInputCloud(croppedCloud);
    extract.setIndices(point_indices);
    extract.setNegative(true);
    extract.filter(*outputCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return outputCloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr plane{new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr obstacles{new pcl::PointCloud<PointT>()};

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plane);

    extract.setNegative(true);
    extract.filter (*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(plane, obstacles);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::CustomSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
                                         float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    std::unordered_set<int> inliers = EuclideanCluster::RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "custom plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return {cloudInliers, cloudOutliers};
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    typename pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

    // Set params for seg
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations(maxIterations);

    // Set segmentation input
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for(const auto& index : it->indices)
        {
            cloud_cluster->push_back (cloud->points[index]); //*
        }

        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::CustomClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
   KdTree* tree = new KdTree();
   std::vector<std::vector<float>> rawPoints;

    for (int i=0; i < cloud->points.size(); i++)
    {
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        tree->insert(point, i);

        // Also fill in vector of points that has adapted format for custom euclideanCluster
        rawPoints.push_back(point);
    }

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    //
    std::vector<std::vector<int>> clusters = EuclideanCluster::EuclideanCluster(rawPoints, tree, clusterTolerance);
    //
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "custom clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::vector<typename pcl::PointCloud<PointT>::Ptr> output;
    for(auto& cluster : clusters)
    {
        if(cluster.size() >= minSize && cluster.size() <= maxSize)
        {
            typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
            for(int idx: cluster)
            {
                PointT point = cloud->points[idx];
                clusterCloud->points.push_back(point);
            }

            clusterCloud->width = clusterCloud->points.size();
            clusterCloud->height = 1;
            clusterCloud->is_dense = true;
            output.push_back(clusterCloud);
        }
    }

    return output;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

#endif /* PROCESSPOINTCLOUDS_H_ */