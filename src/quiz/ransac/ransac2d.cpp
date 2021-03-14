/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
	for(auto i = 0; i < maxIterations; i++)
	{
	    std::unordered_set<int> tempInliers;

        // Randomly sample subset and fit line
        auto point1 = cloud->at(rand() % cloud->points.size());
        auto point2 = cloud->at(rand() % cloud->points.size());

        float A = point1.y - point2.y;
        float B = point2.x - point1.x;
        float C = (point1.x * point2.y) - (point2.x * point1.y);

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        for(auto j = 0; j <  cloud->points.size(); j++)
        {
            pcl::PointXYZ point = cloud->points[j];
            float d = fabs((A * point.x + B * point.y + C) / sqrt((A * A) + (B * B)));

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

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
            pcl::PointXYZ point = cloud->points[j];
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

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();



    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
//	std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);
    std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
}
