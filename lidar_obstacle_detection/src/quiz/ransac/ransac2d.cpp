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

	// TODO (done): Fill in this function
	// For max iterations 
	for (int i = 0; i < maxIterations; i++)
	{
		// Randomly sample subset and fit line
		int n = cloud->points.size();
		int idx1, idx2;
		idx1 = rand() % n;
		idx2 = rand() % n;
		while (idx1 == idx2) // make sure idx1 and idx2 are different
			idx2 = rand() & n;

		auto pt1 = cloud->points[idx1];
		auto pt2 = cloud->points[idx2];
		// Fit a line: Ax + By + C = 0
		float A = pt1.y - pt2.y;
		float B = pt2.x - pt1.x;
		float C = pt1.x * pt2.y - pt2.x * pt1.y;

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		// According to the equation: (x*, y*) to line Ax + By + C = 0 has a distance:
		// |Ax* + By* + C| / sqrt(A^2 + B^2)
		float denom = sqrt(A * A + B * B);

		// use a temp container to store inliers
		std::unordered_set<int> inliers;
		for (int index = 0; index < cloud->points.size(); index++)
		{
			auto pt = cloud->points[index];
			float dist = fabs(A * pt.x + B * pt.y + C) / denom; // must use float version of abs, i.e., fabs()
			if (dist < distanceTol)
				inliers.insert(index);
		}
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

int main ()
{
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	// TODO (done): Change the max iteration and distance tolerance arguments for Ransac function
	// rule of thumb: if having lower ratio of inliers, use a larger iteration number
	std::unordered_set<int> inliers = Ransac(cloud, 64, 0.6);

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
