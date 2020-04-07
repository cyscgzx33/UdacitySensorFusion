/* \author Yusen Chen */
// Implementing simple RANSAC plane fitting

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

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations 
	for (int i = 0; i < maxIterations; i++)
	{
		// Randomly sample subset and fit line
		int n = cloud->points.size();
		
		// use a hash set to sample 3 different indices
		std::unordered_set<int> index_set;
		while (index_set.size() < 3)
			index_set.insert(rand() % n);

		auto it = index_set.begin();
		auto pt1 = cloud->points[*it++];
		auto pt2 = cloud->points[*it++];
		auto pt3 = cloud->points[*it++];

		// Fit a line: Ax + By + Cz + D = 0
		// First creating supportive vectors
		Eigen::Vector3f v1, v2;
		v1 << pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z;
		v2 << pt3.x - pt1.x, pt3.y - pt1.y, pt3.z - pt1.z;
		Eigen::Vector3f v = v1.cross(v2);

		float A = v(0);
		float B = v(1);
		float C = v(2);
		float D = - ( v(0) * pt1.x + v(1) * pt1.y + v(2) * pt1.z );

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		// According to the equation: (x*, y*, z*) to line Ax + By + Cz + D = 0 has a distance:
		// |Ax* + By* + Cz* + D| / sqrt(A^2 + B^2 + C^2)
		float denom = sqrt(A * A + B * B + C * C);

		// use a temp container to store inliers
		std::unordered_set<int> inliers;
		for (int index = 0; index < cloud->points.size(); index++)
		{
			auto pt = cloud->points[index];
			float dist = fabs(A * pt.x + B * pt.y + C * pt.z + D) / denom; // must use float version of abs, i.e., fabs()
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// Rule of thumb of choosing max iteration: if having lower ratio of inliers, use a larger iteration number
	std::unordered_set<int> inliers = Ransac3D(cloud, 64, 0.6);

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
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));    // Green
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));  // Red
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}

  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}

	return 0;
}