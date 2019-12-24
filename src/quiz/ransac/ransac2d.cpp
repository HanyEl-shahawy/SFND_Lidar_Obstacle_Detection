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
	auto start{std::chrono::steady_clock::now()};


	// TODO: Fill in this function
	while (maxIterations--) {
		std::unordered_set<int> inliers;

		while (inliers.size() < 3)
        {
            inliers.insert(rand() % (cloud->points.size()));
            inliers.insert(rand() % (cloud->points.size()));
            inliers.insert(rand() % (cloud->points.size()));
        }
        auto it{inliers.begin()};
		float x1, x2, x3, y1, y2, y3, z1, z2, z3;
		x1 = cloud->points[*it].x;
		y1 = cloud->points[*it].y;
        z1 = cloud->points[*it].z;
		it++;
		x2 = cloud->points[*it].x;
		y2 = cloud->points[*it].y;
        z2 = cloud->points[*it].z;
        it++;
        x3 = cloud->points[*it].x;
        y3 = cloud->points[*it].y;
        z3 = cloud->points[*it].z;
		float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		float d = -(a * x1 + b * y1 + c * z1);

		for (int index = 0; index < cloud->points.size(); ++index) {
			if (inliers.count(index) != 0)
				continue;
			float x = cloud->points[index].x;
			float y = cloud->points[index].y;
			float z = cloud->points[index].z;
			float d3 = (fabs(a * x + b * y + c * z + d)) / (sqrt(a*a + b*b + c*c));

			if (d3 <= distanceTol)
				inliers.insert(index);
		}
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	auto end{std::chrono::steady_clock::now()};
	auto t = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "RANSAC time is: " << t.count() << "\n";
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 500, 0.2);

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
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,0,1));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(0,1,0));
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
