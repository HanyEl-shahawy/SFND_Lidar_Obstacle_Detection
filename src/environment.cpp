/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include <memory>
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    auto lidar{new Lidar {cars,  0.0}};
        // TODO:: Create point processor
    auto pcl{lidar->scan()};
    renderPointCloud(viewer, pcl, "pcl");
    ProcessPointClouds<pcl::PointXYZ> proc_pcl{};
    ///@brief segmentation part
    auto segment_cloud = proc_pcl.SegmentPlane(pcl, 100, 0.2);
    renderPointCloud(viewer, segment_cloud.first, "obstacle", Color{1,0,0});
    //renderPointCloud(viewer, segment_cloud.second, "plane", Color{0,1,0});
    ///@brief clustering part
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =
            proc_pcl.Clustering(segment_cloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        //proc_pcl.numPoints(cluster);
        //renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = proc_pcl.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }


}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
    /*Filter: Crop the scene to requested dimensions and remove the points from the roof top*/
    filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.15 ,
            Eigen::Vector4f (-10, -6, -2, 1), Eigen::Vector4f ( 30, 7, 5, 1));
    /*Segmentation: Segment the scene to create to clouds one for road and another for objects.*/
    auto segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 10, 0.3);
    /*Clustering: Identify the clusters from objects*/
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
            pointProcessorI->Clustering(segmentCloud.second, 1.0, 10, 140);
    renderPointCloud(viewer,segmentCloud.first,"Plane cloud",Color(0,1,0));
    renderPointCloud(viewer,segmentCloud.second,"Obstacles Cloud",Color(1,0,0));

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    /*Render the objects along with bounding boxes to the viewer*/
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);

        ++clusterId;
    }

}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        //renderPointCloud(viewer,inputCloudI,"Example");

        /*Call cityBlock to identify the objects*/
        cityBlock(viewer, pointProcessorI, inputCloudI);

        // Increment to next scene and if the last scene is reached loop back to first scene.
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce ();
    }

}