/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
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
    bool renderScene = false; // if in the rendering scene we don't want to see any cars, check false here
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO (done): Create lidar sensor 
    Lidar* lidar_sensor = new Lidar(cars, 0.0); // Note: The Lidar object should be created with a slope of 0
                                                // It's on the horizontal plane

    // Render lidar point clouds
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_point_clouds = lidar_sensor->scan(); // using typename is a good practice here
                                                                                            // it claims the item following it is a type instead a value
    // renderRays(viewer, lidar_sensor->position, lidar_point_clouds); // (wrong) as the ego car origin is Vect3(0, 0, 0)
    //                                                                 // fix: should be the lidar_sensor->position
    renderPointCloud(viewer, lidar_point_clouds, "lidar_point_clouds"); // default coler is Color(-1, -1, -1)
                                                                        // here the string shows the name of this point clouds
                                                                        // in case we have multiple clouds, we can use names to identify them
    // TODO (done): Create point processor
    ProcessPointClouds<pcl::PointXYZ>* point_processor = new ProcessPointClouds<pcl::PointXYZ>{}; // must specify the typename on both sides (DON'T FORGET THE ONE AFTER `new`!)
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = point_processor->SegmentPlane(lidar_point_clouds, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
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
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}