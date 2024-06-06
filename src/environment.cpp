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


std::vector<std::vector<float>> convertPointCloudToVector(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    std::vector<std::vector<float>> points;

    // Iterate over each point in the point cloud
    for (const auto& point : *cloud) {
        // Extract x, y, z coordinates from the point
        float x = point.x;
        float y = point.y;
        float z = point.z;

        // Create a std::vector<float> for the point and push it into the points vector
        points.push_back({x, y, z});
    }

    return points;
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> convertToPointClouds(
    const std::vector<std::vector<int>>& clusters, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr inputPoints) {

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointClouds;

    for (const auto& cluster : clusters) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        for (int index : cluster) {
           //  cloud->points.push_back(points[index]);
             cloud->points.push_back(inputPoints->points[index]);
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;  // Unorganized point cloud
        cloud->is_dense = true;

        pointClouds.push_back(cloud);
    }

    return pointClouds;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  Eigen::Vector4f minPoint(-10, -5, -2, 1);  // 10 meters to the left, 5 meters back, 2 meter down from the origin
  Eigen::Vector4f maxPoint(30, 8, 1, 1);     // 8 meters to the right, 30 meters forward, 1 meter up from the origin

  auto filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2f, minPoint, maxPoint);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 200, 0.3);

  renderPointCloud(viewer, segmentCloud.second, "segmentCloud", Color(0,1,0));

	KdTree* tree = new KdTree;
  
    for (int i = 0; i < segmentCloud.first->points.size(); ++i) {
        std::vector<float> point = {segmentCloud.first->points[i].x, segmentCloud.first->points[i].y, segmentCloud.first->points[i].z};
        tree->insert(point, i);
    }

    std::vector<std::vector<int>> clusters = pointProcessorI->euclideanClustering(convertPointCloudToVector(segmentCloud.first),tree, .53);
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = convertToPointClouds(clusters, segmentCloud.first);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
      	std::cout << "cluster size ";
      	pointProcessorI->numPoints(cluster);
      	renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
      	++clusterId;
    }
}
 
void cityBlockPCL(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  Eigen::Vector4f minPoint(-10, -5, -2, 1);  // 10 meters to the left, 5 meters back, 2 meter down from the origin
  Eigen::Vector4f maxPoint(30, 8, 1, 1);     // 8 meters to the right, 30 meters forward, 1 meter up from the origin

  auto filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2f, minPoint, maxPoint);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 200, 0.3);

  renderPointCloud(viewer, segmentCloud.second, "segmentCloud", Color(0,1,0));

	KdTree* tree = new KdTree;
  
    for (int i = 0; i < segmentCloud.first->points.size(); ++i) {
        std::vector<float> point = {segmentCloud.first->points[i].x, segmentCloud.first->points[i].y, segmentCloud.first->points[i].z};
        tree->insert(point, i);
    }

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, .53, 15, 500);
	
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
      	std::cout << "cluster size ";
      	pointProcessorI->numPoints(cluster);
      	renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
      	++clusterId;
    }
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
    Lidar* lidar = new Lidar(cars, 0.0);
    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
//    renderRays(viewer, lidar->position, inputCloud); 
//    renderPointCloud(viewer, inputCloud, "inputCloud");
//    ProcessPointClouds<pcl::PointXYZ> pointProcessor;// create process object on stack
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();// create process object on heap

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
	
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
      	std::cout << "cluster size ";
      	pointProcessor->numPoints(cluster);
      	renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = pointProcessor->BoundingBox(cluster);
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
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");//creates a stream of file paths
    auto streamIterator = stream.begin();//pointer to first file path 0000000.pcd
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    // simpleHighway(viewer);
    // cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
       // Clear viewer
       viewer->removeAllPointClouds();
       viewer->removeAllShapes();

       // Load pcd and run obstacle detection process
       inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
       cityBlock(viewer, pointProcessorI, inputCloudI);
    
       streamIterator++;
       if(streamIterator == stream.end())
         streamIterator = stream.begin();
         
       viewer->spinOnce ();
    } 
}
