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
    // bool renderScene = true;
    bool renderScene = 0;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd = lidar->scan();
    // renderRays(viewer, lidar->position, pcd );
    renderPointCloud(viewer, pcd, "pcd" );
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =  pointProcessor.SegmentPlane(pcd, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obsCloud", Color(1,0,0) );
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0) );

    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> opClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3.0, 30);
    std::vector<Color> colours = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr clust : opClusters )
    {
        pointProcessor.numPoints(clust);
        renderPointCloud(viewer, clust, "obsCloud"+std::to_string(clusterId),colours[clusterId%colours.size()] );
        

        Box box  = pointProcessor.BoundingBox(clust);
        renderBox(viewer, box, clusterId);
        ++clusterId;

    }
    std::cout << "clustersize" << opClusters.size() << endl;
  
}

//City block function for custom implementing Ransac and Clustering algorithm
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor,pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud )
{
    
 	//Filtering the huge cloud data so as to use less number of cloud points for efficiency
    inputCloud = pointProcessor->FilterCloud(inputCloud, 0.23, Eigen::Vector4f(-10,-5,-4,1),Eigen::Vector4f(30,7,4,1));
    //Ransac Custom Implementation - For ground segmentation of pointclouds
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =  pointProcessor->RansacPlaneCustomImplemented(inputCloud, 100, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obsCloud", Color(0,1,0) );
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(1,0,0) );
    //Clustering Custom Implementation - For grouping non-ground objects as a set of clusters
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> listOfClusters = pointProcessor->ClusteringCustomImplementation(segmentCloud.first, 0.42,12, 400);
    int clusterId = 0;
    std::vector<Color> colours = {Color(0.75, 0.75, 0.75), Color(0,1,1), Color(1,1,0)};
    //Helps in bounding each cluster with a box
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr oneCluster : listOfClusters)
    {
        int colorId = clusterId%colours.size();
        renderPointCloud(viewer, oneCluster, "obsCloud"+std::to_string(clusterId), colours[colorId] );
        clusterId++;
        Box box = pointProcessor->BoundingBox(oneCluster);
        renderBox(viewer,box,clusterId);
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
    // simpleHighway(viewer);
    //Initializing PointProcessor
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    //streaming the pcd file paths from the corresponding location
    std::vector<boost::filesystem::path> stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIT = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;
    while (!viewer->wasStopped ())
    {
        //set to initial view
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //Load each PCD
        inputCloud = pointProcessor->loadPcd((*streamIT).string());
        cityBlock(viewer, pointProcessor, inputCloud);
        streamIT++;
        if(streamIT == stream.end())
            streamIT = stream.begin();

        viewer->spinOnce ();
    } 
    delete pointProcessor;
}