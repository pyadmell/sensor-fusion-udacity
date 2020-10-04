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
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rawPointCloud = lidar->scan();

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> processPoint;

    // segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> 
      segmentation = processPoint.SegmentPlane(rawPointCloud,100,0.2);

    // clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusterVec = 
      processPoint.Clustering(segmentation.first, 1.0, 3, 30);

    // rendering
    renderPointCloud(viewer,rawPointCloud, "Raw Point Cloud");
    renderPointCloud(viewer,segmentation.first,"Obstacle",Color(1, 0, 0));
    renderPointCloud(viewer,segmentation.second,"Plane",Color(0, 1, 0));

    int clusterID=0;
    std::vector<Color> colorVec={Color(1, 0, 0),Color(0, 1, 0),Color(0, 0, 1)};
    bool renderBoundingBox=true;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr c: cloudClusterVec)
    {
        renderPointCloud(viewer,c,"Cloud Cluster"+std::to_string(clusterID), colorVec[clusterID]);
        Box b = processPoint.BoundingBox(c);
        renderBox(viewer,b,clusterID);
        ++clusterID;
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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessor, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
    // filter point cloud
    float X = 30.0;
    float Y = 6.0;
    float Z = 3.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor->FilterCloud(inputCloud, 0.1, 
        Eigen::Vector4f(-X,-Y,-Z,1),Eigen::Vector4f(X,Y,Z,1));
    
    // render filtered point cloud
    renderPointCloud(viewer, filteredCloud, "Filtered Cloud");

    // segment point cloud into obstacles and plane
    bool usePclSegmentation = false;
    int maxIterations = 50;
    float distanceTol = 0.3;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud;
    if(!usePclSegmentation) {
      segmentCloud = pointProcessor->RansacPlane(filteredCloud, maxIterations, distanceTol);
    } else {
      segmentCloud = pointProcessor->SegmentPlane(filteredCloud, maxIterations, distanceTol);
    }

    if (segmentCloud.first->empty() || segmentCloud.second->empty()) { 
      std::cout<< "segmentation is empty" <<std::endl;
      return; 
    }

    // render obstacles and plane
    renderPointCloud(viewer, segmentCloud.first, "Obstacle Cloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "Plane Cloud", Color(0, 1, 0));

    // cluster obstacle points
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters{ pointProcessor->Clustering(
        segmentCloud.first, 0.5, 10, 2500) };

    if (cloudClusters.empty()){ 
      std::cout<<"No cluster founds"<<std::endl;
      return; 
    }

    // render bounding boxes
    int clusterId = 0;
    int colorIndex = 0;
    const std::vector<Color> colors{ Color(1, 0, 0), Color(0, 0, 1), Color(1, 1, 0) };
    bool renderBoundingBox = true;
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors.at(colorIndex));

        if(renderBoundingBox)
        {
          Box box(pointProcessor->BoundingBox(cluster));
          renderBox(viewer, box, clusterId);
        }
        ++clusterId;
        ++colorIndex;
        colorIndex %= colors.size();
    }
    
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    /*
    simpleHighway(viewer);
    while(!viewer->wasStopped())
    {
      viewer->spinOnce();
    }
    */


    // pcd stream
    ProcessPointClouds<pcl::PointXYZI>* processPoint = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> pcdStream = processPoint->streamPcd("../src/sensors/data/pcd/data_1");

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;
    auto it = pcdStream.begin();
    while (!viewer->wasStopped())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        inputCloud = processPoint->loadPcd((*it).string());
        cityBlock(viewer,processPoint,inputCloud);
        ++it;
        if (it == pcdStream.end())
        {
            it = pcdStream.begin();
        }
        viewer->spinOnce();
    }

}
