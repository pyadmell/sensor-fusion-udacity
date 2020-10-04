// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    
    // Create voxel grid and reduce resolution
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes,filterRes,filterRes);
    typename pcl::PointCloud<PointT>::Ptr filteredVoxel(new pcl::PointCloud<PointT>());
    voxelGrid.filter(*filteredVoxel);

    // Region based filtering
    pcl::CropBox<PointT> regionCrop(true);
    regionCrop.setMin(minPoint);
    regionCrop.setMax(maxPoint);
    regionCrop.setInputCloud(filteredVoxel);
    regionCrop.filter(*filteredVoxel);

    // Filter roof points
    pcl::CropBox<PointT> roofCrop(true);
    roofCrop.setMin(Eigen::Vector4f(-1.5,-1.7,-1.0,1.0));
    roofCrop.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1.0));
    roofCrop.setInputCloud(filteredVoxel);
    std::vector<int> inlierIndex;
    roofCrop.filter(inlierIndex);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (const auto &indx: inlierIndex) {
        inliers->indices.push_back(indx);
    }

    pcl::ExtractIndices<PointT> extractIndex;
    extractIndex.setInputCloud(filteredVoxel);
    extractIndex.setIndices(inliers);
    extractIndex.setNegative(true);
    extractIndex.filter(*filteredVoxel);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
  
    return filteredVoxel;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    
  // extract plane cloud
  for (const auto &indx: inliers->indices)
  {
    planeCloud->points.push_back(cloud->points[indx]);
  }

  // extract obstacle cloud
  pcl::ExtractIndices<PointT> extractIndx;
  extractIndx.setInputCloud(cloud);
  extractIndx.setIndices(inliers);
  extractIndx.setNegative(true);
  extractIndx.filter(*obstacleCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices()};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients()};
    pcl::SACSegmentation<PointT> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(maxIterations);
    segmentation.setDistanceThreshold(distanceThreshold);
    segmentation.setInputCloud (cloud);
    segmentation.segment (*inliers, *coefficients);
    // END TODO

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

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //
    typename pcl::search::KdTree<PointT>::Ptr tree{new pcl::search::KdTree<PointT>};
    tree->setInputCloud(cloud);

    // euclidean cluster
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> euclideanClustering;
    euclideanClustering.setClusterTolerance(clusterTolerance); 
    euclideanClustering.setMinClusterSize(minSize);
    euclideanClustering.setMaxClusterSize(maxSize);
    euclideanClustering.setSearchMethod(tree);
    euclideanClustering.setInputCloud(cloud);
    euclideanClustering.extract(cluster_indices);

    for (pcl::PointIndices get_indices: cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster{new pcl::PointCloud<PointT>};
        for( auto const & indx: get_indices.indices) {
            cloud_cluster->points.push_back(cloud->points[indx]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }
    // End TODO

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
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

//TODO
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
                                        const float distanceTol) {
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;

    // TODO: Fill in this function
    srand(time(NULL));
    while (maxIterations--) {
        std::unordered_set<int> inliers;
        while(inliers.size()<3) { 
          inliers.insert(rand() % cloud->points.size()); 
        }
        auto iter = inliers.begin();
        float x1 = cloud->points[(*iter)].x;
        float y1 = cloud->points[(*iter)].y;
        float z1 = cloud->points[(*iter)].z;
        
        ++iter;
        float x2 = cloud->points[(*iter)].x;
        float y2 = cloud->points[(*iter)].y;
        float z2 = cloud->points[(*iter)].z;
        
        ++iter;
        float x3 = cloud->points[(*iter)].x;
        float y3 = cloud->points[(*iter)].y;
        float z3 = cloud->points[(*iter)].z;

        float i = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
        float j = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
        float k = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));

        float d = -((i * x1) + (j * y1) + (k * z1));
        float euclideanDist = std::sqrt((i*i)+(j*j)+(k*k));
        for (size_t indx =0; indx < cloud->points.size(); ++indx) {
              float plane = std::fabs((i*cloud->points[indx].x)+(j* cloud->points[indx].y)+(k*cloud->points[indx].z)+d);
              if ((plane / euclideanDist) <= distanceTol) { 
                inliers.insert(indx); 
              }
        }
        if (inliers.size() > inliersResult.size()) { 
          inliersResult = inliers; 
        }
    }

    typename pcl::PointCloud<PointT>::Ptr cloudInliers{ new pcl::PointCloud<PointT>() };
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers{ new pcl::PointCloud<PointT>() };
    if (!inliersResult.empty()) {
        for (size_t indx=0; indx<cloud->points.size(); ++indx) {
            PointT point = cloud->points[indx];
            if (inliersResult.find(indx)!=inliersResult.end()) {
                cloudInliers->points.push_back(point);
            } else {
                cloudOutliers->points.push_back(point);
            }
        }
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(
        cloudOutliers, cloudInliers);
}

template <typename PointT>
void ProcessPointClouds<PointT>::proximity(const std::vector<std::vector<float>> &points,
                                           std::vector<int> &cluster, std::vector<bool> &processed,
                                           const int index, const float tolerance, KdTree *tree) {
    processed[index] = true;
    cluster.push_back(index);
    const std::vector<int> nearbyPoints{ tree->search(points[index], tolerance) };
    for (auto const &nearbyIndex: nearbyPoints) {
        if (!processed[nearbyIndex]) {
            proximity(points, cluster, processed, nearbyIndex, tolerance, tree);
        }
    }
}

template <typename PointT>
std::vector<std::vector<int>>
ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree,
                                             float distanceTol) {

    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i) {
        if (!processed[i]) {
            std::vector<int> cluster;
            proximity(points, cluster, processed, i, distanceTol, tree);
            clusters.push_back(cluster);
        }
    }

    return clusters;
}
