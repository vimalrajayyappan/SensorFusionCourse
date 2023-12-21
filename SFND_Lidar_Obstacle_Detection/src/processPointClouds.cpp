// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>
#include "kdtree.h"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vgf;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    vgf.setInputCloud(cloud);
    vgf.setLeafSize(filterRes, filterRes, filterRes);
    vgf.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obsCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    for (int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obsCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obsCloud, planeCloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cout << "no Planar seg" << std::endl;
    }

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (pcl::PointIndices cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (const auto idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud)[idx]);
        } //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1.5;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        // std::stringstream ss;
        // ss << std::setw(4) << std::setfill('0') << j;
        // writer.write<pcl::PointXYZ>("cloud_cluster_" + ss.str() + ".pcd", *cloud_cluster, false); //*
        // j++;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

//RANSAC Custom-Implementation - Segmenting Ground Plane
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlaneCustomImplemented(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time Ransac process
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    //A random num generator
    srand(time(NULL));

    // For max iterations
    while (maxIterations--)
    {
        std::unordered_set<int> inliersSampled;
        // Since its set , there wont be any duplicates,
        // iterating till there will be thre unique indices
        while (inliersSampled.size() < 3)
        {
            inliersSampled.insert(rand() % cloud->points.size());
        }
        double x1, y1, z1, x2, y2, z2, x3, y3, z3;

        //Gathereing each points from corresponding index
        auto itr = inliersSampled.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        //Formula used in classrom has been utilised here
        //The Coefficients calculation
        double a, b, c, d;
        a = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
        b = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
        c = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
        d = -(a * x1 + b * y1 + c * z1);

        for (int index = 0; index < cloud->points.size(); index++)
        {
            // ignoring indices that are already used
            if (inliersSampled.count(index) > 0)
                continue;

            PointT point = cloud->points[index];
            double x4, y4, z4;
            x4 = point.x;
            y4 = point.y;
            z4 = point.z;
            //The perpendicular distance of point to plane
            double distance = fabs((a * x4) + (b * y4) + (c * z4) + d) / (sqrt((a * a) + (b * b) + (c * c)));

            // If distance is smaller than threshold add it as inlier
            if (distance <= distanceTol)
                inliersSampled.insert(index);
        }

        if (inliersSampled.size() > inliersResult.size())
        {
            inliersResult = inliersSampled;
        }
    }

    pcl::PointIndices::Ptr inliersPCL{new pcl::PointIndices};
    for (int ind : inliersResult)
        inliersPCL->indices.push_back(ind);

    // Create the filtering object
    typename pcl::ExtractIndices<PointT> extract;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Custom Ransac Implementation took" << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersPCL, cloud);
    return segResult;
}

//Fill each cluster with corresponding close points in the threshold
template<typename PointT>
void ProcessPointClouds<PointT>::FillOneCluster(typename pcl::PointCloud<PointT>::Ptr ipCloud,typename pcl::PointCloud<PointT>::Ptr oneCluster, std::vector<bool>& processedCloudPoints, int indx,KdTree* tree, float proximityTolerance )
{

    processedCloudPoints[indx] = true;
    PointT point = ipCloud->points[indx];
    oneCluster->push_back(point);
    
    std::vector<double> pointVector = {point.x, point.y, point.z};
    //Provides ids of points in pointcloud that are in tolerance.
    std::vector<int> idsInTolerance = tree->search(pointVector, proximityTolerance);
    for(int id : idsInTolerance)
    {
        if(!processedCloudPoints[id])
        {
            FillOneCluster(ipCloud,oneCluster, processedCloudPoints, id, tree, proximityTolerance );
        }
    }

}

// Clustering-Custom Implementation to group non-ground points into cluster based on input tolerance 
// Returns the number of clusters we found based in tolerance, minSize and maxsize params
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringCustomImplementation(typename pcl::PointCloud<PointT>::Ptr cloud, float clustSizeTol, int minSize, int maxSize)
{
    //Initializing KD-Tree
    KdTree* tree = new KdTree;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        PointT point = cloud->points[i];
        //Building Tree happens here
        tree->insert({point.x, point.y, point.z}, i);
    }

    //processedCloudPoints marks the points that are already processed
    std::vector<bool> processedCloudPoints(cloud->points.size(), false);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;

    for (int inx =0; inx < cloud->points.size(); inx++)
    {
        if(processedCloudPoints[inx])
            continue;

        typename pcl::PointCloud<PointT>::Ptr oneCluster(new pcl::PointCloud<PointT>);
        FillOneCluster(cloud,oneCluster,processedCloudPoints, inx, tree,clustSizeTol  );

        if(oneCluster->size() >=minSize && oneCluster->size() <=maxSize)
        {
            oneCluster->is_dense = true;
            oneCluster->width = oneCluster->size();
            oneCluster->height = 1;
            cloudClusters.push_back(oneCluster);
        }

    }

    std::cout << "Total Clusters Found : " << cloudClusters.size() << std::endl;
    delete tree;
    return cloudClusters;

}

template <typename PointT>
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}