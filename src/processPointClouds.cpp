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
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);// true means want points inside the box, false would give points outside box presume
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

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point: indices)
        inliers->indices.push_back(point);

    // removing rooftop points from point cloud
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


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds: one for the segmented plane and one for the obstacles
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    
    // Ensure cloud and inliers are valid
    if (!cloud || !inliers) {
        throw std::runtime_error("Input cloud or inliers are null");
    }
    
    // Extract the inliers
    for (int index : inliers->indices) {
        if (index >= 0 && index < cloud->points.size()) {
            planeCloud->points.push_back(cloud->points[index]);
        } else {
            throw std::runtime_error("Index out of range in inliers");
        }
    }

    // Extract the obstacle cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    // Create and return the result pair
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlanePCL(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    // Create segmentation object
    //pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
       std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol){
        std::unordered_set<int> inliersResult;// for holding best inliers 
	    srand(time(NULL));
	
	    // For max iterations 
        while(maxIterations--) {
           // randomly pick 3 points to create line from
           std::unordered_set<int> inliers; // to hold samples inliers calculated this iteration
           while(inliers.size() < 3 ) {
             inliers.insert(rand()%(cloud->points.size())); // way of generating random number within range of number of points. This randomly selects 3 points to initially insert to inliers
           }

           float x1, y1, z1, x2, y2, z2, x3, y3, z3;

           auto itr = inliers.begin();//think pointer to inliers unordered_set 1st el
           // get x, y, z values for two points used to form line
           x1 = cloud->points[*itr].x;// accessing index here
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


           // calc line coefficients
           float a = ((y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1));
           float b = ((z2 - z1)*(x3 - x1) - (x2 - x1)*(y3 - y1)); 
           float c = ((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1));
           float d = -(a*x1 + b*y1 + c*z1);

           //  iterate through all other point cloud points
           for(int index = 0; index < cloud->points.size(); index++) {
              if(inliers.count(index)>0) // checks if point is already part of line, unorder_set.count returns no. els matching point in inliers set
                 continue;
              
              //pcl::PointXYZ point = cloud->points[index];// select point to calc distance from to line
              PointT point = cloud->points[index];// select point to calc distance from to line
              float x4 = point.x;
              float y4 = point.y;
              float z4 = point.z;

              // float d = fabs(a*x3+b*y3+c)/sqrt(a*a+b*b);// calc perp distance to line from point
              float dist = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a + b*b + c*c);

	          // If distance is smaller than threshold count it as inlier
              if(dist <= distanceTol)
                  inliers.insert(index);// insert point if w/in tol

 
           }
        
        if(inliers.size()>inliersResult.size()) {
           inliersResult = inliers;// update result if current has more inlier points
        }
        }

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

	std::unordered_set<int> inliers = RansacPlane(cloud, maxIterations, distanceThreshold);
    // Convert std::unordered_set<int> to pcl::PointIndices::Ptr
    typename pcl::PointIndices::Ptr inliersPtr(new pcl::PointIndices());

    for (int index : inliers) {
        inliersPtr->indices.push_back(index);
    }

    if (inliersPtr->indices.size() == 0) {
    // if (inliers.size() == 0) {
       std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersPtr,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, \
std::vector<bool>& processed, KdTree* tree, float distanceTol) {
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice], distanceTol);

	for(int id : nearest) {
		if(!processed[id]){
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
		}
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanClustering(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	int i = 0;

	while (i < points.size()) {
		if(processed[i]) {
			i++;
			continue;
		}

		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);// recursive, will fill cluster. Proximity fn in notes.
		clusters.push_back(cluster);
		i++;
	}
 
	return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
//
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    // loop through clusterIndices and create point clouds from them for each cluster
    for(pcl::PointIndices getIndices: clusterIndices)
    {
      typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
      for(int index : getIndices.indices)
        cloudCluster->points.push_back(cloud->points[index]);

      cloudCluster->width = cloudCluster->points.size();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;

      clusters.push_back(cloudCluster);
    }

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
