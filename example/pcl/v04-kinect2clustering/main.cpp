
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>  //j//
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#ifdef _WIN32
#define sleep(x) Sleep((x)*1000)
#endif

class SimpleOpenNIViewer {
public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
        if (!viewer.wasStopped()) {
//            viewer.showCloud (cloud);
//            writer.write<pcl::PointXYZ> ("kinect_cloud.pcd", *cloud, false); //*
//            std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
//  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Write the planar inliers to disk
    extract.filter (*cloud_plane);
//    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered = cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
 // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

  //  std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
  //  std::stringstream ss;
  //  ss << "cloud_cluster_" << j << ".pcd";
  //  writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
  //  j++;
    viewer.showCloud (cloud_cluster);

  }

        }
    }

    void run () {

        pcl::Grabber* interface = new pcl::OpenNIGrabber();

        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
            boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        interface->registerCallback (f);

        interface->start ();

        while (!viewer.wasStopped()) {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
            printf("alive...\n");
        }

        interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;

    pcl::PCDWriter writer;  //j//

};

int main () {
    SimpleOpenNIViewer v;
    v.run ();
    return 0;
}

