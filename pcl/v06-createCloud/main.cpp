
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

int main () {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "Genarating example point cloud...\n\n";
    for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
      for (float y=-0.5f; y<=0.5f; y+=0.01f)
      {
        pcl::PointXYZ point;  point.x = x;  point.y = y;  point.z = 0;
        cloud->points.push_back (point);
      }
    }
    cloud->width = (int) cloud->points.size ();
    cloud->height = 1;
    std::cout << "Done!\n\n";

    std::cout << "Save...\n\n";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("kinect_cloud.pcd", *cloud, false); //*
    std::cout << "Done!\n\n";

    return 0;
}

