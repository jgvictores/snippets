
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>  //j//

#ifdef _WIN32
#define sleep(x) Sleep((x)*1000)
#endif

class SimpleOpenNIViewer {
public:
//    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}
    SimpleOpenNIViewer () {}

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
//        if (!viewer.wasStopped())
//            viewer.showCloud (cloud);
            printf("alive2...\n");
            writer.write<pcl::PointXYZ> ("kinect_cloud.pcd", *cloud, false); //*
    }

    void run () {
      while(1){

        pcl::Grabber* interface = new pcl::OpenNIGrabber();

        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
            boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        interface->registerCallback (f);

        interface->start ();

//        while (!viewer.wasStopped()) {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
            printf("alive...\n");
//        }

        interface->stop ();
      }
    }

//    pcl::visualization::CloudViewer viewer;
    pcl::PCDWriter writer;  //j//

};

int main () {
    SimpleOpenNIViewer v;
    v.run ();
    return 0;
}

