#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <pcl/console/parse.h>

typedef pcl::PointXYZRGB PointT;

// int main () {
int main (int argc, char** argv) {
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr region_cloud (new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile ("region_full.pcd", *cloud);

    // init colors and name variables
    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};


    // init visualizer
    boost::shared_ptr < pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
    viewer->setBackgroundColor (0.0, 0.0, 0);
    viewer->addPointCloud<PointT> (cloud, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "cloud");
    viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters ();

    if (pcl::console::find_argument (argc, argv, "-r") >= 0) {
        for (int i = 1; i < 5; i++) {
            std::string name = "region_";
            char numstr[2];

            sprintf(numstr, "%i", i);
            name.append(numstr);
            name.append(".pcd");
            pcl::io::loadPCDFile (name, *region_cloud);

            pcl::visualization::PointCloudColorHandlerCustom<PointT> color (region_cloud, red[i], grn[i], blu[i]);
            viewer->addPointCloud<PointT> (region_cloud, color, name);
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
        }
    }

    while (!viewer->wasStopped ()) {
        viewer->spinOnce ();
    }
    return 0;
}
