#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <string>

typedef pcl::PointXYZRGBA PointT;

int main () {
    /**
     * load point cloud
     */

    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile ("scene_input.pcd", *cloud);


    /**
     * Estimating normals
     */

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    // ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(20.0f);
    ne.setInputCloud(cloud);

    // estimate normals using integral image
    ne.compute(*normals);


    /**
     * Build visualizer
     */

    // init point cloud
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (cloud, 0, 255, 0);
    pcl::PointCloud<PointT>::Ptr init_cloud_ptr (new pcl::PointCloud<PointT>);

    // init visualizer
    boost::shared_ptr < pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
    viewer->setBackgroundColor (0.0, 0.0, 0);
    viewer->addPointCloud<PointT> (init_cloud_ptr, single_color, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "cloud");
    viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters ();

    // update the visualizer
    if (!viewer->updatePointCloud<PointT> (cloud, "cloud"))
        viewer->addPointCloud<PointT> (cloud, "cloud");

    /**
     * Segmentation part
     */

    // init colors and name variables
    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
    char name[1024];

    // init segmentation
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label>mps;
    mps.setMinInliers (1000);
    mps.setAngularThreshold (pcl::deg2rad (3.0)); // 3 degrees
    mps.setDistanceThreshold (0.02); // 2cm
    mps.setInputNormals (normals);
    mps.setInputCloud (cloud);

    // compute segmentation
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    mps.segmentAndRefine (regions);

    // display each region
    for (size_t i = 0; i < regions.size (); i++) {
        Eigen::Vector3f centroid = regions[i].getCentroid ();
        Eigen::Vector4f model = regions[i].getCoefficients ();
        pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
        pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                centroid[1] + (0.5f * model[1]),
                centroid[2] + (0.5f * model[2]));

        // get boundary cloud
        sprintf (name, "normal_%lu", i);
        viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);
        pcl::PointCloud<PointT>::Ptr boundary_cloud (new pcl::PointCloud<PointT>);
        boundary_cloud->points = regions[i].getContour ();
        sprintf (name, "plane_%02zu", i);

        // just for info
        printf ("Centroid: (%f, %f, %f)\n  Coefficients: (%f, %f, %f, %f)\n Inliers: %d\n",
                centroid[0], centroid[1], centroid[2],
                model[0], model[1], model[2], model[3],
                boundary_cloud->points.size ());

        // show the regions
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color (boundary_cloud, red[i], grn[i], blu[i]);
        viewer->addPointCloud<PointT> (boundary_cloud, color, name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);

        // prepare boundary cloud for saving
        boundary_cloud->width = boundary_cloud->points.size ();
        boundary_cloud->height = 1;
        boundary_cloud->is_dense = true;

        // save each region into file
        std::stringstream ss;
        std::string result;
        ss << "boundary_cloud_" << i << ".pcd";
        result = ss.str();
        pcl::io::savePCDFileASCII (result, *boundary_cloud);
    }


    // show viewer
    while (!viewer->wasStopped ()) {
        viewer->spinOnce ();
    }

    return 0;
}

