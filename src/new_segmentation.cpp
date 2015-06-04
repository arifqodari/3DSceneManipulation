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
    
int user_data;
    
void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
    
}
    
void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}
    
int 
main ()
{
    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);
    
    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    
    
    // visualize
    //pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    //viewer.setBackgroundColor (0.0, 0.0, 0.5);
    //viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
    
    //while (!viewer.wasStopped ())
    //{
        //viewer.spinOnce ();
    //}
    
    // Segment planes
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ,pcl::Normal,pcl::Label>mps;
    mps.setMinInliers (10000);
    mps.setAngularThreshold (0.017453 * 2.0); // 2 degrees
    mps.setDistanceThreshold (0.02); // 2cm
    mps.setInputNormals (normals);
    mps.setInputCloud (cloud);
    std::vector<pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ> > > regions;
    mps.segmentAndRefine (regions);

    for (size_t i = 0; i < regions.size (); i++)
    {
      Eigen::Vector3f centroid = regions[i].getCentroid ();
      Eigen::Vector4f model = regions[i].getCoefficients ();
      pcl::PointCloud<pcl::PointXYZ> boundary_cloud;;
      boundary_cloud.points = regions[i].getContour ();
      printf ("Centroid: (%f, %f, %f)\n  Coefficients: (%f, %f, %f, %f)\n Inliers: %d\n",
              centroid[0], centroid[1], centroid[2],
              model[0], model[1], model[2], model[3],
              boundary_cloud.points.size ());
              
      boundary_cloud.width = boundary_cloud.points.size ();
      boundary_cloud.height = 1;
      boundary_cloud.is_dense = true;
      
      std::stringstream ss;
      std::string result;
      ss << "boundary_cloud_" << i << ".pcd";
      result = ss.str();
      pcl::io::savePCDFileASCII (result, boundary_cloud);
    }
    
    return 0;
}

