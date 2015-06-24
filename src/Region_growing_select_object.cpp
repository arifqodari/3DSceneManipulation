/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 * 
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: openni_viewer.cpp 5059 2012-03-14 02:12:17Z gedikli $
 *
 */


#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/common/common.h>
#include <pcl/console/time.h>
#include <pcl/common/angles.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/geometry/polygon_operations.h>

#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>

using namespace pcl;
using namespace pcl::console;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
class ObjectSelection
{
  public:
  
    /////////////////////////////////////////////////////////////////////////
    void 
    keyboard_callback (const visualization::KeyboardEvent&, void*)
    {
      //if (event.getKeyCode())
      //  cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode() << ") was";
      //else
      //  cout << "the special key \'" << event.getKeySym() << "\' was";
      //if (event.keyDown())
      //  cout << " pressed" << endl;
      //else
      //  cout << " released" << endl;
    }
    
    /////////////////////////////////////////////////////////////////////////
    void 
    mouse_callback (const visualization::MouseEvent& mouse_event, void*)
    {
      if (mouse_event.getType() == visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == visualization::MouseEvent::LeftButton)
      {
        cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
      }
    }

    /////////////////////////////////////////////////////////////////////////
    void
    segment (const PointT &picked_point, 
             int picked_idx,
             PlanarRegion<PointT> &region)
    {
		// kd-tree object for searches.
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
		kdtree->setInputCloud(cloud_);

		// Color-based region growing clustering object.
		pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
		clustering.setInputCloud(cloud_);
		clustering.setSearchMethod(kdtree);
		// Here, the minimum cluster size affects also the postprocessing step:
		// clusters smaller than this will be merged with their neighbors.
		clustering.setMinClusterSize(400);
		// Set the distance threshold, to know which points will be considered neighbors.
		clustering.setDistanceThreshold(10);
		// Color threshold for comparing the RGB color of two points.
		clustering.setPointColorThreshold(6);
		// Region color threshold for the postprocessing step: clusters with colors
		// within the threshold will be merged in one.
		clustering.setRegionColorThreshold(10);

		std::vector <pcl::PointIndices> clusters;
		clustering.extract(clusters);


		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>());
		cloud_ = clustering.getColoredCloud();

		print_highlight("Number of planar regions detected: %lu for a cloud of %lu points\n", clusters.size(), cloud_->size());

		int K = 1;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		float smallestDist = 100000;
		
		pcl::KdTreeFLANN<pcl::PointXYZRGB> kt;
		kt.setInputCloud(cloud_);
		kt.nearestKSearch(picked_point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
		int counter = 0;
		
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
		{

			//if (pointNKNSquaredDistance[0] < smallestDist)
			//{
//				PCL_INFO("IN SMALLEST");
			if(std::find(i->indices.begin(), i->indices.end(), pointIdxNKNSearch[0]) == i->indices.end())
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
				//typename PointCloud<PointT>::Ptr cloud_;
				for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
				{
					cluster->points.push_back(cloud_->points[*point]);
				}
				//smallestDist = pointNKNSquaredDistance[0];
				cluster->width = cluster->points.size();
				cluster->height = 1;
				cluster->is_dense = true;

				*newCloud = (*cluster) + (*newCloud);


				//cloud_viewer_->addPointCloud(cluster, std::to_string(counter));
			}
		//	counter++;
		}
		

		//pcl::PCDWriter writer;
		//pcl::console::print_highlight("Number of segments done is %lu\n", clusters.size());
		//writer.write<pcl::PointXYZRGB>("segment_result.pcd", *cluster, false);

		//std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		//std::string fileName = "cluster_test.pcd";
		//pcl::io::savePCDFileBinary(fileName, *cluster);
		cloud_viewer_->removePointCloud("scene");
		cloud_viewer_->addPointCloud(newCloud, "scene");
		cloud_ = newCloud;

		//counter++;
		//cloud_viewer_->removePointCloud("scene");

		/*
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

		std::cout << "Neighbors within radius search at (" << searchPoint.x
			<< " " << searchPoint.y
			<< " " << searchPoint.z
			<< ") with radius=" << radius << std::endl;


		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
				std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
				<< " " << cloud->points[pointIdxRadiusSearch[i]].y
				<< " " << cloud->points[pointIdxRadiusSearch[i]].z
				<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
		}*/

    }

    /////////////////////////////////////////////////////////////////////////
    /** \brief Point picking callback. This gets called when the user selects
      * a 3D point on screen (in the PCLVisualizer window) using Shift+click.
      *
      * \param[in] event the event that triggered the call
      */
    void 
    pp_callback (const visualization::PointPickingEvent& event, void*)
    {
      // Check to see if we got a valid point. Early exit.
      int idx = event.getPointIndex ();
      if (idx == -1)
        return;

      vector<int> indices (1);
      vector<float> distances (1);

      // Get the point that was picked
      PointT picked_pt;
      event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);

      print_info (stderr, "Picked point with index %d, and coordinates %f, %f, %f.\n", idx, picked_pt.x, picked_pt.y, picked_pt.z);

      // Add a sphere to it in the PCLVisualizer window
      stringstream ss;
      ss << "sphere_" << idx;
      cloud_viewer_->addSphere (picked_pt, 0.01, 1.0, 0.0, 0.0, ss.str ());

      // Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
      search_->nearestKSearch (picked_pt, 1, indices, distances);

      // Add some marker to the image
      if (image_viewer_)
      {
        // Get the [u, v] in pixel coordinates for the ImageViewer. Remember that 0,0 is bottom left.
        uint32_t width  = search_->getInputCloud ()->width,
                 height = search_->getInputCloud ()->height;
        int v = height - indices[0] / width,
            u = indices[0] % width;

        image_viewer_->addCircle (u, v, 5, 1.0, 0.0, 0.0, "circles", 1.0);
        image_viewer_->addFilledRectangle (u-5, u+5, v-5, v+5, 0.0, 1.0, 0.0, "boxes", 0.5);
        image_viewer_->markPoint (u, v, visualization::red_color, visualization::blue_color, 10);
      }

      // Segment the region that we're interested in, by employing a two step process:
      //  * first, segment all the planes in the scene, and find the one closest to our picked point
      //  * then, use euclidean clustering to find the object that we clicked on and return it
      PlanarRegion<PointT> region;
      segment (picked_pt, indices[0], region);

      // If no region could be determined, exit
      if (region.getContour ().empty ())
      {
        PCL_ERROR ("No planar region detected. Please select another point or relax the thresholds and continue.\n");
        return;
      }
      // Else, draw it on screen
      else
      {
        cloud_viewer_->addPolygon (region, 0.0, 0.0, 1.0, "region");
        cloud_viewer_->setShapeRenderingProperties (visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "region");

        // Draw in image space
        if (image_viewer_)
        {
          image_viewer_->addPlanarPolygon (search_->getInputCloud (), region, 0.0, 0.0, 1.0, "refined_region", 1.0);
        }
      }
    }
    
    /////////////////////////////////////////////////////////////////////////
    void
    compute ()
    {
      // Visualize the data
      while (!cloud_viewer_->wasStopped ())
      {
        /*// Add the plane that we're tracking to the cloud visualizer
        PointCloud<PointT>::Ptr plane (new Cloud);
        if (plane_)
          *plane = *plane_;
        visualization::PointCloudColorHandlerCustom<PointT> blue (plane, 0, 255, 0);
        if (!cloud_viewer_->updatePointCloud (plane, blue, "plane"))
          cloud_viewer_->addPointCloud (plane, "plane");
*/
        cloud_viewer_->spinOnce ();
        if (image_viewer_)
        {
          image_viewer_->spinOnce ();
          if (image_viewer_->wasStopped ())
            break;
        }
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }
    }
    
    /////////////////////////////////////////////////////////////////////////
    void
    initGUI ()
    {
      cloud_viewer_.reset (new visualization::PCLVisualizer ("PointCloud"));

      cloud_viewer_->registerMouseCallback (&ObjectSelection::mouse_callback, *this);
      cloud_viewer_->registerKeyboardCallback(&ObjectSelection::keyboard_callback, *this);
      cloud_viewer_->registerPointPickingCallback (&ObjectSelection::pp_callback, *this);
      cloud_viewer_->setPosition (0, 0);

      cloud_viewer_->addPointCloud (cloud_, "scene");
      cloud_viewer_->resetCameraViewpoint ("scene");
      cloud_viewer_->addCoordinateSystem (0.1, 0, 0, 0, "global");
    }

    /////////////////////////////////////////////////////////////////////////
    bool
    load (const std::string &file)
    {
      // Load the input file
      TicToc tt; tt.tic ();
      print_highlight (stderr, "Loading "); 
      print_value (stderr, "%s ", file.c_str ());
      cloud_.reset (new PointCloud<PointT>);
      if (io::loadPCDFile (file, *cloud_) < 0) 
      {
        print_error (stderr, "[error]\n");
        return (false);
      }
      print_info ("[done, "); print_value ("%g", tt.toc ()); 
      print_info (" ms : "); print_value ("%lu", cloud_->size ()); print_info (" points]\n");
      
      if (cloud_->isOrganized ())
        search_.reset (new search::OrganizedNeighbor<PointT>);
      else
        search_.reset (new search::KdTree<PointT>);

      search_->setInputCloud (cloud_);

      return (true);
    }
    
    /////////////////////////////////////////////////////////////////////////
    
	boost::shared_ptr<visualization::PCLVisualizer> cloud_viewer_;
    boost::shared_ptr<visualization::ImageViewer> image_viewer_;
    
    typename PointCloud<PointT>::Ptr cloud_;
    typename search::Search<PointT>::Ptr search_;
  private:
    // Results
    typename PointCloud<PointT>::Ptr plane_;
};

/* ---[ */
int
main (int argc, char** argv)
{
  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.empty ())
  {
    print_error ("  Need at least an input PCD file (e.g. scene.pcd) to continue!\n\n");
    print_info ("Ideally, need an input file, and three output PCD files, e.g., object.pcd, plane.pcd, rest.pcd\n");
    return (-1);
  }
  
  string object_file = "object.pcd", plane_file = "plane.pcd", rest_file = "rest.pcd";
  if (p_file_indices.size () >= 4)
    rest_file = argv[p_file_indices[3]];
  if (p_file_indices.size () >= 3)
    plane_file = argv[p_file_indices[2]];
  if (p_file_indices.size () >= 2)
    object_file = argv[p_file_indices[1]];


  PCDReader reader;
  // Test the header
  pcl::PCLPointCloud2 dummy;
  reader.readHeader (argv[p_file_indices[0]], dummy);
  if (dummy.height != 1 && getFieldIndex (dummy, "rgb") != -1)
  {
    print_highlight ("Enabling 2D image viewer mode.\n");
    ObjectSelection<PointXYZRGB> s;
    if (!s.load (argv[p_file_indices[0]])) return (-1);
    s.initGUI ();
    s.compute ();
  }
  else
  {
    ObjectSelection<PointXYZRGB> s;
    if (!s.load (argv[p_file_indices[0]])) return (-1);
    s.initGUI ();
    s.compute ();
  }

  return (0);
}
/* ]--- */