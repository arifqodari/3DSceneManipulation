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
#include <pcl/io/vtk_io.h>

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

#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>

#include <math.h>

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
	  int clickMode = 0;
	  bool showSegments = false;
    /////////////////////////////////////////////////////////////////////////
    void keyboard_callback (const visualization::KeyboardEvent& event, void*) {
		string modeNames[2] = { "Delete", "Color" };
		if (event.getKeyCode()) {
			//cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode() << ") was";
		} else {
			//cout << "the special key \'" << event.getKeySym() << "\' was";
		}
		if (event.keyDown()) {
			//cout << " pressed" << endl;
		} else {
			if (strcmp ("m", event.getKeySym().c_str()) == 0) {				
				clickMode = (clickMode + 1) % 2;
				cout << "Clicks set to: " << modeNames[clickMode] << " Mode." << endl;
				cloud_viewer_->updateText(modeNames[clickMode] + " Mode", 10, 10, "modeText");
			}
			if (strcmp("c", event.getKeySym().c_str()) == 0) {
				showSegments = !showSegments;
				cout << "Show color segment set to: " << showSegments << endl;
				string segmentText = "False";
				if (showSegments) {
					segmentText = "True";
				}
				cloud_viewer_->updateText("Show Segments: " + segmentText, 10, 20, "segmentsText");
			}
			//cout << " released" << endl;
		}
		//delete modeNames;
    }
    
    /////////////////////////////////////////////////////////////////////////
    //void mouse_callback (const visualization::MouseEvent& mouse_event, void*)
    //{
      //if (mouse_event.getType() == visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == visualization::MouseEvent::LeftButton)
      //{
        //cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
      //}
    //}
	
    /////////////////////////////////////////////////////////////////////////
    /** \brief Point picking callback. This gets called when the user selects
      * a 3D point on screen (in the PCLVisualizer window) using Shift+click.
      *
      * \param[in] event the event that triggered the call
      */
    void pp_callback (const visualization::PointPickingEvent& event, void*) {
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
	  if (clickMode == 1) {
		  segment(picked_pt, indices[0], true, showSegments);
	  }
	  else {
		  segment(picked_pt, indices[0], false, showSegments);
	  }
    }

	////////////////////////////////// COLOR START ////////////////////////////////// 

	void alterColor(int* color) {
		if (*color > 255) {
			*color = 255;
		}
		else if (*color < 0) {
			*color = 0;
		}
	}

	void segment(const PointT &picked_point, int picked_idx, bool color, bool segment) {
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
		//cloud_ = clustering.getColoredCloud();
		if (segment) {
			cloud_ = clustering.getColoredCloud();
		}

		//print_highlight("Number of planar regions detected: %lu for a cloud of %lu points\n", clusters.size(), cloud_->size());
		int K = 1;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);

		pcl::KdTreeFLANN<pcl::PointXYZRGB> kt;
		kt.setInputCloud(cloud_);
		kt.nearestKSearch(picked_point, K, pointIdxNKNSearch, pointNKNSquaredDistance);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

			if (std::find(i->indices.begin(), i->indices.end(), pointIdxNKNSearch[0]) == i->indices.end())
			{
				for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
				{
					cluster->points.push_back(cloud_->points[*point]);
				}
				cluster->width = cluster->points.size();
				cluster->height = 1;
				cluster->is_dense = true;

				*newCloud = (*cluster) + (*newCloud);
			}
			else if (color)
			{
				//pcl::PointXYZRGB refPoint = picked_point;

				for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
				{
					int rDifference = picked_point.r - cloud_->points[*point].r;
					int gDifference = picked_point.g - cloud_->points[*point].g;
					int bDifference = picked_point.b - cloud_->points[*point].b;

					int rNew = 250 - rDifference;
					int gNew = 15 - gDifference;
					int bNew = 15 - bDifference;

					alterColor(&rNew);
					alterColor(&gNew);
					alterColor(&bNew);

					cloud_->points[*point].r = rNew;
					cloud_->points[*point].g = gNew;
					cloud_->points[*point].b = bNew;

					cluster->points.push_back(cloud_->points[*point]);
				}

				cluster->width = cluster->points.size();
				cluster->height = 1;
				cluster->is_dense = true;

				*newCloud = (*cluster) + (*newCloud);
			}
		}

		cloud_viewer_->removePointCloud("scene");
		cloud_viewer_->addPointCloud(newCloud, "scene");
		cloud_ = newCloud;

	}

	///////////////////////////////// COLOR END ////////////////////////////////// 


    /////////////////////////////////////////////////////////////////////////
    void compute () {
      // Visualize the data
      while (!cloud_viewer_->wasStopped ()) {
        cloud_viewer_->spinOnce ();
        if (image_viewer_) {
          image_viewer_->spinOnce ();
          if (image_viewer_->wasStopped ())
            break;
        }
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }
    }
    
    /////////////////////////////////////////////////////////////////////////
    void initGUI () {
      cloud_viewer_.reset (new visualization::PCLVisualizer ("PointCloud"));

      //cloud_viewer_->registerMouseCallback (&ObjectSelection::mouse_callback, *this);
      cloud_viewer_->registerKeyboardCallback(&ObjectSelection::keyboard_callback, *this);
      cloud_viewer_->registerPointPickingCallback (&ObjectSelection::pp_callback, *this);
      cloud_viewer_->setPosition (0, 0);

      cloud_viewer_->addPointCloud (cloud_, "scene");
      cloud_viewer_->resetCameraViewpoint ("scene");
      cloud_viewer_->addCoordinateSystem (0.1, 0, 0, 0, "global");
	  cloud_viewer_->addText("Delete Mode", 10, 10, "modeText");
	  cloud_viewer_->addText("Show Segments: False", 10, 20, "segmentsText");
    }

    /////////////////////////////////////////////////////////////////////////
    bool load (const std::string &file) {
      // Load the input file
      TicToc tt; tt.tic ();
      print_highlight (stderr, "Loading "); 
      print_value (stderr, "%s ", file.c_str ());
      cloud_.reset (new PointCloud<PointT>);
      if (io::loadPCDFile (file, *cloud_) < 0) {
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
int main (int argc, char** argv) {  // Parse the command line arguments for .pcd files
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