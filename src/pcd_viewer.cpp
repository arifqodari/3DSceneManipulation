/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */
// PCL
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <cfloat>
#include <pcl/visualization/eigen.h>
#include <pcl/visualization/vtk.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/histogram_visualizer.h>
#if VTK_MAJOR_VERSION==6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
#include <pcl/visualization/pcl_plotter.h>
#endif
#include <pcl/visualization/point_picking_event.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>
#include <vtkPolyDataReader.h>

using namespace pcl::console;

typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

typedef pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2> GeometryHandler;
typedef GeometryHandler::Ptr GeometryHandlerPtr;
typedef GeometryHandler::ConstPtr GeometryHandlerConstPtr;

bool isValidFieldName (const std::string &field) {
  if (field == "_")
    return (false);

  if ((field == "vp_x") || (field == "vx") || (field == "vp_y") || (field == "vy") || (field == "vp_z") || (field == "vz"))
    return (false);
  return (true);
}

bool isMultiDimensionalFeatureField (const pcl::PCLPointField &field) {
  if (field.count > 1)
    return (true);
  return (false);
}

bool isOnly2DImage (const pcl::PCLPointField &field) {
  if (field.name == "rgba" || field.name == "rgb")
    return (true);
  return (false);
}
  
void printHelp (int, char **argv) {
  print_error ("Syntax is: %s <file_name 1..N>.<pcd or vtk> <options>\n", argv[0]);
  print_info ("  where options are:\n");

  print_info ("                     -fc r,g,b                = foreground color\n");
  print_info ("                     -ps X                    = point size ("); print_value ("1..64"); print_info (") \n");
  print_info ("                     -opaque X                = rendered point cloud opacity ("); print_value ("0..1"); print_info (")\n");
  print_info ("                     -shading X               = rendered surface shading ("); print_value ("'flat' (default), 'gouraud', 'phong'"); print_info (")\n");
  print_info ("\n");
  print_info ("                     -immediate_rendering 0/1 = use immediate mode rendering to draw the data (default: "); print_value ("disabled"); print_info (")\n");
  print_info ("                                                Note: the use of immediate rendering will enable the visualization of larger datasets at the expense of extra RAM.\n");
  print_info ("                                                See http://en.wikipedia.org/wiki/Immediate_mode for more information.\n");
  print_info ("                     -vbo_rendering 0/1       = use OpenGL 1.4+ Vertex Buffer Objects for rendering (default: "); print_value ("disabled"); print_info (")\n");
  print_info ("                                                Note: the use of VBOs will enable the visualization of larger datasets at the expense of extra RAM.\n");
  print_info ("                                                See http://en.wikipedia.org/wiki/Vertex_Buffer_Object for more information.\n");
  print_info ("\n");
  print_info ("                     -use_point_picking       = enable the usage of picking points on screen (default "); print_value ("disabled"); print_info (")\n");
  print_info ("\n");
  print_info ("\n(Note: for multiple .pcd files, provide multiple -{fc,ps,opaque,position,orientation} parameters; they will be automatically assigned to the right file)\n");
}

// Global visualizer object
#if VTK_MAJOR_VERSION==6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
pcl::visualization::PCLPlotter ph_global;
#endif
boost::shared_ptr<pcl::visualization::PCLVisualizer> p;
std::vector<boost::shared_ptr<pcl::visualization::ImageViewer> > imgs;
pcl::search::KdTree<pcl::PointXYZ> search;
pcl::PCLPointCloud2::Ptr cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr xyzcloud;

void pp_callback (const pcl::visualization::PointPickingEvent& event, void* cookie) {
  int idx = event.getPointIndex ();
  if (idx == -1)
    return;

  if (!cloud) {
    cloud = *reinterpret_cast<pcl::PCLPointCloud2::Ptr*> (cookie);
    xyzcloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (*cloud, *xyzcloud);
    search.setInputCloud (xyzcloud);
  }
  // Return the correct index in the cloud instead of the index on the screen
  std::vector<int> indices (1);
  std::vector<float> distances (1);

  // Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
  pcl::PointXYZ picked_pt;
  event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);
  //TODO: Look into this.
  search.nearestKSearch (picked_pt, 1, indices, distances);

  PCL_INFO ("Point index picked: %d (real: %d) - [%f, %f, %f]\n", idx, indices[0], picked_pt.x, picked_pt.y, picked_pt.z);

  idx = indices[0];
  // If two points were selected, draw an arrow between them
  pcl::PointXYZ p1, p2;
  if (event.getPoints (p1.x, p1.y, p1.z, p2.x, p2.y, p2.z) && p) {
    std::stringstream ss;
    ss << p1 << p2;
    p->addArrow<pcl::PointXYZ, pcl::PointXYZ> (p1, p2, 1.0, 1.0, 1.0, ss.str ());
    return;
  }

  // Else, if a single point has been selected
  std::stringstream ss;
  ss << idx;
  // Get the cloud's fields
  for (size_t i = 0; i < cloud->fields.size (); ++i) {
    if (!isMultiDimensionalFeatureField (cloud->fields[i]))
      continue;
    PCL_INFO ("Multidimensional field found: %s\n", cloud->fields[i].name.c_str ());
#if VTK_MAJOR_VERSION==6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
    ph_global.addFeatureHistogram (*cloud, cloud->fields[i].name, idx, ss.str ());
    ph_global.renderOnce ();
#endif
  }
  if (p) {
    pcl::PointXYZ pos;
    event.getPoint (pos.x, pos.y, pos.z);
    p->addText3D<pcl::PointXYZ> (ss.str (), pos, 0.0005, 1.0, 1.0, 1.0, ss.str ());
  }  
}

/* ---[ */
int main (int argc, char** argv) {
  srand (static_cast<unsigned int> (time (0)));

  print_info ("The viewer window provides interactive commands; for help, press 'h' or 'H' from within the window.\n");

  if (argc < 2) {
    printHelp (argc, argv);
    return (-1);
  }

  bool debug = false;
  pcl::console::parse_argument (argc, argv, "-debug", debug);
  if (debug)
    pcl::console::setVerbosityLevel (pcl::console::L_DEBUG);

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices   = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> vtk_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".vtk");

  if (p_file_indices.size () == 0 && vtk_file_indices.size () == 0) {
    print_error ("No .PCD or .VTK file given. Nothing to visualize.\n");
    return (-1);
  }

  // Command line parsing
  double bcolor[3] = {0, 0, 0};

  std::vector<double> fcolor_r, fcolor_b, fcolor_g;
  bool fcolorparam = pcl::console::parse_multiple_3x_arguments (argc, argv, "-fc", fcolor_r, fcolor_g, fcolor_b);

  std::vector<int> psize;
  pcl::console::parse_multiple_arguments (argc, argv, "-ps", psize);

  std::vector<double> opaque;
  pcl::console::parse_multiple_arguments (argc, argv, "-opaque", opaque);

  std::vector<std::string> shadings;
  pcl::console::parse_multiple_arguments (argc, argv, "-shading", shadings);

  bool use_vbos = false;
  pcl::console::parse_argument (argc, argv, "-vbo_rendering", use_vbos);
  if (use_vbos) 
    print_highlight ("Vertex Buffer Object (VBO) visualization enabled.\n");

  bool use_pp   = pcl::console::find_switch (argc, argv, "-use_point_picking");
  if (use_pp) 
    print_highlight ("Point picking enabled.\n");

  // If VBOs are not enabled, then try to use immediate rendering
  bool use_immediate_rendering = false;
  if (!use_vbos) {
    pcl::console::parse_argument (argc, argv, "-immediate_rendering", use_immediate_rendering);
    if (use_immediate_rendering) 
      print_highlight ("Using immediate mode rendering.\n");
  }

  // Fix invalid multiple arguments
  if (psize.size () != p_file_indices.size () && psize.size () > 0)
    for (size_t i = psize.size (); i < p_file_indices.size (); ++i)
      psize.push_back (1);
  if (opaque.size () != p_file_indices.size () && opaque.size () > 0)
    for (size_t i = opaque.size (); i < p_file_indices.size (); ++i)
      opaque.push_back (1.0);
  if (shadings.size () != p_file_indices.size () && shadings.size () > 0)
    for (size_t i = shadings.size (); i < p_file_indices.size (); ++i)
      shadings.push_back ("flat");

  // Create the PCLVisualizer object
#if VTK_MAJOR_VERSION==6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
  boost::shared_ptr<pcl::visualization::PCLPlotter> ph;
#endif  
  // Using min_p, max_p to set the global Y min/max range for the histogram
  float min_p = FLT_MAX; float max_p = -FLT_MAX;

  int k = 0, l = 0, viewport = 0;
  // Load the data files
  pcl::PCDReader pcd;
  pcl::console::TicToc tt;
  ColorHandlerPtr color_handler;
  GeometryHandlerPtr geometry_handler;

  // Go through VTK files
  for (size_t i = 0; i < vtk_file_indices.size (); ++i) {
    // Load file
    tt.tic ();
    print_highlight (stderr, "Loading "); print_value (stderr, "%s ", argv[vtk_file_indices.at (i)]);
    vtkPolyDataReader* reader = vtkPolyDataReader::New ();
    reader->SetFileName (argv[vtk_file_indices.at (i)]);
    reader->Update ();
    vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput ();
    if (!polydata)
      return (-1);
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", polydata->GetNumberOfPoints ()); print_info (" points]\n");

    // Create the PCLVisualizer object here on the first encountered XYZ file
    if (!p)
      p.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PCD viewer"));

    // Add as actor
    std::stringstream cloud_name ("vtk-");
    cloud_name << argv[vtk_file_indices.at (i)] << "-" << i;
    p->addModelFromPolyData (polydata, cloud_name.str (), viewport);

    // Change the shape rendered color
    if (fcolorparam && fcolor_r.size () > i && fcolor_g.size () > i && fcolor_b.size () > i)
      p->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, fcolor_r[i], fcolor_g[i], fcolor_b[i], cloud_name.str ());

    // Change the shape rendered point size
    if (psize.size () > 0)
      p->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize.at (i), cloud_name.str ());

    // Change the shape rendered opacity
    if (opaque.size () > 0)
      p->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opaque.at (i), cloud_name.str ());

    // Change the shape rendered shading
    if (shadings.size () > 0) {
      if (shadings[i] == "flat") {
        print_highlight (stderr, "Setting shading property for %s to FLAT.\n", argv[vtk_file_indices.at (i)]);
        p->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_FLAT, cloud_name.str ());
      } else if (shadings[i] == "gouraud") {
        print_highlight (stderr, "Setting shading property for %s to GOURAUD.\n", argv[vtk_file_indices.at (i)]);
        p->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, cloud_name.str ());
      } else if (shadings[i] == "phong") {
        print_highlight (stderr, "Setting shading property for %s to PHONG.\n", argv[vtk_file_indices.at (i)]);
        p->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, cloud_name.str ());
      }
    }
  }

  pcl::PCLPointCloud2::Ptr cloud;
  // Go through PCD files
  for (size_t i = 0; i < p_file_indices.size (); ++i) {
    tt.tic ();
    cloud.reset (new pcl::PCLPointCloud2);
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int version;

    print_highlight (stderr, "Loading "); print_value (stderr, "%s ", argv[p_file_indices.at (i)]);

    if (pcd.read (argv[p_file_indices.at (i)], *cloud, origin, orientation, version) < 0)
      return (-1);

    std::stringstream cloud_name;
    // ---[ Special check for 1-point multi-dimension histograms
    if (cloud->fields.size () == 1 && isMultiDimensionalFeatureField (cloud->fields[0])) {
      cloud_name << argv[p_file_indices.at (i)];

#if VTK_MAJOR_VERSION==6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
      if (!ph)
        ph.reset (new pcl::visualization::PCLPlotter);
#endif

      pcl::getMinMax (*cloud, 0, cloud->fields[0].name, min_p, max_p);
#if VTK_MAJOR_VERSION==6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
      ph->addFeatureHistogram (*cloud, cloud->fields[0].name, cloud_name.str ());
#endif
      print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud->fields[0].count); print_info (" points]\n");
      continue;
    }

    // ---[ Special check for 2D images
    if (cloud->fields.size () == 1 && isOnly2DImage (cloud->fields[0])) {
      print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%u", cloud->width * cloud->height); print_info (" points]\n");
      print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (*cloud).c_str ());
      
      std::stringstream name;
      name << "PCD Viewer :: " << argv[p_file_indices.at (i)];
      pcl::visualization::ImageViewer::Ptr img (new pcl::visualization::ImageViewer (name.str ()));
      pcl::PointCloud<pcl::RGB> rgb_cloud;
      pcl::fromPCLPointCloud2 (*cloud, rgb_cloud);

      img->addRGBImage (rgb_cloud);
      imgs.push_back (img);

      continue;
    }

    cloud_name << argv[p_file_indices.at (i)] << "-" << i;

    // Create the PCLVisualizer object here on the first encountered XYZ file
    if (!p) {
      p.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PCD viewer"));
      if (use_pp)   // Only enable the point picking callback if the command line parameter is enabled
        p->registerPointPickingCallback (&pp_callback, static_cast<void*> (&cloud));

      // Set whether or not we should be using the vtkVertexBufferObjectMapper
      p->setUseVbos (use_vbos);

      if (!p->cameraParamsSet () && !p->cameraFileLoaded ()) {
        Eigen::Matrix3f rotation;
        rotation = orientation;
        p->setCameraPosition (origin [0]                  , origin [1]                  , origin [2],
                              origin [0] + rotation (0, 2), origin [1] + rotation (1, 2), origin [2] + rotation (2, 2),
                                           rotation (0, 1),              rotation (1, 1),              rotation (2, 1));
      }
    }

    if (cloud->width * cloud->height == 0) {
      print_error ("[error: no points found!]\n");
      return (-1);
    }

    // If no color was given, get random colors
    if (fcolorparam) {
      if (fcolor_r.size () > i && fcolor_g.size () > i && fcolor_b.size () > i)
        color_handler.reset (new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2> (cloud, fcolor_r[i], fcolor_g[i], fcolor_b[i]));
      else
        color_handler.reset (new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2> (cloud));
    }
    else
      color_handler.reset (new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2> (cloud));

    // Add the dataset with a XYZ and a random handler
    geometry_handler.reset (new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2> (cloud));
    // Add the cloud to the renderer
    p->addPointCloud (cloud, geometry_handler, color_handler, origin, orientation, cloud_name.str (), viewport);
	
    // Add every dimension as a possible color
    if (!fcolorparam) {
      int rgb_idx = 0;
      int label_idx = 0;
      for (size_t f = 0; f < cloud->fields.size (); ++f) {
        if (cloud->fields[f].name == "rgb" || cloud->fields[f].name == "rgba") {
          rgb_idx = f + 1;
          color_handler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2> (cloud));
        } else {
          if (!isValidFieldName (cloud->fields[f].name))
            continue;
          color_handler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (cloud, cloud->fields[f].name));
        }
        // Add the cloud to the renderer
        p->addPointCloud (cloud, color_handler, origin, orientation, cloud_name.str (), viewport);
      }
      // Set RGB color handler or label handler as default
      p->updateColorHandlerIndex (cloud_name.str (), (rgb_idx ? rgb_idx : label_idx));
    }

    // Additionally, add normals as a handler
    geometry_handler.reset (new pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<pcl::PCLPointCloud2> (cloud));
    if (geometry_handler->isCapable ())
      p->addPointCloud (cloud, geometry_handler, origin, orientation, cloud_name.str (), viewport);

    if (use_immediate_rendering)
      // Set immediate mode rendering ON
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0, cloud_name.str ());

    // Change the cloud rendered point size
    if (psize.size () > 0)
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize.at (i), cloud_name.str ());

    // Change the cloud rendered opacity
    if (opaque.size () > 0)
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opaque.at (i), cloud_name.str ());

    // Reset camera viewpoint to center of cloud if camera parameters were not passed manually and this is the first loaded cloud
    if (i == 0 && !p->cameraParamsSet () && !p->cameraFileLoaded ()) {
      p->resetCameraViewpoint (cloud_name.str ());
      p->resetCamera ();
    }

    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%u", cloud->width * cloud->height); print_info (" points]\n");
    print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (*cloud).c_str ());
    if (p->cameraFileLoaded ())
      print_info ("Camera parameters restored from %s.\n", p->getCameraFile ().c_str ());
  }

  if (p)
    p->setBackgroundColor (bcolor[0], bcolor[1], bcolor[2]);
  
  // Clean up the memory used by the binary blob
  // Note: avoid resetting the cloud, otherwise the PointPicking callback will fail
  // Only enable the point picking callback if the command line parameter is enabled
  if (!use_pp) {   
    cloud.reset ();
    xyzcloud.reset ();
  }

  // If we have been given images, create our own loop so that we can spin each individually
  if (!imgs.empty ()) {
    bool stopped = false;
    do {
#if VTK_MAJOR_VERSION==6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
      if (ph) ph->spinOnce ();
#endif
      for (int i = 0; i < int (imgs.size ()); ++i) {
        if (imgs[i]->wasStopped ()) {
          stopped = true;
          break;
        }
        imgs[i]->spinOnce ();
      }
      if (p) {
        if (p->wasStopped ()) {
          stopped = true;
          break;
        }
        p->spinOnce ();
      }
      boost::this_thread::sleep (boost::posix_time::microseconds (100));
    } while (!stopped);
  } else {
    // If no images, continue
#if VTK_MAJOR_VERSION==6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
    if (ph) {
      if (p)
        p->spin ();
      else
        ph->spin ();
    } else
#endif
      if (p)
        p->spin ();
  }
}
/* ]--- */