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
 * $Id$
 *
 */

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <math.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

float default_search_radius = 0.025f;

void
printHelp (int, char **argv)
{
    print_error ("Syntax is: %s input.pcd output.vtk [optional_arguments]\n", argv[0]);
    print_info ("  where the optional arguments are:\n");
    print_info ("                     -search_radius X = Search Radius (default: ");
    print_value ("%f", default_search_radius);
    print_info (")\n")-;
}


void
triangulate (PointCloud<PointXYZ>::ConstPtr cloud, float search_radius, PolygonMesh &triangles)
{
    print_info ("Fast triangulation of point clouds.\n");

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    // gp3.setSearchRadius (0.025);
    gp3.setSearchRadius (search_radius);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
}

float
signedVolumeOfTriangle (pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3)
{
    float v321 = p3.x*p2.y*p1.z;
    float v231 = p2.x*p3.y*p1.z;
    float v312 = p3.x*p1.y*p2.z;
    float v132 = p1.x*p3.y*p2.z;
    float v213 = p2.x*p1.y*p3.z;
    float v123 = p1.x*p2.y*p3.z;
    return (1.0f/6.0f)*(-v321 + v231 + v312 - v132 - v213 + v123);
}

float
volumeOfMesh (pcl::PolygonMesh mesh)
{
    print_info ("Computing volume of the mesh.\n");

    float vols = 0.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud,*cloud);

    for(int triangle=0;triangle<mesh.polygons.size();triangle++)
    {
        pcl::PointXYZ pt1 = cloud->points[mesh.polygons[triangle].vertices[0]];
        pcl::PointXYZ pt2 = cloud->points[mesh.polygons[triangle].vertices[1]];
        pcl::PointXYZ pt3 = cloud->points[mesh.polygons[triangle].vertices[2]];
        vols = vols + signedVolumeOfTriangle(pt1, pt2, pt3);
    }
    return abs(vols);
}

float
areaOfTriangle (pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3)
{
    // the area of the triangle is equal to half of cross-product
    // 0.5 * |u x v|
    // u == p1p2
    // v == p1p3

    float u[3];
    float v[3];
    float result[3];

    u[0] = p2.x - p1.x;
    u[1] = p2.y - p1.y;
    u[2] = p2.z - p1.z;

    v[0] = p3.x - p1.x;
    v[1] = p3.y - p1.y;
    v[2] = p3.z - p1.z;

    // compute cross product
    result[0] = u[1] * v[2] - u[2] * v[1];
    result[1] = u[2] * v[0] - u[0] * v[2];
    result[2] = u[0] * v[1] - u[1] * v[0];

    float norm = sqrt(pow(result[0], 2.0) + pow(result[1], 2.0) + pow(result[2], 2.0));

    // print_info ("point 1 %f %f %f\n", p1.x, p1.y, p1.z);
    // print_info ("point 2 %f %f %f\n", p2.x, p2.y, p2.z);
    // print_info ("point 3 %f %f %f\n", p3.x, p3.y, p3.z);
    // print_info ("vector 1 %f %f %f\n", u[0], u[1], u[2]);
    // print_info ("vector 2 %f %f %f\n", v[0], v[1], v[2]);
    // print_info ("result %f %f %f\n", result[0], result[1], result[2]);
    // print_info ("norm %f\n", norm);
    // print_info ("area %f\n", 0.5 * norm);
    // print_info ("-------------\n");
    return 0.5 * norm;
}

float
areaOfMesh (pcl::PolygonMesh mesh)
{
    print_info ("Computing area of the mesh.\n");

    float area = 0.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud,*cloud);

    for(int triangle=0;triangle<mesh.polygons.size();triangle++)
    {
        pcl::PointXYZ pt1 = cloud->points[mesh.polygons[triangle].vertices[0]];
        pcl::PointXYZ pt2 = cloud->points[mesh.polygons[triangle].vertices[1]];
        pcl::PointXYZ pt3 = cloud->points[mesh.polygons[triangle].vertices[2]];
        area = area + areaOfTriangle(pt1, pt2, pt3);
    }
    return area;
}


int
main (int argc, char** argv)
{
    print_info ("===============================================\n");
    print_info ("Surface Reconstruction and Extracting Features.\n");
    print_info ("===============================================\n");

    if (argc < 3)
    {
        printHelp (argc, argv);
        return (-1);
    }

    // Command line parsing
    float search_radius = default_search_radius;
    parse_argument (argc, argv, "-search_radius", search_radius);
    print_info ("Using a search radius of: "); print_value ("%f\n", search_radius);

    vector<int> pcd_file_indices;
    pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
    if (pcd_file_indices.size () != 1)
    {
        print_error ("Need one input PCD file to continue.\n");
        return (-1);
    }

    vector<int> vtk_file_indices;
    vtk_file_indices = parse_file_extension_argument (argc, argv, ".vtk");
    if (vtk_file_indices.size () != 1)
    {
        print_error ("Need one ouput VTK file to continue.\n");
        return (-1);
    }


    // Load in the point cloud
    PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());
    if (loadPCDFile (argv[pcd_file_indices[0]], *cloud_in) != 0)
    {
        print_error ("Could not load input file %s\n", argv[pcd_file_indices[0]]);
        return (-1);
    }

    PolygonMesh mesh_out;

    // fast triangulation
    triangulate (cloud_in, search_radius, mesh_out);

    // Save the mesh
    saveVTKFile (argv[vtk_file_indices[0]], mesh_out);

    // compute volume
    float volume;
    volume = volumeOfMesh(mesh_out);
    print_info ("Volume of object = %f.\n", volume);

    // compute area of surface
    float area;
    area = areaOfMesh(mesh_out);
    print_info ("Total surface area = %f.\n", area);


    return (0);
}
