/*******************************************************************************
written by : Waqar Shahid Qureshi

*******************************************************************************/
/// @file 3dscan_obj_pcl.cpp
/// Sample utility for writing an html version of the mesh data for sharing.

#include "3dscan_mesh.h"
#include <fstream>
#include <windows.h>
#undef max
#undef min
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/io/io_exception.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>

using namespace pcl; 

// LoadObjFile demonstrates how to load an obj file from disk.
// It is defined in 3dscan_obj_reader.cpp
int LoadObjFile(std::wstring in_file, mesh_t& out_mesh);

// GenerateHtmlFromObj demonstrates how to convert a scanned mesh into a 
// viewable/sharable three.js format. 
// It is called from 3dscan.cpp
int loadPolygonFilelFromObj(std::wstring in_obj_filename, pcl::PolygonMesh& mesh_pcl, pcl::PointCloud<pcl::Normal>::Ptr normal_cloud)
{

            // Read the provided obj file into memory.
            mesh_t mesh_obj;
            { // Optimization to reduce/eliminate vector resizes on push_back()
                const size_t SIZE = 250000;
                mesh_obj.faces.reserve(SIZE);
                mesh_obj.verticies.reserve(SIZE);
            }
            if (!LoadObjFile(in_obj_filename, mesh_obj))
            {
				// size of vertices, normal vertices, and faces
                const size_t NVERTS = mesh_obj.verticies.size();
				const size_t NVN = mesh_obj.vn.size();
				const size_t NFACES = mesh_obj.faces.size();

                // Initialize the pcl mesh
				mesh_pcl.polygons.resize (0);
				mesh_pcl.cloud.data.clear ();
				mesh_pcl.cloud.width = mesh_pcl.cloud.height = 0;
				mesh_pcl.cloud.is_dense = true;

				// read the virtices first
				pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
				xyz_cloud->points.resize (NVERTS);
				xyz_cloud->width = static_cast<uint32_t> (xyz_cloud->points.size ());
				xyz_cloud->height = 1;
				xyz_cloud->is_dense = true;

				// color information
				pcl::PointCloud<pcl::RGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::RGB> ());
				rgb_cloud->points.resize (NVERTS);
				rgb_cloud->width = static_cast<uint32_t> (rgb_cloud->points.size ());
				rgb_cloud->height = 1;
				rgb_cloud->is_dense = true;

                //copy the data to pcl cloud
				for (size_t i = 0; i < NVERTS; i++)
                {
                    const vertex_t* pv = & mesh_obj.verticies[i];
					xyz_cloud->points[i].x = static_cast<float> (pv->x);
					xyz_cloud->points[i].y = static_cast<float> (pv->y);
					xyz_cloud->points[i].z = static_cast<float> (pv->z);
					rgb_cloud->points[i].r = (unsigned char)(pv->r * 255);
					rgb_cloud->points[i].g = (unsigned char)(pv->g * 255);
					rgb_cloud->points[i].b = (unsigned char)(pv->b * 255);
                }
				
				// vertices normals
				normal_cloud->points.resize (NVN);
				normal_cloud->width = static_cast<uint32_t> (normal_cloud->points.size ());
				normal_cloud->height = 1;
				normal_cloud->is_dense = true;
				
				for (size_t i = 0; i < NVN; ++i)
                {
                  
					const vertex_norm_t* pvn = & mesh_obj.vn[i];
                    normal_cloud->points[i].normal_x = static_cast<float> (pvn->x);
					normal_cloud->points[i].normal_y = static_cast<float> (pvn->y);
					normal_cloud->points[i].normal_z = static_cast<float> (pvn->z);
                }
				//put it in the mesh could
				pcl::toPCLPointCloud2 (*xyz_cloud, mesh_pcl.cloud);
				pcl::PCLPointCloud2 rgb_cloud2;
				pcl::toPCLPointCloud2 (*rgb_cloud, rgb_cloud2);
				pcl::PCLPointCloud2 aux;
				pcl::concatenateFields (rgb_cloud2, mesh_pcl.cloud, aux);
				mesh_pcl.cloud = aux;
				//pcl::PCLPointCloud2 normal_cloud2;
				//pcl::toPCLPointCloud2 (*normal_cloud, normal_cloud2);
				//pcl::concatenateFields (normal_cloud2, mesh_pcl.cloud, aux);
				//mesh_pcl.cloud = aux;

				//////////////////////////////////////////////////////////////////////////////////////////////////
				// Now handle the polygons a.k.a. faces
				mesh_pcl.polygons.resize (NFACES);
				
                for (size_t f = 0; f < NFACES; ++f)
                {
					mesh_pcl.polygons[f].vertices.resize(3);
					mesh_pcl.polygons[f].vertices[0] = static_cast<int> (mesh_obj.faces[f].vertex_indicies[0]);
					mesh_pcl.polygons[f].vertices[1] = static_cast<int> (mesh_obj.faces[f].vertex_indicies[1]);
					mesh_pcl.polygons[f].vertices[2] = static_cast<int> (mesh_obj.faces[f].vertex_indicies[2]);
                }
				return 0;
			}
			else
			{
				return 1;
			}

    
}
