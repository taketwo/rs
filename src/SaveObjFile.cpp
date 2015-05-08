#include <fstream>
#include <iostream>
#include <pcl/common/io.h>

using namespace pcl; 

int 
 saveOBJFile (const std::string &file_name, 
                const pcl::PolygonMesh &poly_mesh, unsigned precision = 6) 
{ 
        if (poly_mesh.cloud.data.empty ()) 
        { 
                PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n"); 
                return (-1); 
        } 
        // Open file 
        std::ofstream fs; 
        fs.precision (precision); 
        fs.open (file_name.c_str ()); 

        /* Write 3D information */ 
        // number of points 
        int nr_points  = poly_mesh.cloud.width * poly_mesh.cloud.height; 

        // mesh size 
        int nr_meshes = poly_mesh.polygons.size(); 

        // Write the header information 
        fs << "####" << std::endl; 
        fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl; 
        fs << "# Vertices: " << nr_points << std::endl; 
        fs << "# Faces: " << nr_meshes << std::endl; 
        fs << "####" << std::endl; 

        // Write vertex coordinates 
        fs << "# Vertices" << std::endl; 

        PointCloud<PointXYZRGBNormal>::Ptr rgbNormalCloud (new PointCloud<PointXYZRGBNormal>); 
        fromROSMsg(poly_mesh.cloud, *rgbNormalCloud); 

        for (unsigned int i=0; i<rgbNormalCloud->points.size(); i++) 
        { 
                fs << "v " << rgbNormalCloud->points[i].x << " " << rgbNormalCloud->points[i].y << " " << rgbNormalCloud->points[i].z << " " << (double)((double)rgbNormalCloudrgbCloud->points[i].r/255.0f) << " "  << (double)((double)rgbNormalCloud->points[i].g/255.0f) << " " << (double)((double)rgbNormalCloud->points[i].b/255.0f) << std::endl; 
        } 
		fs <<"Number of normal vertex coordinates:" << std::endl; // need to fix the normal vertice size
		        for (unsigned int i=0; i<rgbNormalCloud->points.size(); i++) 
        { 
                fs << "vn " << rgbCloud->points[i].normal_x << " " << rgbCloud->points[i].normal_y << " " << rgbCloud->points[i].normal_y << std::endl; 
        } 

        fs << "# "<< nr_points <<" vertices" << std::endl; 

        for(int m = 0; m < nr_meshes; ++m){ 
                // Write faces with "f" 
                fs << "f "; 
                size_t j = 0; 
                for (j = 0; j < poly_mesh.polygons[m].vertices.size () - 1; ++j) 
                        fs << poly_mesh.polygons[m].vertices[j] +1 << " "; // vertex index in obj file format starting with 1 
                fs << poly_mesh.polygons[m].vertices[j]+1 <<  std::endl; 
        } 
        fs << "# End of File"; 

        // Close obj file 
        fs.close (); 
        return (0); 
} 
