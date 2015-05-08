/*******************************************************************************
INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2014-2015 Intel Corporation. All Rights Reserved.
*******************************************************************************/
/// @file 3dscan_obj_reader.cpp
/// Sample loader for PXC3DScan generated obj file
/// http://en.wikipedia.org/wiki/Wavefront_.obj_file

//This file read the obj file for generating the html file.

#include "3dscan_mesh.h"
#include <sstream>
#include <fstream>

#define TOKEN_VERTEX_POS L"v" // vertex pos, color
#define TOKEN_FACE L"f" // faces
#define TOKEN_VN L"vn" // vertex normals

// LoadObjFile demonstrates how to load an obj file from disk.

int LoadObjFile(std::wstring in_file, mesh_t& out_mesh)
{
    // Try to open the provided file
    std::wifstream fs;
    fs.open(in_file.c_str());
    if (fs.fail()) return -1;

    // Load the mesh from file
    std::wstring line;
    while (std::getline(fs, line))
    {
        std::wstringstream str_stream(line);
        std::wstring type_str;
        str_stream >> type_str;

        if (type_str == TOKEN_VERTEX_POS)
        {
            vertex_t v;
            str_stream >> v.x >> v.y >> v.z >> v.r >> v.g >> v.b;
            out_mesh.verticies.push_back(v);
        }
        else if (type_str == TOKEN_FACE)
        {
            face_t f;
            wchar_t separater_unused;
            long normal_index_unused;
            for (size_t i = 0; i < 3; i++)
            {
                str_stream >> f.vertex_indicies[i] >> separater_unused
                    >> separater_unused >> normal_index_unused;
                f.vertex_indicies[i] -= 1; // Convert into a zero based index
            }
            out_mesh.faces.push_back(f);
        }
		else if ( type_str == TOKEN_VN)
		{
			vertex_norm_t vn;
			str_stream >> vn.x >> vn.y >> vn.z;
			out_mesh.vn.push_back(vn);
		}
    }

    fs.close();

    return 0;
}
