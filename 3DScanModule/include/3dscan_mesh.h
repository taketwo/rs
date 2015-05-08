/*******************************************************************************
INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2014-2015 Intel Corporation. All Rights Reserved.
*******************************************************************************/
/// @file 3dscan_mesh.h
/// 3D Capture (PXC3DScan) sample mesh structure

#include <vector>

struct vertex_t
{
    float x, y, z;
    float r, g, b;
};
struct vertex_norm_t
{
    float x, y, z;
};

struct face_t
{
    long vertex_indicies[3];
};

struct mesh_t
{
    std::vector<vertex_t> verticies;
    std::vector<face_t> faces;
	std::vector<vertex_norm_t> vn;
};
