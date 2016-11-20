//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
/* Copyright (c) Russell Gillette
 * December 2013
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 * to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/* == MeshUtils.h ==
 * 
 * Utility and helper functions that operate on meshes or mesh data without a
 * specified Mesh instance.
 */

/*
 * Edited by Glen Berseth @ www.fracturedplane.com
 */

#ifndef MESH_UTILS_H
#define MESH_UTILS_H

#include "Mesh.h"
#include "ACCLMesh.h"
#include "OBJLoader.h"

#include <string>
 
#define DEFAULT_END_FRAME 1000000

// load an Edit Mesh object from an OBJ file.
Mesh *loadEditMeshFromFile(string file_name)
{
    Mesh *m = NULL;

    // parse mesh
    ObjLoader obj_parser(file_name);
    //obj_parser.generateNormals(); // don't use normals atm so no point in doing this

    // gather data from the parser to construct mesh
    GLubyte *data = NULL;
    int *indices;
    int data_size, num_indices, num_attr;
    obj_attrib_info *parsed_attr_info;

    // export parser data
    // NOTE: data is owned by the obj_parser
    obj_parser.objExportGLSeparate(data_size, data, num_indices, indices, num_attr, parsed_attr_info);

    if (parsed_attr_info == NULL)
    {
        printf("Mesh Not Found: Failed to load\n");
        return NULL;
    }

    std::vector<double> xyzPositions;
    std::vector<std::size_t> triangleIndices;

    // populate indices
    for (int i = 0; i < num_indices; i++)
        triangleIndices.push_back(indices[i]);
    
    // populate vertices from byte array (so lots of pointer pushing)
    int attr_end = data_size;
    int v_stride = parsed_attr_info[0].data_stride;
    int v_offset = parsed_attr_info[0].data_offset;

    if (v_stride == 0)
    for (int i = 0; i < num_attr; i++)
    {
        int off = parsed_attr_info[i].data_offset;
        if (off < attr_end && off > v_offset)
            attr_end = off;
    }

    int      attrib_size = parsed_attr_info[0].attrib_size;
    int      elem_size = attrib_size / parsed_attr_info[0].num_comp;

    GLubyte *pAttrib = data;
    GLubyte *pEnd    = data + attr_end;

    // TODO: safety check on number of elements per attribute
    // (there should never be less than 3 but who knows)
    // if (sizeof(float) != elem_size)
    //    unhandled as of right now

    // not particularily safe...
    for (; pAttrib < pEnd; pAttrib += elem_size)
    {
        double tmp = (double)*(float*)pAttrib;
        xyzPositions.push_back(tmp);
    }

    m = new Mesh();
    m->init(xyzPositions, triangleIndices);

    return m;
}

ACCLMesh::ACCLMesh *loadACCLMeshFromFile(string file_name)
{
    ACCLMesh::ACCLMesh *m = NULL;

    // parse mesh
    ObjLoader obj_parser(file_name);
    //obj_parser.generateNormals(); // don't use normals atm so no point in doing this

    // gather data from the parser to construct mesh
    GLubyte *data = NULL;
    int *indices;
    int data_size, num_indices, num_attr;
    obj_attrib_info *parsed_attr_info;

    // export parser data
    // NOTE: data is owned by the obj_parser
    obj_parser.objExportGLSeparate(data_size, data, num_indices, indices, num_attr, parsed_attr_info);

    if (parsed_attr_info == NULL)
    {
        printf("Mesh Not Found: Failed to load\n");
        return NULL;
    }

    std::vector<double> xyzPositions;
    std::vector<std::size_t> triangleIndices;

    // populate indices
    for (int i = 0; i < num_indices; i++)
        triangleIndices.push_back(indices[i]);

    // populate vertices from byte array (so lots of pointer pushing)
    int attr_end = data_size;
    int v_stride = parsed_attr_info[0].data_stride;
    int v_offset = parsed_attr_info[0].data_offset;

    if (v_stride == 0)
    for (int i = 0; i < num_attr; i++)
    {
        int off = parsed_attr_info[i].data_offset;
        if (off < attr_end && off > v_offset)
            attr_end = off;
    }

    int      attrib_size = parsed_attr_info[0].attrib_size;
    int      elem_size = attrib_size / parsed_attr_info[0].num_comp;

    GLubyte *pAttrib = data;
    GLubyte *pEnd    = data + attr_end;

    // TODO: safety check on number of elements per attribute
    // (there should never be less than 3 but who knows)
    // if (sizeof(float) != elem_size)
    //    unhandled as of right now

    // not particularily safe...
    for (; pAttrib < pEnd; pAttrib += elem_size)
    {
        double tmp = (double)*(float*)pAttrib;
        xyzPositions.push_back(tmp);
    }

    m = new ACCLMesh::ACCLMesh();
    m->init(xyzPositions, triangleIndices);

    return m;
}

#endif // MESH_UTILS_H
