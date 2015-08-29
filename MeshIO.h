#ifndef MESH_IO_H
#define MESH_IO_H

#include <fstream>
#include "Types.h"

class MeshData;

class MeshIO {
public:
    // reads data from obj file
    static bool read(std::ifstream& in, Mesh& mesh);
    
    // writes data in obj format
    static void write(std::ofstream& out, Mesh& mesh);
    
private:
    // reserves spave for mesh vertices, uvs, normals and faces
    static void preallocateMeshElements(const MeshData& data, Mesh& mesh);
    
    // sets index for vertices
    static  void indexVertices(Mesh& mesh);
    
    // checks if any vertex is not contained in a face
    static void checkIsolatedVertices(const Mesh& mesh);
    
    // checks if a vertex is non-manifold
    static void checkNonManifoldVertices(const Mesh& mesh);
    
    // builds the halfedge mesh
    static bool buildMesh(const MeshData& data, Mesh& mesh);
};

#endif
