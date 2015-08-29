#ifndef TYPES_H
#define TYPES_H

#include <stdlib.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>
#include "math.h"
#include <Eigen/Core>
#include <Eigen/Dense>

class Vertex;
class Edge;
class Face;
class HalfEdge;
class Mesh;
class MeshIO;

typedef std::vector<HalfEdge>::iterator HalfEdgeIter;
typedef std::vector<HalfEdge>::const_iterator HalfEdgeCIter;
typedef std::vector<Vertex>::iterator VertexIter;
typedef std::vector<Vertex>::const_iterator VertexCIter;
typedef std::vector<Edge>::iterator EdgeIter;
typedef std::vector<Edge>::const_iterator EdgeCIter;
typedef std::vector<Face>::iterator FaceIter;
typedef std::vector<Face>::const_iterator FaceCIter;
typedef std::vector<Eigen::Vector3d>::iterator VectorIter;
typedef std::vector<Eigen::Vector3d>::const_iterator VectorCIter;

#endif
