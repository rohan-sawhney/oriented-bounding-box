#include "Edge.h"
#include "HalfEdge.h"
#include "Vertex.h"

double Edge::length() const
{
    Eigen::Vector3d a = he->vertex->position;
    Eigen::Vector3d b = he->flip->vertex->position;
    
    return (b-a).norm();
}