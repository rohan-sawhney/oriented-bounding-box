#ifndef FACE_H
#define FACE_H

#include "Types.h"

class Face {
public:
    // one of the halfedges associated with this face
    HalfEdgeIter he;
    
    // checks if this face lies on boundary
    bool isBoundary() const;
    
    // returns face area
    double area() const;
    
    // returns normal to face
    Eigen::Vector3d normal() const;
};

#endif
