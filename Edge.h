#ifndef EDGE_H
#define EDGE_H

#include "Types.h"

class Edge {
public:
    // one of the two half edges associated with this edge
    HalfEdgeIter he;
    
    double length() const;
};

#endif
