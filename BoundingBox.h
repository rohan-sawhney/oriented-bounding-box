#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include "Types.h"

class BoundingBox {
public:
    // default constructor
    BoundingBox();
    
    // initialize with specified components
    BoundingBox(const Eigen::Vector3d& min0, const Eigen::Vector3d& max0);
    
    // initialize with specified components
    BoundingBox(const Eigen::Vector3d& p);
    
    // expand bounding box to include point/ bbox
    void expandToInclude(const Eigen::Vector3d& p);
    void expandToInclude(const BoundingBox& b);
    
    // return the max dimension
    int maxDimension() const;
    
    // check if bounding box and face intersect
    bool contains(const BoundingBox& boundingBox, double& dist) const;
    
    // computes axis aligned bounding box
    void computeAxisAlignedBox(std::vector<Vertex>& vertices);
    
    // computes oriented bounding box using principal component analysis
    void computeOrientedBox(std::vector<Vertex>& vertices);
        
    // member variables
    Eigen::Vector3d min;
    Eigen::Vector3d max;
    Eigen::Vector3d extent;
    std::vector<Eigen::Vector3d> orientedPoints;
    std::string type;
};

#endif 