#pragma once

#include <vector>

#include "mesh.hpp"
#include "triangle.hpp"
#include "ray.hpp"
#include "aabb.hpp"
#include "bvh_node.hpp"

// Bounding Volume Hierarchy (BVH) Interface.
class BVH
{
public:
    BVH(void);
    ~BVH(void);

    // Constructs the BVH from the given mesh.
    void buildBVH(Mesh const& mesh);

    int getMaxDepth() const;

    // Traverses the BVH, collects and prints statistics to the given stream.
    void printStatistics(std::ostream& out);

    // Samples the edges of AABBs for visualization and adds it to the mesh.
    // Warning: Can produce huge meshes.
    void addAABBsToMesh(Mesh* mesh, int nrOfDots) const;

    const std::vector<GPUBVH>& getGPUBVH() const;
    const std::vector<GPUTriangle>& getGPUTriangles() const;

private:
    bool convert(Mesh const& mesh);

    BVHNode* root;
    std::vector<GPUBVH> gpuBVH;
    std::vector<GPUTriangle> gpuTriangles;
};

/* ------------------------ Implementation ------------------------ */

inline BVH::BVH()
{
    this->root = NULL;
}

inline BVH::~BVH()
{
    delete this->root;
}

inline const std::vector<GPUBVH>& BVH::getGPUBVH() const
{
    return gpuBVH;
}

inline const std::vector<GPUTriangle>& BVH::getGPUTriangles() const
{
    return gpuTriangles;
}
