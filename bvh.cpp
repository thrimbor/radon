#include <iostream>
#include <limits>
#include <stdexcept>
#include <list>

#include "visual.hpp"
#include "bvh.hpp"

void BVH::buildBVH(Mesh const& mesh)
{
    delete this->root;
    this->root = new BVHNode();
    std::vector<unsigned int> faceIDs(mesh.faces.size() / 3);
    for (std::size_t i = 0; i < faceIDs.size(); ++i)
        faceIDs[i] = i;
    root->insert(mesh, &faceIDs);
    this->convert(mesh);
}

void BVH::addAABBsToMesh(Mesh* mesh, int nrOfDots) const
{
    if (this->root == NULL)
        return;

    typedef std::list<BVHNode const*> NodeList;
    NodeList nodes;
    nodes.push_back(this->root);
    while (!nodes.empty())
    {
        BVHNode const* node = nodes.back();
        nodes.pop_back();
        sampleAABBToMesh(mesh, nrOfDots, node->aabb);
        if (node->left)
            nodes.push_back(node->left);
        if (node->right)
            nodes.push_back(node->right);
    }
}

int BVH::getMaxDepth() const
{
    if (this->root == NULL)
        return 0;

    int maxLeafDepth = 0;

    typedef std::list<std::pair<BVHNode const*, int> > NodeLevelList;
    NodeLevelList nodes;
    nodes.push_back(std::make_pair(this->root, 1));
    while (!nodes.empty())
    {
        BVHNode const* node = nodes.back().first;
        int const level = nodes.back().second;
        nodes.pop_back();
        if (node->left == NULL && node->right == NULL)
        {
            maxLeafDepth = std::max(maxLeafDepth, level);
        }
        else
        {
            if (node->left)
                nodes.push_back(std::make_pair(node->left, level + 1));
            if (node->right)
                nodes.push_back(std::make_pair(node->right, level + 1));
        }
    }

    return maxLeafDepth;
}

void BVH::printStatistics(std::ostream& out)
{
    if (this->root == NULL)
        return;

    int numInnerNodes = 0;
    int minLeafDepth = std::numeric_limits<int>::max();
    int maxLeafDepth = 0;
    int numLeafs = 0;

    typedef std::list<std::pair<BVHNode const*, int> > NodeLevelList;
    NodeLevelList nodes;
    nodes.push_back(std::make_pair(this->root, 1));
    while (!nodes.empty())
    {
        BVHNode const* node = nodes.back().first;
        int const level = nodes.back().second;
        nodes.pop_back();
        if (node->left == NULL && node->right == NULL)
        {
            minLeafDepth = std::min(minLeafDepth, level);
            maxLeafDepth = std::max(maxLeafDepth, level);
            numLeafs += 1;
        }
        else
        {
            numInnerNodes += 1;
            if (node->left)
                nodes.push_back(std::make_pair(node->left, level + 1));
            if (node->right)
                nodes.push_back(std::make_pair(node->right, level + 1));
        }
    }

    out << "  Number of inner nodes: " << numInnerNodes << std::endl;
    out << "  Number of leaf nodes: " << numLeafs << std::endl;
    out << "  Minimum leaf depth: " << minLeafDepth << std::endl;
    out << "  Maximum leaf depth: " << maxLeafDepth << std::endl;
}

bool BVH::convert(Mesh const& mesh)
{
    if (this->root == nullptr)
        return false;

    this->root->convert(gpuBVH, gpuTriangles, mesh);
    return true;
}
