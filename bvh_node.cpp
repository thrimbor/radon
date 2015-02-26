#include <limits>
#include <stdexcept>

#include "visual.hpp"
#include "bvh_node.hpp"
#include "bvh.hpp"

BVHNode::BVHNode()
{
    this->right = NULL;
    this->left = NULL;
}

BVHNode::~BVHNode()
{
    delete this->left;
    delete this->right;
}

void BVHNode::insert(Mesh const& mesh, std::vector<unsigned int>* faceIDs)
{
	// create aabb
	for (std::size_t i=0; i<faceIDs->size(); ++i)
	{
		this->aabb.merge(mesh.vertices[mesh.faces[(*faceIDs)[i] * 3 + 0]]);
		this->aabb.merge(mesh.vertices[mesh.faces[(*faceIDs)[i] * 3 + 1]]);
		this->aabb.merge(mesh.vertices[mesh.faces[(*faceIDs)[i] * 3 + 2]]);
	}

	// create leaf?
	if (faceIDs->size() <= MAX_LEAF_TRIANGLES)
    {
        for (std::size_t i = 0; i < faceIDs->size(); ++i)
        {
            Triangle tri(&mesh, (*faceIDs)[i]);
            this->triangles.push_back(tri);
        }
        return;
    }

	// SAH
	float minCosts = std::numeric_limits<float>::max();
	int bestAxis;
	std::size_t bestPosition=0;

	for (std::size_t axis=0; axis<3; ++axis)
	{
		std::vector<float> costs(faceIDs->size()-1, 0.0f);
		std::sort(faceIDs->begin(), faceIDs->end(), [&](unsigned int a, unsigned int b)
			{
				return Triangle(&mesh, a).getCentroid()[axis] < Triangle(&mesh, b).getCentroid()[axis];
			});

		AABB aabbLeft, aabbRight;
		aabbLeft.merge(mesh.vertices[mesh.faces[(*faceIDs)[0] * 3 + 0]]);
		aabbLeft.merge(mesh.vertices[mesh.faces[(*faceIDs)[0] * 3 + 1]]);
		aabbLeft.merge(mesh.vertices[mesh.faces[(*faceIDs)[0] * 3 + 2]]);
		aabbRight.merge(mesh.vertices[mesh.faces[(*faceIDs)[faceIDs->size()-1] * 3 + 0]]);
		aabbRight.merge(mesh.vertices[mesh.faces[(*faceIDs)[faceIDs->size()-1] * 3 + 1]]);
		aabbRight.merge(mesh.vertices[mesh.faces[(*faceIDs)[faceIDs->size()-1] * 3 + 2]]);

		for (std::size_t i=1; i<faceIDs->size(); ++i)
		{
			costs[i-1] += (i*aabbLeft.getSurface());
			costs[faceIDs->size()-1-i] += i * aabbRight.getSurface();

			aabbLeft.merge(mesh.vertices[mesh.faces[(*faceIDs)[i] * 3 + 0]]);
			aabbLeft.merge(mesh.vertices[mesh.faces[(*faceIDs)[i] * 3 + 1]]);
			aabbLeft.merge(mesh.vertices[mesh.faces[(*faceIDs)[i] * 3 + 2]]);

			aabbRight.merge(mesh.vertices[mesh.faces[(*faceIDs)[faceIDs->size()-i] * 3 + 0]]);
			aabbRight.merge(mesh.vertices[mesh.faces[(*faceIDs)[faceIDs->size()-i] * 3 + 1]]);
			aabbRight.merge(mesh.vertices[mesh.faces[(*faceIDs)[faceIDs->size()-i] * 3 + 2]]);
		}

		for (std::size_t i=0; i<costs.size(); ++i)
		{
			if (costs[i] < minCosts)
			{
				minCosts = costs[i];
				bestAxis = axis;
				bestPosition = i;
			}
		}
	}

	std::sort(faceIDs->begin(), faceIDs->end(), [&](unsigned int a, unsigned int b)
		{
			return Triangle(&mesh, a).getCentroid()[bestAxis] < Triangle(&mesh, b).getCentroid()[bestAxis];
		});

	std::vector<unsigned int> leftObjects, rightObjects;

	for (std::size_t i=0; i<faceIDs->size(); ++i)
    {
        if (i <= bestPosition)
			leftObjects.push_back((*faceIDs)[i]);
		else
			rightObjects.push_back((*faceIDs)[i]);
    }

    faceIDs->clear();

	this->left = new BVHNode();
	this->left->insert(mesh, &leftObjects);
	this->right = new BVHNode();
	this->right->insert(mesh, &rightObjects);

    Ray ray;
	ray.position = Vec3f(0.0f, 0.0f, 2.0f);
	ray.direction = (this->left->aabb.getAABBMax() + this->left->aabb.getAABBMin()) / 2.0f;
    const float leftDist = this->left->aabb.intersectDistance(ray);
    ray.direction = (this->right->aabb.getAABBMax() + this->right->aabb.getAABBMin()) / 2.0f;
    const float rightDist = this->right->aabb.intersectDistance(ray);

    if (rightDist<leftDist)
    {
		BVHNode* t = this->left;
		this->left = this->right;
		this->right = t;
	}
}

void BVHNode::convert (std::vector<GPUBVH>& gpuBVH, std::vector<GPUTriangle>& gpuTriangles, Mesh const& mesh) const
{
    GPUBVH bvh;
    bvh.aabb.max.x = this->aabb.getAABBMax()[0];
    bvh.aabb.max.y = this->aabb.getAABBMax()[1];
    bvh.aabb.max.z = this->aabb.getAABBMax()[2];
    bvh.aabb.min.x = this->aabb.getAABBMin()[0];
    bvh.aabb.min.y = this->aabb.getAABBMin()[1];
    bvh.aabb.min.z = this->aabb.getAABBMin()[2];
    bvh.numTriangles = this->triangles.size();

    if (!this->triangles.empty())
    {
        bvh.jump_nodes = gpuTriangles.size();
        for (std::size_t triangleCounter=0; triangleCounter<this->triangles.size(); ++triangleCounter)
        {
            GPUTriangle triangle;
            for (std::size_t point=0; point<3; ++point)
            {
                for (std::size_t vertex=0; vertex<3; ++vertex)
                {
                    triangle.points[point].s[vertex] = this->triangles[triangleCounter][point][vertex];
                    triangle.normals[point].s[vertex] = mesh.vnormals[mesh.faces[triangles[triangleCounter].faceID*3+point]][vertex];
                }
            }
            gpuTriangles.push_back(triangle);
        }
        gpuBVH.push_back(bvh);
        return;
    }

    std::size_t index = gpuBVH.size();
    gpuBVH.push_back(bvh);

	std::size_t left_index = gpuBVH.size();
    this->left->convert(gpuBVH, gpuTriangles, mesh);

	std::size_t right_index = gpuBVH.size();
    this->right->convert(gpuBVH, gpuTriangles, mesh);

	int jump_nodes = 1;
	if (gpuBVH[left_index].numTriangles == 0)
		jump_nodes += gpuBVH[left_index].jump_nodes;
	else
		jump_nodes++;

	if (gpuBVH[right_index].numTriangles == 0)
		jump_nodes += gpuBVH[right_index].jump_nodes;
	else
		jump_nodes++;

	gpuBVH[index].jump_nodes = jump_nodes;

    return;
}
