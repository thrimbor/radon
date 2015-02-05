#pragma once

#include "vec3.hpp"
#include "aabb.hpp"
#include "ray.hpp"
#include "mesh.hpp"

// Representation of a triangle which contains an intersection test
// and a few convenience functions.
class Triangle
{
public:
    Triangle(void);
    Triangle(Mesh const* mesh, unsigned int faceID);

    Vec3f getCentroid(void) const;
    Vec3f getAABBMin(void) const;
    Vec3f getAABBMax(void) const;
    AABB getAABB(void) const;

    Vec3f const& operator[] (int index) const;
    Vec3f& operator[] (int index);

//private:
    Mesh const* mesh;
    unsigned int faceID;
};

/* ------------------------ Implementation ------------------------ */

inline Triangle::Triangle(void)
    : mesh(NULL)
{
}

inline Triangle::Triangle(Mesh const* mesh, unsigned int faceID)
    : mesh(mesh), faceID(faceID)
{
}

inline Vec3f const& Triangle::operator[] (int index) const
{
    return this->mesh->vertices[this->mesh->faces[this->faceID * 3 + index]];
}
