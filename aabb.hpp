#pragma once

#include <limits>

#include "vec3.hpp"
#include "ray.hpp"
#include "mesh.hpp"

// Axis-aligned bounding box.
class AABB
{
public:
    // Creates an inverted bounding box such that min/max operations work.
    AABB(void);
    // Creates and initializes a bounding box.
    AABB(Vec3f const& mi, Vec3f const& ma) : min(mi), max(ma) {}

    // Returns the AABB extent.
    Vec3f const& getAABBMin(void) const;
    Vec3f const& getAABBMax(void) const;

    // Sets the AABB extent.
    void setMin(Vec3f const& vec);
    void setMax(Vec3f const& vec);

    // Merge the AABB with another AABB.
    void merge(AABB const& aabb);
    // Merge the AABB with another vector.
    void merge(Vec3f const& vec);
    // Returns the longest axis, 0: x-axis, 1: y-axis, 2: z-axis.
    int getLongestAxis(void) const;
    // Returns true iff point is inside the AABB.
    bool inside(Vec3f const& point) const;
    // Returns whether a ray intersects the AABB.
    bool intersect(Ray const& ray) const;
    bool intersect(Ray const& ray, float maxDistance) const;

    float intersectDistance(Ray const& ray) const;
    float getSurface() const;
private:
    Vec3f min;
    Vec3f max;
};

/* ------------------------ Implementation ------------------------ */

inline AABB::AABB(void)
    : min(Vec3f(std::numeric_limits<float>::max()))
    , max(Vec3f(-std::numeric_limits<float>::max()))
{
}

inline void AABB::setMin(Vec3f const& vec)
{
    this->min = vec;
}

inline void AABB::setMax(Vec3f const& vec)
{
    this->max = vec;
}

inline Vec3f const& AABB::getAABBMin(void) const
{
    return this->min;
}

inline Vec3f const& AABB::getAABBMax(void) const
{
    return this->max;
}

inline float AABB::getSurface() const
{
	const Vec3f d = this->max - this->min;

	const float dx = d[0];
	const float dy = d[1];
	const float dz = d[2];
	return (dx*dy + dx*dz + dy*dz)*2.0f;
}
