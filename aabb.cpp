#include "aabb.hpp"

#include <xmmintrin.h>

void AABB::merge(AABB const& bb)
{
    for (int i = 0; i < 3; ++i)
    {
        this->min[i] = std::min(this->min[i], bb.min[i]);
        this->max[i] = std::max(this->max[i], bb.max[i]);
    }
}

void AABB::merge(Vec3f const& vec)
{
    for (int i = 0; i < 3; ++i)
    {
        this->min[i] = std::min(this->min[i], vec[i]);
        this->max[i] = std::max(this->max[i], vec[i]);
    }
}

int AABB::getLongestAxis() const
{
    Vec3f diff = this->max - this->min;
    if (diff[0] >= diff[1] && diff[0] >= diff[2])
        return 0;
    if (diff[1] >= diff[0] && diff[1] >= diff[2])
        return 1;
    return 2;
}

bool AABB::inside(Vec3f const& point) const
{
    for (int i = 0; i < 3; ++i)
        if (point[i] > this->max[i] || point[i] < this->min[i])
            return false;
    return true;
}

bool AABB::intersect(Ray const& ray) const
{
	return this->intersect(ray, std::numeric_limits<float>::max());
}

static const __m128 one = _mm_set_ps(1.0f, 1.0f, 1.0f, 1.0f);

bool AABB::intersect(Ray const& ray, float maxDistance) const
{
	const __m128 box_min = _mm_set_ps(0.0f, min[2], min[1], min[0]);
	const __m128 box_max = _mm_set_ps(0.0f, max[2], max[1], max[0]);
	const __m128 origin = _mm_set_ps(0.0f, ray.position[2], ray.position[1], ray.position[0]);
	const __m128 inv_dir = _mm_div_ps(one, _mm_set_ps(1.0f, ray.direction[2], ray.direction[1], ray.direction[0]));

	const __m128 l1 = _mm_mul_ps(_mm_sub_ps(box_min, origin), inv_dir);
	const __m128 l2 = _mm_mul_ps(_mm_sub_ps(box_max, origin), inv_dir);

	__m128 lmax = _mm_max_ps(l1, l2);
	__m128 lmin = _mm_min_ps(l1, l2);

	const __m128 lmax0 = _mm_shuffle_ps(lmax, lmax, 0x39);
	const __m128 lmin0 = _mm_shuffle_ps(lmin, lmin, 0x39);
	const __m128 lmax1 = _mm_movehl_ps(lmax,lmax);
	const __m128 lmin1 = _mm_movehl_ps(lmin,lmin);

	lmax = _mm_min_ss(lmax, lmax0);
	lmin = _mm_max_ss(lmin, lmin0);
	lmax = _mm_min_ss(lmax, lmax1);
	lmin = _mm_max_ss(lmin, lmin1);

	float dist;
	_mm_store_ss(&dist, lmin);

	return (_mm_comige_ss(lmax, _mm_setzero_ps()) & _mm_comige_ss(lmax, lmin)) && (dist < maxDistance);
}

float AABB::intersectDistance(Ray const& ray) const
{
	const __m128 box_min = _mm_set_ps(0.0f, min[2], min[1], min[0]);
	const __m128 box_max = _mm_set_ps(0.0f, max[2], max[1], max[0]);
	const __m128 origin = _mm_set_ps(0.0f, ray.position[2], ray.position[1], ray.position[0]);
	const __m128 inv_dir = _mm_div_ps(one, _mm_set_ps(1.0f, ray.direction[2], ray.direction[1], ray.direction[0]));

	const __m128 l1 = _mm_mul_ps(_mm_sub_ps(box_min, origin), inv_dir);
	const __m128 l2 = _mm_mul_ps(_mm_sub_ps(box_max, origin), inv_dir);

	__m128 lmax = _mm_max_ps(l1, l2);
	__m128 lmin = _mm_min_ps(l1, l2);

	const __m128 lmax0 = _mm_shuffle_ps(lmax, lmax, 0x39);
	const __m128 lmin0 = _mm_shuffle_ps(lmin, lmin, 0x39);
	const __m128 lmax1 = _mm_movehl_ps(lmax,lmax);
	const __m128 lmin1 = _mm_movehl_ps(lmin,lmin);

	lmax = _mm_min_ss(lmax, lmax0);
	lmin = _mm_max_ss(lmin, lmin0);
	lmax = _mm_min_ss(lmax, lmax1);
	lmin = _mm_max_ss(lmin, lmin1);

	float dist;
	_mm_store_ss(&dist, lmin);

	if (_mm_comige_ss(lmax, _mm_setzero_ps()) & _mm_comige_ss(lmax, lmin))
		return dist;

	return std::numeric_limits<float>::max();
}

/*
bool AABB::intersect(Ray const& ray) const
{
    // Smits ray-box intersection test using slabs
    // http://www.cs.utah.edu/~awilliam/box/box.pdf
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    float div = 1.0f / ray.direction[0];
    if (div >= 0)
    {
        tmin = (this->min[0] - ray.position[0]) * div;
        tmax = (this->max[0] - ray.position[0]) * div;
    }
    else
    {
        tmin = (this->max[0] - ray.position[0]) * div;
        tmax = (this->min[0] - ray.position[0]) * div;
    }

    div = 1 / ray.direction[1];
    if (div >= 0)
    {
        tymin = (this->min[1] - ray.position[1]) * div;
        tymax = (this->max[1] - ray.position[1]) * div;
    }
    else
    {
        tymin = (this->max[1] - ray.position[1]) * div;
        tymax = (this->min[1] - ray.position[1]) * div;
    }

    if (tmin > tymax || tymin > tmax)
        return false;

    tmin = std::max(tmin, tymin);
    tmax = std::min(tmax, tymax);

    div = 1 / ray.direction[2];
    if (div >= 0)
    {
        tzmin = (this->min[2] - ray.position[2]) * div;
        tzmax = (this->max[2] - ray.position[2]) * div;
    }
    else
    {
        tzmin = (this->max[2] - ray.position[2]) * div;
        tzmax = (this->min[2] - ray.position[2]) * div;
    }

    if (tmin > tzmax || tzmin > tmax)
        return false;

    tmin = std::max(tmin, tzmin);
    tmax = std::min(tmax, tzmax);

    return tmin < 100000.0f && tmax > 0.0f;
}*/
/*
bool AABB::intersect(Ray const& ray, float maxDistance) const
{
    // Smits ray-box intersection test using slabs
    // http://www.cs.utah.edu/~awilliam/box/box.pdf
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    float div = 1.0f / ray.direction[0];
    if (div >= 0)
    {
        tmin = (this->min[0] - ray.position[0]) * div;
        tmax = (this->max[0] - ray.position[0]) * div;
    }
    else
    {
        tmin = (this->max[0] - ray.position[0]) * div;
        tmax = (this->min[0] - ray.position[0]) * div;
    }

    div = 1 / ray.direction[1];
    if (div >= 0)
    {
        tymin = (this->min[1] - ray.position[1]) * div;
        tymax = (this->max[1] - ray.position[1]) * div;
    }
    else
    {
        tymin = (this->max[1] - ray.position[1]) * div;
        tymax = (this->min[1] - ray.position[1]) * div;
    }

    if (tmin > tymax || tymin > tmax)
        return false;

    tmin = std::max(tmin, tymin);
    tmax = std::min(tmax, tymax);

    div = 1 / ray.direction[2];
    if (div >= 0)
    {
        tzmin = (this->min[2] - ray.position[2]) * div;
        tzmax = (this->max[2] - ray.position[2]) * div;
    }
    else
    {
        tzmin = (this->max[2] - ray.position[2]) * div;
        tzmax = (this->min[2] - ray.position[2]) * div;
    }

    if (tmin > tzmax || tzmin > tmax)
        return false;

    tmin = std::max(tmin, tzmin);
    tmax = std::min(tmax, tzmax);

    return tmin < 100000.0f && tmax > 0.0f && (std::min(tmax, tmin) < maxDistance);
}*/
