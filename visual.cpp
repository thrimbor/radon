#include "visual.hpp"

void sampleLineToMesh(Mesh* mesh, int nrOfDots, Vec3f const& start, Vec3f const& end)
{
    Vec3f const diff = end - start;
    for (int i = 0; i < nrOfDots; ++i)
    {
        float const fraction = static_cast<float>(i) / static_cast<float>(nrOfDots - 1);
        Vec3f sample = start + diff * fraction;
        mesh->vertices.push_back(sample);
    }
}

void sampleAABBToMesh(Mesh* mesh, int nrOfDots, AABB const& aabb)
{
    Vec3f const aabb_min = aabb.getAABBMin();
    Vec3f const aabb_max = aabb.getAABBMax();

    Vec3f const a(aabb_min[0], aabb_min[1], aabb_min[2]);
    Vec3f const b(aabb_max[0], aabb_min[1], aabb_min[2]);
    Vec3f const c(aabb_max[0], aabb_max[1], aabb_min[2]);
    Vec3f const d(aabb_min[0], aabb_max[1], aabb_min[2]);
    Vec3f const e(aabb_min[0], aabb_min[1], aabb_max[2]);
    Vec3f const f(aabb_max[0], aabb_min[1], aabb_max[2]);
    Vec3f const g(aabb_max[0], aabb_max[1], aabb_max[2]);
    Vec3f const h(aabb_min[0], aabb_max[1], aabb_max[2]);

    sampleLineToMesh(mesh, nrOfDots, a, b);
    sampleLineToMesh(mesh, nrOfDots, a, e);
    sampleLineToMesh(mesh, nrOfDots, a, d);
    sampleLineToMesh(mesh, nrOfDots, f, e);
    sampleLineToMesh(mesh, nrOfDots, f, b);
    sampleLineToMesh(mesh, nrOfDots, f, g);
    sampleLineToMesh(mesh, nrOfDots, c, d);
    sampleLineToMesh(mesh, nrOfDots, c, g);
    sampleLineToMesh(mesh, nrOfDots, c, b);
    sampleLineToMesh(mesh, nrOfDots, h, d);
    sampleLineToMesh(mesh, nrOfDots, h, g);
    sampleLineToMesh(mesh, nrOfDots, h, e);
}

void sampleRayToMesh(Mesh* mesh, Ray const& ray, int nrOfDots, float length)
{
    Vec3f dx(0.1f, 0.0f, 0.0f);
    Vec3f dy(0.0f, 0.1f, 0.0f);
    Vec3f dz(0.0f, 0.0f, 0.1f);
    sampleLineToMesh(mesh, nrOfDots / 10, ray.position - dx, ray.position + dx);
    sampleLineToMesh(mesh, nrOfDots / 10, ray.position - dy, ray.position + dy);
    sampleLineToMesh(mesh, nrOfDots / 10, ray.position - dz, ray.position + dz);
    sampleLineToMesh(mesh, nrOfDots, ray.position, ray.position + ray.direction * length);
}
