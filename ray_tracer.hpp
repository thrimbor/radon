#pragma once

#define __CL_ENABLE_EXCEPTIONS
#include "cl_1.1.hpp"

#include <vector>

#include "mesh.hpp"
#include "ray.hpp"
#include "triangle.hpp"
#include "bvh.hpp"

class RayTracer
{
public:
    // Structure with basic options for raytracer
    // - width : image witdh
    // - focalLength      : focal length of virtual camera
    // - nSuperSamples  : Number of Samples !per pixel!
    //                      Runtime basically explodes if you turn this too high
    // - smoothShading    : switch to turn on/off smooth shading
    // - ambientOcclusion : switch to turn on/off ambient occlusion
    // - aoMaxDistance    : Ambient Occlusion distance
    //                      Should be about 10% of the max scene dimension
    // - aoNumSamples     : Number of samples for each ambient occlusion
    //                      evaluation
    struct Options
    {
        int width;
        int height;
        float focalLength;
        int nSuperSamples;
        bool shading;
        bool ambientOcclusion;
        float aoMaxDistance;
        int aoNumSamples;
    };

public:
    // Constructor with options argument
    RayTracer(Options const& opts, BVH const& scene);

    // Traces the scene contained in a BVH sturcture and
    // writes the result into the 'image' vector
    void trace(std::vector<unsigned char>* image);

private:
    Options opts;

	cl::Context context;
	cl::CommandQueue queue;
	cl::Kernel primaryRayGenerator;
	cl::Kernel primaryRayTraverser;
	cl::Kernel reconstruct;
	cl::Kernel aoTraverser;
	cl::Buffer nodeBuffer;
	cl::Buffer triangleBuffer;
	cl::Buffer raybuffer;
	cl::Buffer intersectbuffer;
	cl::Buffer imageBuffer;
	cl::Buffer aoIntersectBuffer;
	cl::Buffer aoHitsBuffer;

	unsigned int totalSampleCount;
};
