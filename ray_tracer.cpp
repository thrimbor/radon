#include <iostream>
#include <sstream>
#include <fstream>

#define __CL_ENABLE_EXCEPTIONS
#include "cl_1.1.hpp"

#include "ray_tracer.hpp"

struct AoHitInfo
{
	cl_float3 origin;
	cl_float3 normal;
};

RayTracer::RayTracer(RayTracer::Options const& opts, BVH const& scene)
{
    this->opts = opts;
    this->totalSampleCount = opts.width*opts.height*opts.nSuperSamples*opts.nSuperSamples;

    std::vector<cl::Platform> platforms;
    cl::Platform::get(&platforms);

    std::vector<cl::Device> devices;
    platforms[0].getDevices(CL_DEVICE_TYPE_GPU, &devices);

    cl::Device device = devices[0];
    devices.clear();
    devices.push_back(device);

    context = cl::Context(devices, nullptr, nullptr, nullptr, nullptr);
    queue = cl::CommandQueue(context, device, 0, nullptr);

    // read the kernel source
    std::ifstream sourcefile("kernel.cl");
    std::string sourcecode;
    while (!sourcefile.eof())
    {
        char line[512];
        sourcefile.getline(line, 511);
        sourcecode += line;
        sourcecode += '\n';
    }

    std::stringstream compilerOptions;
    compilerOptions << "-D AA_SAMPLES=" << opts.nSuperSamples;
    compilerOptions << " -D STACK_SIZE=" << (2*scene.getMaxDepth());

    if (opts.shading) compilerOptions << " -D SHADING_ENABLED";

    compilerOptions << " -D AO_SAMPLES=" << opts.aoNumSamples;
    compilerOptions << " -D AO_MAXDISTANCE=" << opts.aoMaxDistance << "f";
    if (opts.ambientOcclusion) compilerOptions << " -D AO_ENABLED";

    compilerOptions << " -cl-single-precision-constant -cl-strict-aliasing -cl-mad-enable -cl-no-signed-zeros -cl-unsafe-math-optimizations -cl-finite-math-only -cl-fast-relaxed-math -Werror -cl-denorms-are-zero";
    std::string defineString = compilerOptions.str();

    const char* str = sourcecode.c_str();

    cl::Program::Sources clSources;
    clSources.push_back(std::pair<const char *, size_t>(str, sourcecode.length()));
    cl::Program program(context, clSources, nullptr);
    program.build(devices, defineString.c_str(), nullptr, nullptr);

    primaryRayGenerator = cl::Kernel(program, "primaryRayGenerator", nullptr);
    primaryRayTraverser = cl::Kernel(program, "primaryRayTraverser", nullptr);
    reconstruct = cl::Kernel(program, "reconstruct", nullptr);
    aoTraverser = cl::Kernel(program, "aoTraverser", nullptr);

    imageBuffer = cl::Buffer(context, CL_MEM_WRITE_ONLY, opts.height*opts.width, nullptr, nullptr);
    raybuffer = cl::Buffer(context, CL_MEM_READ_WRITE, sizeof(cl_float3)*totalSampleCount, nullptr, nullptr);

    primaryRayGenerator.setArg(0, opts.width);
    primaryRayGenerator.setArg(1, opts.height);
    primaryRayGenerator.setArg(2, opts.focalLength);
    primaryRayGenerator.setArg(3, raybuffer);

    std::vector<GPUBVH> gpuBVH = scene.getGPUBVH();
    std::vector<GPUTriangle> triangles = scene.getGPUTriangles();

    nodeBuffer = cl::Buffer(context, CL_MEM_READ_ONLY, sizeof(GPUBVH)*gpuBVH.size(), nullptr, nullptr);
    queue.enqueueWriteBuffer(nodeBuffer, CL_TRUE, 0, sizeof(GPUBVH)*gpuBVH.size(), gpuBVH.data(), nullptr, nullptr);

    triangleBuffer = cl::Buffer(context, CL_MEM_READ_ONLY, sizeof(GPUTriangle)*triangles.size(), nullptr, nullptr);
    queue.enqueueWriteBuffer(triangleBuffer, CL_TRUE, 0, sizeof(GPUTriangle)*triangles.size(), triangles.data(), nullptr, nullptr);

    intersectbuffer = cl::Buffer(context, CL_MEM_READ_WRITE, sizeof(cl_float)*totalSampleCount, nullptr, nullptr);

    primaryRayTraverser.setArg(0, raybuffer);
    primaryRayTraverser.setArg(1, nodeBuffer);
    primaryRayTraverser.setArg(2, triangleBuffer);
    primaryRayTraverser.setArg(3, intersectbuffer);

    if (opts.ambientOcclusion)
    {
        aoIntersectBuffer = cl::Buffer(context, CL_MEM_READ_WRITE, sizeof(AoHitInfo)*totalSampleCount, nullptr, nullptr);
        primaryRayTraverser.setArg(4, aoIntersectBuffer);

        cl_int* fb = new cl_int[totalSampleCount];
        for (unsigned int x=0; x<totalSampleCount; ++x) fb[x] = 0;
        aoHitsBuffer = cl::Buffer(context, CL_MEM_READ_WRITE|CL_MEM_COPY_HOST_PTR, sizeof(cl_int)*totalSampleCount, (void*)fb, nullptr);
        delete[] fb;

        aoTraverser.setArg(0, nodeBuffer);
        aoTraverser.setArg(1, triangleBuffer);
        aoTraverser.setArg(2, aoIntersectBuffer);
        aoTraverser.setArg(3, aoHitsBuffer);

        reconstruct.setArg(2, aoHitsBuffer);
    }

    reconstruct.setArg(0, intersectbuffer);
    reconstruct.setArg(1, imageBuffer);
}

void
RayTracer::trace(std::vector<unsigned char>* image)
{
    cl::NDRange workItems(opts.width*opts.height);

    queue.enqueueNDRangeKernel(primaryRayGenerator, cl::NullRange, workItems, cl::NullRange, nullptr, nullptr);

    cl::NDRange workItems2(totalSampleCount);
    queue.enqueueNDRangeKernel(primaryRayTraverser, cl::NullRange, workItems2, cl::NullRange, nullptr, nullptr);

    if (opts.ambientOcclusion)
    {
        cl::NDRange workItems3(totalSampleCount*opts.aoNumSamples);
        queue.enqueueNDRangeKernel(aoTraverser, cl::NullRange, workItems3, cl::NullRange, nullptr, nullptr);
    }

    queue.enqueueNDRangeKernel(reconstruct, cl::NullRange, workItems, cl::NullRange, nullptr, nullptr);

    image->reserve(opts.width*opts.height);
    queue.enqueueReadBuffer(imageBuffer, CL_TRUE, 0, opts.width*opts.height, image->data(), nullptr, nullptr);
}
