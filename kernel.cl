#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics : enable

typedef struct
{
	float3 min;
	float3 max;
} AABB;

typedef struct
{
	float3 points[3];
	float3 normals[3];
} Triangle;

typedef struct
{
	float3 bary;
	float3 position;
	float distance;
	float3 normals[3];
} Intersection;

typedef struct
{
	AABB aabb;
	
	int numTriangles;
	
	int jump_nodes;
} Node;

// primary ray generator

__kernel void primaryRayGenerator (const uint width,
								   const uint height,
								   const float focalLength,
								   __global float3* rayDirections)
{
	const int id = get_global_id(0);
	const float a = focalLength * max(width, height);
	const float x = (id % width);
	const float y = (id / width);
	
	int r = id*AA_SAMPLES*AA_SAMPLES;
	for (int xSample=0; xSample<AA_SAMPLES; ++xSample)
	{
		for (int ySample=0; ySample<AA_SAMPLES; ++ySample)
		{
			rayDirections[r] = normalize((float3) (((x+(0.5f/AA_SAMPLES) + xSample*(1.0f/AA_SAMPLES)) / a - width / (2.0f * a)),
												  -((y+(0.5f/AA_SAMPLES) + ySample*(1.0f/AA_SAMPLES)) / a - height / (2.0f * a)),
													-1.0f));
			
			++r;
		}
	}
}

// primary ray traverser

inline float3 getSmoothNormal(const Intersection* intersection)
{
	return normalize(intersection->normals[0]*intersection->bary.x + intersection->normals[1]*intersection->bary.y + intersection->normals[2] * intersection->bary.z);
}

inline bool intersectAABB(const AABB* aabb,
						  const float3 origin,
						  const float3 invdir,
						  const float maxDistance)
{
	const float3 tMins = (aabb->min - origin)*invdir;
	const float3 tMaxs = (aabb->max - origin)*invdir;
	
	const float tmin = fmax(fmax(fmin(tMins.x, tMaxs.x), fmin(tMins.y, tMaxs.y)), fmin(tMins.z, tMaxs.z));
	const float tmax = fmin(fmin(fmax(tMins.x, tMaxs.x), fmax(tMins.y, tMaxs.y)), fmax(tMins.z, tMaxs.z));
	
	return (tmax >= tmin) && (tmin <= maxDistance);
}

inline void intersectTriangle(__global const Triangle* triangle,
							  const float3 origin,
							  const float3 direction,
							  Intersection* intersection)
{
	float const EPSILON2 = 0.000001f;

    // get triangle edge vectors and plane normal
    const float3 u = triangle->points[1] - triangle->points[0];
    const float3 v = triangle->points[2] - triangle->points[0];
    const float3 n = cross(u,v);              // cross product

    const float3 w0 = origin - triangle->points[0];
    const float a = -dot(n,w0);
    const float b = dot(n,direction);

    // Check if ray is parallel to triangle plane.
    if (fabs(b) < EPSILON2)
        return;

    // get intersect point of ray with triangle plane
    const float r = a / b;
    if (r < 0.0f) // ray goes away from triangle
        return;

    // intersect point of ray and plane
    const float3 intersectionPoint = origin + (direction * r);

    // is I inside T?
    const float uu = dot(u,u);
    const float uv = dot(u,v);
    const float vv = dot(v,v);
    const float3 w = intersectionPoint - triangle->points[0];
    const float wu = dot(w,u);
    const float wv = dot(w,v);
    const float D = native_recip(uv * uv - uu * vv);

    // get and test parametric coords
    const float s = (uv * wv - vv * wu) * D;
    if (s < -0.00001f || s > 1.00001f)         // I is outside T
        return;
    const float t = (uv * wu - uu * wv) * D;
    if (t < -0.00001f || (s + t) > 1.00001f)  // I is outside T
        return;

    // Intersection looks good. Fill result.
    const float distance = length(intersectionPoint - origin);
    if (intersection->distance > distance)
	{
		intersection->bary = (float3) (1.0f-s-t, s, t);
		intersection->position = intersectionPoint;
		intersection->distance = distance;
		intersection->normals[0] = triangle->normals[0];
		intersection->normals[1] = triangle->normals[1];
		intersection->normals[2] = triangle->normals[2];
	}
}

inline float shade (const float3 direction, const float3 normal)
{
	return fabs(dot(normal, direction));
}

typedef struct
{
	float3 origin;
	float3 normal;
} AoHitInfo;

__kernel void primaryRayTraverser (__global const float3* rayDirections,
								   __global const Node* nodes,
								   __global const Triangle* triangles,
								   __global float* shadeValues
								   #ifdef AO_ENABLED
									   ,__global AoHitInfo* aoHitInfo
								   #endif
								   )
{
	const int id = get_global_id(0);

	Intersection intersection;
	intersection.distance = MAXFLOAT;
	
	const float3 origin = (float3) (0.0f, 0.0f, 2.0f);
	const float3 direction = rayDirections[id];
	const float3 invdir = native_recip(direction);
	
	int currentOffset = 0;
	
	for (;;)
	{
		const Node node = nodes[currentOffset];
		
		if (intersectAABB(&node.aabb, origin, invdir, intersection.distance))
		{
			if (node.numTriangles == 0)
			{
				// internal node, step down to left
				++currentOffset;
			}
			else
			{
				// leaf node, check triangles
				int triangle = node.jump_nodes;
				intersectTriangle(&triangles[triangle++], origin, direction, &intersection);
				
				if (triangle<node.numTriangles+node.jump_nodes)
				{
					intersectTriangle(&triangles[triangle++], origin, direction, &intersection);
					
					if (triangle<node.numTriangles+node.jump_nodes)
					{
						intersectTriangle(&triangles[triangle++], origin, direction, &intersection);
						
						if (triangle<node.numTriangles+node.jump_nodes)
						{
							intersectTriangle(&triangles[triangle++], origin, direction, &intersection);
						}
					}
				}
				
				if (currentOffset+1 >= MAXNODES) break;
				++currentOffset;
			}
		}
		else
		{
			if (node.numTriangles > 0)
			{
				++currentOffset;
			}
			else
			{
				if (currentOffset+node.jump_nodes >= MAXNODES) break;
				currentOffset += node.jump_nodes;
			}
			
		}
		
	}
	
	if (intersection.distance < MAXFLOAT)
	{
		const float3 normal = getSmoothNormal(&intersection);
		
		#ifdef SHADING_ENABLED
			shadeValues[id] = shade(direction, normal);
		#else
			shadeValues[id] = 1.0f;
		#endif
		
		#ifdef AO_ENABLED
			aoHitInfo[id].normal = normal;
			aoHitInfo[id].origin = intersection.position + (normal*0.000001f);
		#endif
	}
	else
	{
		shadeValues[id] = 0.0f;
		
		#ifdef AO_ENABLED
			aoHitInfo[id].normal = (float3) (0.0f);
		#endif
	}
}

// image reconstructor

__kernel void reconstruct(__global const float* shadeValues,
						  __global uchar* dst
						  #ifdef AO_ENABLED
							, __global const int* hits
						  #endif
						  )
{
	const int id = get_global_id(0);
	
	float value = 0.0f;
	
	for (int sample=0; sample<AA_SAMPLES*AA_SAMPLES; ++sample)
	{
		#ifdef AO_ENABLED
			const float aoFactor = 1.0f - (((float)hits[id+sample]) / ((float)AO_SAMPLES));
			value += shadeValues[id*(AA_SAMPLES*AA_SAMPLES)+sample] * aoFactor;
		#else
			value += shadeValues[id*(AA_SAMPLES*AA_SAMPLES)+sample];
		#endif
	}
	
	dst[id] = (uchar) (255.0f * value/(AA_SAMPLES*AA_SAMPLES));
}

typedef struct
{
	uint4 rand;
	float3 basisX;
	float3 basisY;
	float3 basisZ;
} RandState;

inline uint rndInt(RandState* randState) {
	const uint t = randState->rand.x ^ (randState->rand.x << 11u);
	randState->rand.xyz = randState->rand.yzw;
	
	return randState->rand.w = randState->rand.w ^ (randState->rand.w >> 19) ^ (t ^ (t >> 8));
}

inline float rndFloat(RandState* randState)
{
	return 2.32830643653869629E-10f * rndInt(randState);
}

inline void rndInit(RandState* randState, const uint seed) {
	uint4 f = (uint4) (123456789u, 362436069u, 521288629u, 88675123u);
	uint4 g = (uint4) (88675123u, 123456789u, 362436069u, 521288629u);
	
	randState->rand.x = f.x ^ seed;
	randState->rand.y = f.y ^ seed;
	randState->rand.z = f.z ^ seed;
	randState->rand.w = f.w ^ seed;
	randState->rand *= g;
	
	rndInt(randState);
}

#define TWO_PI 6.283185307179586476925286766559f

inline void initHemisphereSampler (RandState* randState, const float3 normal, const int pid)
{
	randState->basisY = normal;
	float3 h = randState->basisY;
	
	if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
		h.x = 1.0f;
	else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
		h.y = 1.0f;
	else
		h.z = 1.0f;
	
	randState->basisX = normalize(cross(h, randState->basisY));
	randState->basisZ = normalize(cross(randState->basisX, randState->basisY));
	
	rndInit(randState, 536870923u * pid);
}

inline float3 getHemisphereSample(RandState* randState)
{
	const float xi1 = rndFloat(randState);
	const float xi2 = rndFloat(randState);
	
	const float theta = acos(native_sqrt(1.0f-xi1));
	const float phi = TWO_PI * xi2;
	
	const float xs = native_sin(theta) * native_cos(phi);
	const float ys = native_cos(theta);
	const float zs = native_sin(theta) * native_sin(phi);
	
	return ((float3) (randState->basisX*xs + randState->basisY*ys + randState->basisZ*zs));
}



inline bool aoIntersectTriangle(__global const Triangle* triangle,
							  const float3 origin,
							  const float3 direction)
{
	float const EPSILON2 = 0.000001f;

    // get triangle edge vectors and plane normal
    const float3 u = triangle->points[1] - triangle->points[0];
    const float3 v = triangle->points[2] - triangle->points[0];
    const float3 n = cross(u,v);              // cross product

    const float3 w0 = origin - triangle->points[0];
    const float a = -dot(n,w0);
    const float b = dot(n,direction);

    // Check if ray is parallel to triangle plane.
    if (fabs(b) < EPSILON2)
        return false;

    // get intersect point of ray with triangle plane
    const float r = a / b;
    if (r < 0.0f) // ray goes away from triangle
        return false;

    // intersect point of ray and plane
    const float3 intersectionPoint = origin + (direction * r);

    // is I inside T?
    const float uu = dot(u,u);
    const float uv = dot(u,v);
    const float vv = dot(v,v);
    const float3 w = intersectionPoint - triangle->points[0];
    const float wu = dot(w,u);
    const float wv = dot(w,v);
    const float D = native_recip(uv * uv - uu * vv);

    // get and test parametric coords
    const float s = (uv * wv - vv * wu) * D;
    if (s < -0.00001f || s > 1.00001f)         // I is outside T
        return false;
    const float t = (uv * wu - uu * wv) * D;
    if (t < -0.00001f || (s + t) > 1.00001f)  // I is outside T
        return false;

    // Intersection looks good. Fill result.
    if (length(intersectionPoint-origin) <= AO_MAXDISTANCE) return true;
    
    return false;
}

inline bool aoTraverse(__global const Node* nodes,
				__global const Triangle* triangles,
				const float3 origin,
				const float3 direction)
{
	const float3 invdir = native_recip(direction);
	
	int currentOffset = 0;
	
	for (;;)
	{
		const Node node = nodes[currentOffset];
		
		if (intersectAABB(&node.aabb, origin, invdir, AO_MAXDISTANCE))
		{
			// internal?
			if (node.numTriangles == 0)
			{
				++currentOffset;
			}
			else
			{
				int triangle = node.jump_nodes;
				if (aoIntersectTriangle(&triangles[triangle++], origin, direction)) return true;
				
				if (triangle<node.numTriangles+node.jump_nodes)
				{
					if (aoIntersectTriangle(&triangles[triangle++], origin, direction)) return true;
					
					if (triangle<node.numTriangles+node.jump_nodes)
					{
						if (aoIntersectTriangle(&triangles[triangle++], origin, direction)) return true;
						
						if (triangle<node.numTriangles+node.jump_nodes)
						{
							if (aoIntersectTriangle(&triangles[triangle++], origin, direction)) return true;
						}
					}
				}
				
				if (currentOffset+1 >= MAXNODES) break;
				++currentOffset;
			}
		}
		else
		{
			if (node.numTriangles > 0)
			{
				++currentOffset;
			}
			else
			{
				if (currentOffset+node.jump_nodes >= MAXNODES) break;
				currentOffset += node.jump_nodes;
			}
		}
	}
	
	return false;
}

__kernel void aoTraverser(__global const Node* nodes,
						  __global const Triangle* triangles,
						  __global const AoHitInfo* intersect,
						  __global int* hits)
{
	const int id = get_global_id(0) / AO_SAMPLES;
	
	RandState randState;
	initHemisphereSampler(&randState, intersect[id].normal, get_global_id(0));
	
	const float3 origin = intersect[id].origin;
	if (origin.x == 0.0f && origin.y == 0.0f && origin.z == 0.0f) return;
	
	const float3 direction = getHemisphereSample(&randState);
	
	if (aoTraverse(nodes, triangles, origin, direction)) atomic_inc(&hits[id]);
}
