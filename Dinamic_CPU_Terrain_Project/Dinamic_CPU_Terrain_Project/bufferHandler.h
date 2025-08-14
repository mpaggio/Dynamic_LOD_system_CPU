#pragma once
#include "lib.h"
#include "strutture.h"

BufferPair INIT_SIMPLE_VERTEX_BUFFERS(vector<float> vertices);
MeshBuffers INIT_VERTEX_BUFFERS(const vector<Vertex>& vertices, const vector<unsigned int>& indices);
BufferPair INIT_SPHERE_BUFFERS(vector<vec3> instancePositions, vector<vec3> allCenters);
GLuint INIT_TRANSFORM_FEEDBACK_BUFFERS();
ModelBufferPair INIT_MODEL_BUFFERS();