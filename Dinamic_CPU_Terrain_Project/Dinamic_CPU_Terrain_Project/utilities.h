#pragma once
#include "lib.h"
#include "strutture.h"

float randomFloat(float min, float max);
vec3 randomPosition(float L);
float getHeightFromFBM(float u, float v, int width, int height, const vector<float>& fbmData);
void applyDisplacement(vector<Vertex>& vertices, int texWidth, int texHeight, const vector<float>& fbmData);
int lodFromDistance(float distance, float minDist, float maxDist, int minSubdiv, int maxSubdiv);