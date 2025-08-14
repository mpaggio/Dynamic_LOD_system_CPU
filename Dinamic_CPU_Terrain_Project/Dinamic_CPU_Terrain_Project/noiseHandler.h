#pragma once
#include "lib.h"
#include "strutture.h"

float FractalBrownianMotion(float x, float y, int numOctaves);
vector<float> generateFBMData(int width, int height, int numOctaves);
float getHeightAt(float x, float z, float terrainSize, int texWidth, int texHeight);
vec3 getNormalAt(float x, float z, float terrainSize, int texWidth, int texHeight);