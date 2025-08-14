#pragma once
#include "lib.h"
#include "strutture.h"

void loadModel(string modelPath, ModelState state);
void updateBoneTransforms(float animationTime, ModelState state);
void extractEmbeddedTextures(const string modelPath, const string& outputDirectory);
vector<vec3> getModelBoundingVolume();
vec3 getBoundingBoxBaseCenter();