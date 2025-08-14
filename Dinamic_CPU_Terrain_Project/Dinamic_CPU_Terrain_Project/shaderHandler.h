#pragma once
#include "lib.h"
#include "strutture.h"

string loadShaderSource(const char* filepath);
unsigned int compileShader(const char* path, GLenum type);
unsigned int createShaderProgram(const char* vs = "vertex.glsl", const char* tcs = "tess_control.glsl",
    const char* tes = "tess_eval.glsl", const char* gs = "geometry.glsl", const char* fs = "fragment.glsl");
unsigned int createSimpleShaderProgram(const char* vs, const char* fs);
unsigned int createTransformFeedbackShaderProgram(const char* vs, const char* tcs, const char* tes, const char* gs);