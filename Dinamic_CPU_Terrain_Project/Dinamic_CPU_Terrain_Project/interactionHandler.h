#pragma once
#include "lib.h"
#include "strutture.h"

pair<vec3, float> process_input(GLFWwindow* window);
void cursor_position_callback(GLFWwindow* window, double xposIn, double yposIn);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);