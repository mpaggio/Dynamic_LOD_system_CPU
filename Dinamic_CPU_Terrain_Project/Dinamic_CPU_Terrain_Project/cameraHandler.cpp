#include "cameraHandler.h"

float cameraSpeed = 0.05f;
vec3 slide_vector;

extern int height, width;
extern ViewSetup SetupTelecamera;
extern PerspectiveSetup SetupProspettiva;

void INIT_CAMERA_PROJECTION(void) {
    //Impostazione della telecamera
    SetupTelecamera.position = vec3(0.0, 0.5, 30.0);
    SetupTelecamera.target = vec3(0.0, 0.0, 0.0);
    SetupTelecamera.direction = normalize(SetupTelecamera.target - SetupTelecamera.position);
    SetupTelecamera.upVector = vec3(0.0, 1.0, 0.0);

    //Imposto la proiezione prospettica
    SetupProspettiva.aspect = (GLfloat)width / (GLfloat)height;
    SetupProspettiva.fovY = 45.0f;
    SetupProspettiva.far_plane = 2000.0f;
    SetupProspettiva.near_plane = 0.1f;
}

void cameraUp(void) {
    SetupTelecamera.direction = SetupTelecamera.target - SetupTelecamera.position;
    slide_vector = normalize(cross(SetupTelecamera.direction, SetupTelecamera.upVector));
    vec3 upDirection = cross(SetupTelecamera.direction, slide_vector) * cameraSpeed;
    SetupTelecamera.position -= upDirection;
    SetupTelecamera.target -= upDirection;
}

void cameraDown(void) {
    SetupTelecamera.direction = SetupTelecamera.target - SetupTelecamera.position;
    slide_vector = normalize(cross(SetupTelecamera.direction, vec3(SetupTelecamera.upVector)));
    vec3 upDirection = cross(SetupTelecamera.direction, slide_vector) * cameraSpeed;
    SetupTelecamera.position += upDirection;
    SetupTelecamera.target += upDirection;
}

void cameraLeft(void) {
    SetupTelecamera.direction = SetupTelecamera.target - SetupTelecamera.position;
    slide_vector = cross(SetupTelecamera.direction, vec3(SetupTelecamera.upVector)) * cameraSpeed;
    SetupTelecamera.position -= slide_vector;
    SetupTelecamera.target -= slide_vector;
}

void cameraRight(void) {
    SetupTelecamera.direction = SetupTelecamera.target - SetupTelecamera.position;
    slide_vector = cross(SetupTelecamera.direction, vec3(SetupTelecamera.upVector)) * cameraSpeed;
    SetupTelecamera.position += slide_vector;
    SetupTelecamera.target += slide_vector;
}

void cameraForward(void) {
    SetupTelecamera.direction = SetupTelecamera.target - SetupTelecamera.position;
    SetupTelecamera.position += SetupTelecamera.direction * cameraSpeed;
    SetupTelecamera.target = SetupTelecamera.position + SetupTelecamera.direction;
}

void cameraBack(void) {
    SetupTelecamera.direction = SetupTelecamera.target - SetupTelecamera.position;
    SetupTelecamera.position -= SetupTelecamera.direction * cameraSpeed;
    SetupTelecamera.target = SetupTelecamera.position + SetupTelecamera.direction;
}