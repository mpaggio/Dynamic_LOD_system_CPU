#pragma once
#include "lib.h"

#define MAX_NUM_BONES_PER_VERTEX 4
#define ARRAY_SIZE_IN_ELEMENTS(arr) (sizeof(arr) / sizeof((arr)[0]))

enum ModelState {
    WALKING = 0,
    STANDING = 1
};

struct FeedbackData {
    vec3 pos;
    vec3 normal;
};

struct VertexBoneData {
    unsigned int boneIDs[MAX_NUM_BONES_PER_VERTEX] = { 0 };
    float weights[MAX_NUM_BONES_PER_VERTEX] = { 0.0f };

    void addBone(const unsigned int boneID, float boneWeight) {
        for (int i = 0; i < ARRAY_SIZE_IN_ELEMENTS(boneIDs); i++) {
            if (weights[i] == 0.0) {
                boneIDs[i] = boneID;
                weights[i] = boneWeight;
                return;
            }
        }
    }

    void normalize() {
        float total = 0.0f;
        for (int i = 0; i < MAX_NUM_BONES_PER_VERTEX; i++) {
            total += weights[i];
        }
        if (total > 0.0f) {
            for (int i = 0; i < MAX_NUM_BONES_PER_VERTEX; i++) {
                weights[i] /= total;
            }
        }
    }

};

typedef struct {
    mat4 offsetMatrix; // trasforma dal modello al sistema di riferimento dell'osso
    mat4 finalTransform; // verrà calcolata a runtime durante l'animazione
} BoneInfo;

typedef struct {
    vec3 position; // Posizione della camera nello spazio 3D
    vec3 target; // Punto verso cui la camera è puntata
    vec3 upVector; // Vettore che indica la direzione "up" della camera
    vec3 direction; // Vettore che indica la direzione di visione della camera
} ViewSetup;

//gestione proiezione
typedef struct {
    float fovY; // Campo visivo verticale in gradi
    float aspect; // Rapporto tra larghezza e altezza del viewport
    float near_plane; // Distanza del piano di clipping vicino
    float far_plane; // Distanza del piano di clipping lontano
} PerspectiveSetup;

//gestione buffer
typedef struct{
    unsigned int vao;
    unsigned int vbo;
    unsigned int centerVBO;
} BufferPair;

typedef struct {
    unsigned int vao;
    unsigned int vboPositions;
    unsigned int vboNormals;
    unsigned int vboTexCoords;
    unsigned int vboBoneIDs;
    unsigned int vboBoneWeights;
    unsigned int ebo;
} ModelBufferPair;

typedef struct {
    vec3 position;
    vec3 color;
    GLfloat power;
} pointLight;

struct Vertex {
    vec3 pos;
    vec3 normal;
    vec2 uv;
};

struct MeshBuffers {
    GLuint vao;
    GLuint vbo;
    GLuint ebo;
};

struct Patch {
    vec3 v0, v1, v2, v3; // TL, TR, BR, BL
    vec2 uv0, uv1, uv2, uv3;
};