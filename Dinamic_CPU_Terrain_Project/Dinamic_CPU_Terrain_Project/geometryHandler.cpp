#include "geometryHandler.h"
#include "utilities.h"

#define M_PI 3.14159265358979323846

extern float terrainSize;


// --- SKYBOX --- //
vector<float> generateSkyboxCube() {
    vector<float> skyboxVertices = vector<float>{
        -1.0f,  1.0f, -1.0f,  // fronte
        -1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,

        -1.0f, -1.0f,  1.0f,  // retro
        -1.0f, -1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f,  1.0f,
        -1.0f, -1.0f,  1.0f,

         1.0f, -1.0f, -1.0f,  // destra
         1.0f, -1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,

        -1.0f, -1.0f,  1.0f,  // sinistra
        -1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f, -1.0f,  1.0f,
        -1.0f, -1.0f,  1.0f,

        -1.0f,  1.0f, -1.0f,  // alto
         1.0f,  1.0f, -1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
        -1.0f,  1.0f,  1.0f,
        -1.0f,  1.0f, -1.0f,

        -1.0f, -1.0f, -1.0f,  // basso
        -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f,  1.0f
    };

    return skyboxVertices;
}




// --- PLANE --- //
void generateTerrainBase(int division, float width, vector<Vertex>& vertices) {
    vertices.clear();

    float triangleSide = width / division;

    for (int row = 0; row <= division; row++) {
        for (int col = 0; col <= division; col++) {
            vec3 pos(col * triangleSide, 0.0f, row * -triangleSide);
            vec2 uv((float)col / division, (float)row / division);

            // Normale iniziale fissa verso l’alto
            vertices.push_back({ pos, vec3(0,1,0), uv });
        }
    }
}




// --- PLANE PATCHES --- //
vector<Patch> generatePatches(const vector<Vertex>& baseVertices, int division) {
    vector<Patch> patches;
    int rowLength = division + 1;

    for (int row = 0; row < division; ++row) {
        for (int col = 0; col < division; ++col) {
            const Vertex& TL = baseVertices[row * rowLength + col];
            const Vertex& TR = baseVertices[row * rowLength + col + 1];
            const Vertex& BR = baseVertices[(row + 1) * rowLength + col + 1];
            const Vertex& BL = baseVertices[(row + 1) * rowLength + col];

            patches.push_back({
                TL.pos, TR.pos, BR.pos, BL.pos,
                TL.uv,  TR.uv,  BR.uv,  BL.uv
                });
        }
    }

    return patches;
}



// LOD dinamico
void generatePatchMesh(const Patch& patch, int subdivTop, int subdivRight, int subdivBottom, 
    int subdivLeft, const vector<float>& fbmData, int texWidth, int texHeight, vector<Vertex>& outVertices, 
    vector<unsigned int>& outIndices, unsigned int& indexOffset) {

    float globalMinX = 0.0f;
    float globalMaxX = terrainSize;
    float globalMinZ = -terrainSize;
    float globalMaxZ = 0.0f;
    
    // Calcolo numero massimo di suddivisioni per generare griglia interna
    int maxSubdivX = std::max(subdivTop, subdivBottom);
    int maxSubdivY = std::max(subdivLeft, subdivRight);

    // Griglia interna
    vector<Vertex> gridVertices((maxSubdivX + 1) * (maxSubdivY + 1));
    for (int y = 0; y <= maxSubdivY; ++y) {
        float ty = float(y) / maxSubdivY;

        // Interpolazione verticale sui lati sinistro e destro
        vec3 leftEdge = mix(patch.v0, patch.v3, ty);
        vec3 rightEdge = mix(patch.v1, patch.v2, ty);

        for (int x = 0; x <= maxSubdivX; ++x) {
            float tx = float(x) / maxSubdivX;

            // Interpolazione orizzontale tra i lati sinistro e destro
            vec3 pos = mix(leftEdge, rightEdge, tx);

            // UV mapping semplice (puoi adattarlo)
            float u = (pos.x - globalMinX) / (globalMaxX - globalMinX);
            float v = (pos.z - globalMinZ) / (globalMaxZ - globalMinZ);
            vec2 uv(u, v);

            gridVertices[y * (maxSubdivX + 1) + x] = { pos, vec3(0, 1, 0), uv };
        }
    }

    // Applica displacement FBM ai vertici finali
    applyDisplacement(gridVertices, texWidth, texHeight, fbmData);

    // Genera indici triangoli
    for (int y = 0; y < maxSubdivY; ++y) {
        for (int x = 0; x < maxSubdivX; ++x) {
            int start = y * (maxSubdivX + 1) + x;

            outIndices.push_back(indexOffset + start);
            outIndices.push_back(indexOffset + start + 1);
            outIndices.push_back(indexOffset + start + maxSubdivX + 1);

            outIndices.push_back(indexOffset + start + 1);
            outIndices.push_back(indexOffset + start + maxSubdivX + 2);
            outIndices.push_back(indexOffset + start + maxSubdivX + 1);
        }
    }

    // Aggiungi vertici generati al vettore globale
    outVertices.insert(outVertices.end(), gridVertices.begin(), gridVertices.end());
    indexOffset += gridVertices.size();
}




// --- SPHERES --- //
vector<vec3> sphereCorners = {
    vec3(0,  1,  0),
    vec3(0,  -1,  0),
    vec3(0, 0,  1),
    vec3(0, 0,  -1),
    vec3(1,  0, 0),
    vec3(-1,  0, 0)
};

const int ottanteTriangles[8][3] = {
    {0, 2, 4}, // Ottante 1
    {0, 3, 4}, // Ottante 2
    {0, 3, 5}, // Ottante 3
    {0, 2, 5}, // Ottante 4
    {1, 2, 4}, // Ottante 5
    {1, 3, 4}, // Ottante 6
    {1, 3, 5}, // Ottante 7
    {1, 2, 5}  // Ottante 8
};

vector<Vertex> generateSphereCPU(const vec3& center, float radius, int latSegments, int lonSegments) {
    vector<Vertex> vertices;
    for (int lat = 0; lat <= latSegments; ++lat) {
        float theta = lat * M_PI / latSegments; // da 0 a pi
        float sinTheta = sin(theta);
        float cosTheta = cos(theta);

        for (int lon = 0; lon <= lonSegments; ++lon) {
            float phi = lon * 2 * M_PI / lonSegments; // da 0 a 2pi
            float sinPhi = sin(phi);
            float cosPhi = cos(phi);

            vec3 normal = vec3(cosPhi * sinTheta, cosTheta, sinPhi * sinTheta);
            vec3 pos = center + radius * normal;
            vertices.push_back({ pos, normal });
        }
    }
    return vertices;
}


vector<unsigned int> generateSphereIndices(int latSegments, int lonSegments) {
    vector<unsigned int> indices;
    for (int lat = 0; lat < latSegments; ++lat) {
        for (int lon = 0; lon < lonSegments; ++lon) {
            int current = lat * (lonSegments + 1) + lon;
            int next = current + lonSegments + 1;

            // Primo triangolo
            indices.push_back(current);
            indices.push_back(next);
            indices.push_back(current + 1);

            // Secondo triangolo
            indices.push_back(current + 1);
            indices.push_back(next);
            indices.push_back(next + 1);
        }
    }
    return indices;
}


void generateGrassBlade(const vec3& pos, const vec3& normal, vector<Vertex>& vertices, vector<unsigned int>& indices, unsigned int& baseIndex) {
    // Calcola tangente arbitrario ortogonale alla normale
    vec3 tangent = normalize(cross(normal, vec3(0, 0, 1)));
    if (length(tangent) < 0.001f)
        tangent = vec3(1, 0, 0);

    // Angolo random (usa std::rand o meglio una funzione random con seme)
    float angle = ((float)rand() / RAND_MAX - 0.5f) * 2.0f * radians(20.0f);

    vec3 dir = normalize(tangent * cos(angle) + cross(normal, tangent) * sin(angle));

    float bladeHeight = 0.015f;
    float bladeWidth = 0.005f;

    vec3 baseLeft = pos - dir * bladeWidth * 0.5f;
    vec3 baseRight = pos + dir * bladeWidth * 0.5f;
    vec3 tip = pos + normal * bladeHeight;

    // Crea i vertici
    vertices.push_back({ baseLeft, normal, vec2(0, 0) });
    vertices.push_back({ baseRight, normal, vec2(1, 0) });
    vertices.push_back({ tip, normal, vec2(0.5f, 1) });

    // Crea un triangolo
    indices.push_back(baseIndex);
    indices.push_back(baseIndex + 1);
    indices.push_back(baseIndex + 2);

    baseIndex += 3;
}


void generateKelp(const vec3& baseCenter, vector<Vertex>& vertices, vector<unsigned int>& indices, 
    unsigned int& baseIndex, float u_time, int kelpSegments, float kelpWidth, float kelpSegmentLength, float kelpOscStrength) {
    
    vec3 up(0.0f, 1.0f, 0.0f);
    vec3 tangent(1.0f, 0.0f, 0.0f);  // direzione base orizzontale X
    vec3 bitangent = normalize(cross(up, tangent));

    float baseAngle = 0.0f; // puoi modificarlo per variare la direzione iniziale

    vec3 p0 = baseCenter;
    vec3 dir = normalize(tangent * cos(baseAngle) + bitangent * sin(baseAngle) + up * 1.0f);
    vec3 side = normalize(cross(dir, up)) * kelpWidth * 0.5f;

    vec3 v0 = p0 - side;
    vec3 v1 = p0 + side;

    for (int i = 0; i < kelpSegments; ++i) {
        float phase = float(i) * 1.3f;
        float oscAngle = sin(u_time * 1.5f + phase) * radians(25.0f);

        vec3 oscillatedDir = normalize(dir + (tangent * cos(oscAngle) + bitangent * sin(oscAngle)) * kelpOscStrength);

        vec3 p1 = p0 + oscillatedDir * kelpSegmentLength;
        vec3 nextSide = normalize(cross(oscillatedDir, up)) * kelpWidth * 0.5f;

        vec3 v2 = p1 - nextSide;
        vec3 v3 = p1 + nextSide;

        // Normale primo triangolo
        vec3 normal1 = normalize(cross(v1 - v0, v2 - v0));
        // Normale secondo triangolo
        vec3 normal2 = normalize(cross(v3 - v1, v2 - v1));

        // Primo triangolo
        vertices.push_back({ v0, normal1 });
        vertices.push_back({ v1, normal1 });
        vertices.push_back({ v2, normal1 });
        indices.push_back(baseIndex);
        indices.push_back(baseIndex + 1);
        indices.push_back(baseIndex + 2);
        baseIndex += 3;

        // Secondo triangolo
        vertices.push_back({ v2, normal2 });
        vertices.push_back({ v1, normal2 });
        vertices.push_back({ v3, normal2 });
        indices.push_back(baseIndex);
        indices.push_back(baseIndex + 1);
        indices.push_back(baseIndex + 2);
        baseIndex += 3;

        p0 = p1;
        v0 = v2;
        v1 = v3;
        dir = oscillatedDir;
    }
}
