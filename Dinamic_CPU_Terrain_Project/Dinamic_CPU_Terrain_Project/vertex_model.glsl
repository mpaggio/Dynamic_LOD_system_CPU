#version 460 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTextCoords;
layout(location = 3) in ivec4 aBoneIDs;
layout(location = 4) in vec4 aWeights;

out vec2 TextCoords;
out vec3 Normal;
out vec3 FragPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;
uniform mat4 bones[128]; // Numero massimo di bone supportati

void main() {
    mat4 skinMatrix =
        aWeights[0] * bones[aBoneIDs[0]] +
        aWeights[1] * bones[aBoneIDs[1]] +
        aWeights[2] * bones[aBoneIDs[2]] +
        aWeights[3] * bones[aBoneIDs[3]];

    vec4 skinnedPos = skinMatrix * vec4(aPos, 1.0);
    FragPos = vec3(model * skinnedPos);

    mat3 normalMatrix = transpose(inverse(mat3(model * skinMatrix)));
    Normal = normalize(normalMatrix * aNormal);

    TextCoords = aTextCoords;

    gl_Position = proj * view * vec4(FragPos, 1.0);
}