#version 460 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

out vec3 fragNormal;
out vec3 fragPos;

void main() {
    vec4 worldPos = model * vec4(aPos.x, aPos.z, aPos.y, 1.0);
    fragPos = worldPos.xyz;
    fragNormal = mat3(transpose(inverse(model))) * aNormal; // Normale corretta se model include scale/rotazioni

    gl_Position = proj * view * worldPos;
}
