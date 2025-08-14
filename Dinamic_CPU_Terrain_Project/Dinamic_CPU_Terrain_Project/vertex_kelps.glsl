#version 460 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

uniform float time;

out vec3 FragPos;
out vec3 Normal;

void main() {
    // Movimento oscillante semplice sull'asse X basato sulla posizione Y e il tempo
    float swayAmount = 0.1;
    float sway = sin(time + aPos.y * 5.0) * swayAmount;

    vec3 displacedPos = aPos + vec3(sway, 0.0, 0.0);

    FragPos = vec3(model * vec4(displacedPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;

    gl_Position = proj * view * vec4(FragPos, 1.0);
}
