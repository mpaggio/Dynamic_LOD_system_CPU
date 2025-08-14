#version 460 core

layout(location = 0) in vec3 aPos;

uniform mat4 Projection;
uniform mat4 View;

out vec3 TexCoords;

void main() {
    TexCoords = aPos;
    mat4 viewNoTranslation = mat4(mat3(View));
    gl_Position = Projection * viewNoTranslation * vec4(aPos, 1.0);
    gl_Position = gl_Position.xyww; // forza w per depth corretto
}