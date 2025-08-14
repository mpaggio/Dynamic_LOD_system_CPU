#version 460 core

in vec3 FragPos;
in vec3 Normal;

out vec4 FragColor;

uniform vec3 lightPos;
uniform vec3 viewPos;

void main() {
    // Colore base kelp verde-bluastro
    vec3 baseColor = vec3(0.05, 0.4, 0.3);
    vec3 highlight = vec3(0.2, 0.7, 0.6);

    // Lighting semplice (ambient + diffuse)
    vec3 ambient = 0.3 * baseColor;

    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * highlight;

    vec3 result = ambient + diffuse;

    float alpha = 0.9;

    FragColor = vec4(result, alpha);
}
