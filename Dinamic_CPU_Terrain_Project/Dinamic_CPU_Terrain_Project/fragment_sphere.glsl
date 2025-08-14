#version 460 core

in vec3 fragNormal;
in vec3 fragPos;

out vec4 FragColor;

struct PointLight {
    vec3 position;
    vec3 color;
    float power;
};

uniform vec3 ViewPos;
uniform PointLight light;

void main() {
    vec3 norm = normalize(fragNormal);
    vec3 lightDir = normalize(light.position - fragPos);

    // Ambient
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * light.color;

    // Diffuse
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * light.color;

    // Specular
    vec3 viewDir = normalize(ViewPos - fragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    float specularStrength = 0.5;
    vec3 specular = specularStrength * spec * light.color;

    vec3 baseColor = vec3(1.0, 1.0, 0.0); // giallo

    vec3 lighting = (ambient + diffuse + specular) * baseColor * light.power;

    FragColor = vec4(lighting, 1.0);
}
