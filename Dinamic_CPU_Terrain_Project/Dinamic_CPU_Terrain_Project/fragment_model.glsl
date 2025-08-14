#version 460 core

in vec2 TextCoords;
in vec3 FragPos;
in vec3 Normal;

out vec4 color;

struct PointLight {
    vec3 position;
    vec3 color;
    float power;
};

uniform vec3 ViewPos;
uniform PointLight light;

uniform sampler2D modelTexture;

void main() {

    // Normale normalizzata
    vec3 norm = normalize(Normal);

    // Direzione della luce
    vec3 lightDir = normalize(light.position - FragPos);

    // Ambient
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * light.color;

    // Diffuse
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * light.color;

    // Specular
    vec3 viewDir = normalize(ViewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0); // shininess fisso
    float specularStrength = 0.5;
    vec3 specular = specularStrength * spec * light.color;

    //Sample della texture
    vec3 baseColor = texture(modelTexture, TextCoords).rgb;

    //Combinazione luce - texture
    vec3 lighting = (ambient + diffuse + specular) * baseColor * light.power;

    color = vec4(lighting, 1.0);
}