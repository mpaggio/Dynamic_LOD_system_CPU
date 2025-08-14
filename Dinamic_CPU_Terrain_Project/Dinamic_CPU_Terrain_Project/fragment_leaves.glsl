#version 460 core

in vec3 FragPos;
in vec3 Normal;

out vec4 FragColor;


uniform vec3 lightPos;
uniform vec3 viewPos;

void main() {
    // Colore base foglia verde con variazione casuale
    vec3 baseColor = vec3(0.1, 0.6, 0.1);  // verde scuro
    vec3 highlight = vec3(0.4, 0.8, 0.4);  // verde chiaro

    // Lighting semplice (diffuse + ambient)
    vec3 ambient = 0.3 * baseColor;

    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * highlight;

    // Combina
    vec3 result = ambient + diffuse;

    // Trasparenza per effetto foglia
    float alpha = 0.8;

    FragColor = vec4(result, alpha);
}
