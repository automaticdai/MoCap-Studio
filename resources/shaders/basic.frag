#version 330 core

in vec3 vFragPos;
in vec3 vNormal;
in vec4 vColor;

out vec4 FragColor;

uniform vec3 uLightDir = vec3(0.3, 1.0, 0.5);
uniform vec3 uAmbient = vec3(0.3, 0.3, 0.3);

void main() {
    vec3 normal = normalize(vNormal);
    vec3 lightDir = normalize(uLightDir);

    float diff = max(dot(normal, lightDir), 0.0);
    vec3 lighting = uAmbient + diff * vec3(0.7, 0.7, 0.7);

    FragColor = vec4(vColor.rgb * lighting, vColor.a);
}
