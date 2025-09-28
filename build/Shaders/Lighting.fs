#version 330 core

layout (location=0) out vec4 color;

in vec3 FragPos;
in vec3 Normal;

uniform vec4 uColor;
uniform vec3 LightPos;

void main()
{
    float ambientStrength = 0.1;
    vec4 ambient = ambientStrength * uColor;

    vec3 norm = normalize(Normal);
    vec3 lighDir = normalize(LightPos - FragPos);
    float diff = max(dot(norm, lighDir), 0.0);
    vec4 diffuse = diff * uColor;

    color = diffuse + ambient;
}