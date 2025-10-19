#version 330 core

layout (location=0) out vec4 color;


uniform vec4 uColor;
uniform vec2 uPos;

void main()
{
    if(length(gl_FragCoord.xy - uPos) < 10)
    {
        color = uColor;
    }
    else{
        color = vec4(0);
    }
}