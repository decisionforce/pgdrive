#version 150

in float distanceToCamera;
in vec4 p3d_Color;
out vec4 fragColor;

void main() {
  fragColor = vec4(distanceToCamera/512, 0, 0, 1);
}