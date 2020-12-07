#version 150

in float distanceToCamera;
out vec4 fragColor;

void main() {
  float base=16;
  float b = 32;
  float c = log(distanceToCamera/base)/log(b);
  fragColor = vec4(c, c , c, c);
}