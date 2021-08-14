#pragma once

const char* shader_simple_v=R"""(
#version 330 core
layout(location = 0) in vec4 position;
layout(location = 1) in vec4 aColor;

out vec4 v_color;

uniform mat4 u_MVP;

void main()
{
  gl_Position = u_MVP * position;
  v_color = aColor;
};
)""";
const char* shader_simple_f=R"""(
#version 330 core
uniform vec4 u_Color;

layout(location = 0) out vec4 color;
in vec4 v_color;

void main()
{
	color = v_color;
};
)""";