#if 0

#version 400 core

layout (location = 0) in vec3 	position;
layout (location = 1) in float 	index;
layout (location = 2) in vec2 	tesscoord;

flat out vec3 	vPosition;
flat out int	vIndex;
flat out vec2 	vTessCoord;

void main()
{
	vPosition 	= position;
	vIndex 		= int(index);
	vTessCoord 	= tesscoord;
}

#endif
