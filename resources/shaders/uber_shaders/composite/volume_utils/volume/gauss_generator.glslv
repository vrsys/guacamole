
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#version 420 core

// attribute layout definitions ///////////////////////////////////////////////////////////////////
layout(location = 0) in vec3 in_position;

// input/output definitions ///////////////////////////////////////////////////////////////////////
out per_vertex {
    smooth vec2 value_range;
} v_out;

// uniform input definitions //////////////////////////////////////////////////////////////////////
uniform mat4 mvp_matrix;

void main() {
    gl_Position       = mvp_matrix * vec4(in_position, 1.0);
    v_out.value_range = in_position.xy;
}
