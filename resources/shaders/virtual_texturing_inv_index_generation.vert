// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

uniform uint feedback_index_size;

layout(std430, binding = 4) buffer in_feedback_inv_index { uint in_feedback_inv_index_values[]; };

layout(location = 0) in uint position;

void main()
{
    if(gl_VertexID >= feedback_index_size)
    {
        return;
    }

    in_feedback_inv_index_values[position] = uint(gl_VertexID);
}
