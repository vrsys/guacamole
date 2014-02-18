
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#version 420 core

// input/output definitions ///////////////////////////////////////////////////////////////////////
in per_vertex {
    smooth vec2 value_range;
} v_in;

// input layout definitions ///////////////////////////////////////////////////////////////////////
//layout(early_fragment_tests) in;

// attribute layout definitions ///////////////////////////////////////////////////////////////////
layout(location = 0, index = 0) out vec4 out_int_color;

// uniform input definitions //////////////////////////////////////////////////////////////////////
layout(binding = 0) uniform sampler2D ca_map;
                    uniform int       preint_steps;
                    uniform float     op_correction;

void main()
{
    vec4    d     = vec4(0.0);
    float   s     = v_in.value_range.x;
    float   s_inc = (v_in.value_range.y - v_in.value_range.x) / float(preint_steps);

    for (int i = 0; i < preint_steps; ++i) {
        // get sample
        vec4 c = texture(ca_map, vec2(s, 0.0));
        s += s_inc;
        // opacity correction
        c.a = 1.0 - pow(1.0 - c.a, op_correction);
        // compositing
        float omda_sa = (1.0 - d.a) * c.a;
        d.rgb += omda_sa * c.rgb;
        d.a   += omda_sa;
    }

    // normalize
    d.rgb /= d.a;
    //d.rgb =  (d.a < 0.00001) ? d.rgb * 100000.0 : d.rgb / d.a;
    out_int_color = d;//vec4(v_in.value_range.xy, 0.0, 1.0);//vec4(1.0, 0.0, 0.0, 1.0);//
}
