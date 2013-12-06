
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#version 420 core
#extension GL_NV_gpu_shader5 : enable
// input/output definitions ///////////////////////////////////////////////////////////////////////
in per_vertex {
    smooth vec2 value_range;
} v_in;

// input layout definitions ///////////////////////////////////////////////////////////////////////
//layout(early_fragment_tests) in;

// attribute layout definitions ///////////////////////////////////////////////////////////////////
layout(location = 0, index = 0) out vec4 out_int_color;

// global constants ///////////////////////////////////////////////////////////////////////////////
const float sqrt_ttpi = 2.506628274631000502415765284811;

// uniform input definitions //////////////////////////////////////////////////////////////////////
uniform uvec2	  ca_map;
uniform int       gauss_steps;

sampler2D gua_get_float_sampler(in uvec2 handle) {
	return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

void main()
{
    //const float pi_f = 2.0 * asin(1.0);

    vec4    d     = vec4(0.0);

    float   mean  = v_in.value_range.x;
    float   dev   = v_in.value_range.y;

    float   s_inc = 1.0/float(gauss_steps);
    
    float s = 0.0;
    float n = 0.0;
    float e = 0.0;
    float gau = 0.0;
    
#if 1
    if(dev <= s_inc){
		d = texture(gua_get_float_sampler(ca_map), vec2(mean, 0.0));
        d.rgb *= d.a;
    }
    else
#endif
    {
    for (int i = 0; i < gauss_steps; ++i) {
            
            n   = dev * sqrt_ttpi;//sqrt(2.0 * pi_f);
            e   = exp(-1.0 * (((s - mean)*(s - mean))/(2.0 * dev * dev)));

            gau = 1.0/(n) * e;

            // get sample
			vec4 c = texture(gua_get_float_sampler(ca_map), vec2(s, 0.0));
            s += s_inc;

            // compositing        
            d.rgb += c.rgb * gau * c.a;
            d.a   += c.a * gau;
        }	
        d = d/255.0;
    }	
    // normalize

    out_int_color = d;    
}

/*
    gaussian_lut.reset(new float[table_size * table_size * 4]);

    for(unsigned dev = 0; dev != table_size; ++dev)
        for(unsigned mean = 0; mean != table_size; ++mean)
        {
            float dev_f = dev / 255.0f;
            float mean_f = mean / 255.0f;

            scm::math::vec4f color_alpha_weight(0.0,0.0,0.0,0.0);

            for(unsigned s = 0; s != table_size; ++s)
            {
                float s_f = s / 255.0f;

                float n = dev_f * scm::math::sqrt(2*scm::math::pi_f);

                float e = std::exp(-1 * scm::math::pow(s_f-mean_f, 2)/(2 * scm::math::pow(dev_f, 2)));

                float gau = 1.0f/(n) * e;

                //std::cerr << gau << std::endl;

                color_alpha_weight.r += combined_lut[s * 4 + 0] * gau * combined_lut[s * 4 + 3];
                color_alpha_weight.g += combined_lut[s * 4 + 1] * gau * combined_lut[s * 4 + 3];
                color_alpha_weight.b += combined_lut[s * 4 + 2] * gau * combined_lut[s * 4 + 3];
                color_alpha_weight.a += combined_lut[s * 4 + 3] * gau;

            }

            unsigned offset = (mean + dev * table_size) * 4;

            gaussian_lut[offset + 0] = color_alpha_weight.r / (float)table_size;
            gaussian_lut[offset + 1] = color_alpha_weight.g / (float)table_size;
            gaussian_lut[offset + 2] = color_alpha_weight.b / (float)table_size;
            gaussian_lut[offset + 3] = color_alpha_weight.a / (float)table_size;

        }
*/