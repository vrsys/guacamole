/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

// class header
#include <gua/renderer/GuaMethodsFactory.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/databases/Resources.hpp>

// external headers
#include <sstream>

namespace gua {

std::string const GuaMethodsFactory::get_sampler_casts() const {
    return R"(
        isampler2D gua_get_int_sampler(uvec2 handle) {
            return isampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
        }

        usampler2D gua_get_uint_sampler(uvec2 handle) {
            return usampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
        }

        sampler2D gua_get_float_sampler(uvec2 handle) {
            return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
        }

        sampler2D gua_get_double_sampler(uvec2 handle) {
            return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
        }

        sampler2DShadow  gua_get_shadow_sampler(uvec2 handle) {
            return sampler2DShadow(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
        }
    )";
}

std::string const GuaMethodsFactory::get_material_id() const {
    return R"(
        uint gua_get_material_id() {
            return texture2D(gua_get_uint_sampler(gua_uint_gbuffer_in_1[0]), gua_get_quad_coords()).x;
        }
    )";
}

std::string const GuaMethodsFactory::get_depth() const {
    return R"(
        float gua_get_depth(vec2 frag_pos) {
            return texture2D(gua_get_float_sampler(gua_depth_gbuffer_in), frag_pos).x * 2.0 - 1.0;
        }

        float gua_get_depth() {
            vec2 frag_pos = gua_get_quad_coords();
            return texture2D(gua_get_float_sampler(gua_depth_gbuffer_in), frag_pos).x * 2.0 - 1.0;
        }
    )";
}

std::string const GuaMethodsFactory::get_position() const {
    return R"(
        vec3 gua_get_position(vec2 frag_pos) {
            vec4 screen_space_pos = vec4(frag_pos * 2.0 - 1.0, gua_get_depth(frag_pos), 1.0);
            vec4 h = gua_inverse_projection_view_matrix * screen_space_pos;
            h /= h.w;
            return h.xyz;
        }

        vec3 gua_get_position() {
            return gua_get_position(gua_get_quad_coords());
        }
    )";
}

}
