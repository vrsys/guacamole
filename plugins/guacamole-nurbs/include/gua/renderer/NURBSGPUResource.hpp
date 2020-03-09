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

#ifndef GUA_NURBS_GPU_RESOURCE_HPP
#define GUA_NURBS_GPU_RESOURCE_HPP

// guacamole headers
#include <gua/renderer/NURBS.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/detail/NURBSData.hpp>

// external headers
#include <scm/core/math.h>

#include <scm/gl_core/gl_core_fwd.h>
#include <scm/gl_core/data_types.h>
#include <scm/gl_core/state_objects.h>

#include <scm/gl_util/primitives/primitives_fwd.h>
#include <scm/gl_util/primitives/geometry.h>

#include <scm/core/platform/platform.h>
#include <scm/core/utilities/platform_warning_disable.h>

namespace gua
{
struct NURBSGPUResource : public PluginResource
{
    // array and texture buffers for adaptive tesselation
    struct surface_tesselation_buffer
    {
        scm::gl::vertex_array_ptr vertex_array;

        scm::gl::buffer_ptr vertex_buffer;
        scm::gl::buffer_ptr index_buffer;
        scm::gl::buffer_ptr hullvertexmap;
        scm::gl::buffer_ptr attribute_buffer;

        scm::gl::texture_buffer_ptr parametric_texture_buffer;
        scm::gl::texture_buffer_ptr domain_texture_buffer;
        scm::gl::texture_buffer_ptr obb_texture_buffer;
        scm::gl::texture_buffer_ptr attribute_texture_buffer;
    } _surface_tesselation_data;

    // texture buffers for trimming
    struct
    {
        scm::gl::texture_buffer_ptr partition_texture_buffer;
        scm::gl::texture_buffer_ptr contourlist_texture_buffer;
        scm::gl::texture_buffer_ptr curvelist_texture_buffer;
        scm::gl::texture_buffer_ptr curvedata_texture_buffer;
        scm::gl::texture_buffer_ptr pointdata_texture_buffer;
        scm::gl::texture_buffer_ptr preclassification_buffer;
    } _contour_trimming_data;
};
} // namespace gua
#endif // GUA_NURBS_GPU_RESOURCE_HPP
