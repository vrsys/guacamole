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

#ifndef GUA_PLOD_SHARED_RESOURCES_HPP
#define GUA_PLOD_SHARED_RESOURCES_HPP

#include <map>

#include <gua/renderer/Lod.hpp>

#include <scm/gl_core/texture_objects.h>
namespace gua
{
struct GUA_LOD_DLL plod_shared_resources
{
    plod_shared_resources(){};

    enum class AttachmentID
    {
        DEPTH_PASS_LIN_DEPTH = 0,

        ACCUM_PASS_COLOR_RESULT = 1,
        ACCUM_PASS_NORMAL_RESULT = 2,
        ACCUM_PASS_PBR_RESULT = 3,
        ACCUM_PASS_WEIGHT_AND_DEPTH_RESULT = 4,

        LINKED_LIST_ACCUM_PASS_PBR_OUT = 5,
        LINKED_LIST_ACCUM_PASS_MIN_ES_DIST = 6,
        LINKED_LIST_ACCUM_PASS_DUMMY_ATTACHMENT = 7,

        LINKED_LIST_ACCUM_PASS_FRAG_COUNT = 8,

        LINKED_LIST_ACCUM_PASS_PBR_IMAGE = 9,
        LINKED_LIST_ACCUM_PASS_NORMAL_IMAGE = 10,

        LINKED_LIST_RESOLVE_PASS_COLOR_IMAGE = 11,
        LINKED_LIST_RESOLVE_PASS_NORMAL_IMAGE = 12,
        LINKED_LIST_RESOLVE_PASS_PBR_IMAGE = 13
    };

    enum class TextureBufferID
    {
        LINKED_LIST_BUFFER = 0
    };

    std::map<AttachmentID, scm::gl::texture_2d_ptr> attachments_;
    std::map<TextureBufferID, scm::gl::texture_buffer_ptr> tex_buffers_;
};

} // namespace gua

#endif // GUA_PLOD_SHARED_RESOURCES_HPP