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

#ifndef GUA_VTINFO_HPP
#define GUA_VTINFO_HPP

#include <gua/platform.hpp>

//#include <lamure/vt/VTConfig.h>

//#include <lamure/vt/common.h>
//#include <lamure/vt/ren/CutUpdate.h>
//#include <lamure/vt/ren/CutDatabase.h>


#include <scm/gl_core/buffer_objects/buffer.h>
#include <scm/gl_core/texture_objects/texture_2d.h>
#include <scm/gl_util/data/imaging/texture_image_data.h>

namespace vt {
  class CutUpdate;
}

namespace gua {



struct GUA_DLL VTInfo {
  uint32_t texture_id_;
  uint16_t view_id_;
  uint16_t context_id_;
  uint64_t cut_id_;
  ::vt::CutUpdate *cut_update_;

  scm::math::vec2ui physical_texture_size_;
  scm::math::vec2ui physical_texture_tile_size_;
  size_t size_feedback_;

  int32_t  *feedback_lod_cpu_buffer_;
  uint32_t *feedback_count_cpu_buffer_;

  scm::gl::buffer_ptr feedback_lod_storage_;
  scm::gl::buffer_ptr feedback_count_storage_;

  int toggle_visualization_;
  bool enable_hierarchy_;
};



}


#endif //GUA_VTINFO_HPP