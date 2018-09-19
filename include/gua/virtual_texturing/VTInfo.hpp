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

#include <unordered_map>
//#include <lamure/vt/VTConfig.h>

//#include <lamure/vt/common.h>
//#include <lamure/vt/ren/CutUpdate.h>
//#include <lamure/vt/ren/CutDatabase.h>


#include <scm/gl_core/buffer_objects/buffer.h>
#include <scm/gl_core/texture_objects/texture_2d.h>
#include <scm/gl_util/data/imaging/texture_image_data.h>

namespace gua {



struct GUA_DLL VTInfo {
  //uint32_t texture_id_;
  std::unordered_map<std::size_t, uint16_t> gua_view_id_to_lamure_view_id_;
  uint16_t context_id_;

};



}


#endif //GUA_VTINFO_HPP