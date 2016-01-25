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

#ifndef GUA_PIPELINE_FACTORY_HPP
#define GUA_PIPELINE_FACTORY_HPP

// guacamole headers
#include <gua/renderer/PipelineDescription.hpp>

namespace gua {

class GUA_DLL PipelineFactory {
 public:

  enum Capabilities {
    NONE                              = 0,
    DRAW_BBOXES                       = 1 << 0,
    DRAW_TRIMESHES                    = 1 << 1,
    DRAW_PLODS                        = 1 << 2,
    DRAW_NURBS                        = 1 << 3,
    DRAW_VIDEO3D                      = 1 << 4,
    DRAW_TEXTURED_QUADS               = 1 << 5,
    DRAW_SCREEN_SPACE_TEXTURED_QUADS  = 1 << 6,
    DRAW_VOLUMES                      = 1 << 7,
    WARPING                           = 1 << 8,
    DEBUG_GBUFFER                     = 1 << 9,
    ABUFFER                           = 1 << 10,
    DEFAULT                           = DRAW_TRIMESHES | DRAW_TEXTURED_QUADS |
                                        DRAW_SCREEN_SPACE_TEXTURED_QUADS
  };

  static std::shared_ptr<PipelineDescription> make_pipeline(int caps = Capabilities::DEFAULT);


};

}
#endif  // GUA_PIPELINE_FACTORY_HPP
