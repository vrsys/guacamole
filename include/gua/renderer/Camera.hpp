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

#ifndef GUA_CAMERA_HPP
#define GUA_CAMERA_HPP

// external headers
#include <string>

namespace gua {

/**
 *  This struct describes a user's view on the scene.
 *
 *  It is defined by a screen, a view point a a render mask.
 */

struct Camera {
  Camera(std::string const& eye_l = "unknown_left_eye",
         std::string const& eye_r = "unknown_right_eye",
         std::string const& screen_l = "unknown_left_screen",
         std::string const& screen_r = "unknown_right_screen",
         std::string const& g = "scene_graph", std::string const& m = "")
      : eye_l(eye_l), eye_r(eye_r), screen_l(screen_l), screen_r(screen_r),
        scene_graph(g), render_mask(m) {}

  std::string eye_l;
  std::string eye_r;
  std::string screen_l;
  std::string screen_r;
  std::string scene_graph;
  std::string render_mask;
};

}

#endif  // GUA_CAMERA_HPP
