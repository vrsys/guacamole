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

#ifndef GUA_DISPLAY_DATA_HPP
#define GUA_DISPLAY_DATA_HPP

#include <gua/platform.hpp>
#include <chrono>

namespace gua
{
/**
 * Temporary class that holds data for the PostFXPass.
 */

class GUA_DLL DisplayData
{
  public:
    void set_physics_fps(float fps);

    float get_physics_fps() const;

  private:
    static float physics_fps_;

    static std::chrono::high_resolution_clock::time_point last_update_;
};

} // namespace gua

#endif // GUA_DISPLAY_DATA_HPP
