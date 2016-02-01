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

// header
#include <gua/renderer/DisplayData.hpp>

//#include <boost/asio/high_resolution_timer.hpp>
//#include <boost/chrono.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

float DisplayData::physics_fps_ = 0.f;

boost::chrono::high_resolution_clock::time_point DisplayData::last_update_ =
    boost::chrono::high_resolution_clock::now();

////////////////////////////////////////////////////////////////////////////////

void DisplayData::set_physics_fps(float fps) {

  if (boost::chrono::duration_cast<boost::chrono::milliseconds>(
          boost::chrono::high_resolution_clock::now() - last_update_)
          .count() < 250)
    return;

  DisplayData::physics_fps_ = fps;

  last_update_ = boost::chrono::high_resolution_clock::now();
}

////////////////////////////////////////////////////////////////////////////////

float DisplayData::get_physics_fps() const { return DisplayData::physics_fps_; }

////////////////////////////////////////////////////////////////////////////////

}  // namespace gua