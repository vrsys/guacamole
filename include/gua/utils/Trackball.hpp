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

#ifndef GUA_TRACKBALL_HPP
#define GUA_TRACKBALL_HPP

#include <gua/platform.hpp>

#include <scm/gl_core/math.h>

namespace gua {
  namespace utils {

    class GUA_DLL Trackball
    {
    public:

      enum button_type {
        left = 0x01,
        middle = 0x02,
        right = 0x03
      };

      enum state_type {
        pressed = 0x01,
        released = 0x02
      };

      Trackball(float zoom_factor = 1.0f, float shift_factor = 0.5f, float rotation_factor = 0.25f);

      /* virtual */ ~Trackball();

    public:

      scm::math::mat4     rotation() const;

      float               distance() const;
      float               shiftx() const;
      float               shifty() const;

      void                reset();

      int                 posx() const;
      int                 posy() const;

      void                mouse(enum button_type button, enum state_type state, int x, int y);
      void                motion(int x, int y);

    private: 

      // trackball configuration
      float mapping_zoom_;
      float mapping_shift_;
      float mapping_rotate_;

    private: // temporary state

      // current button state
      bool                    button_left_;
      bool                    button_middle_;
      bool                    button_right_;

      // current pixel position
      int                     mousepos_x_;
      int                     mousepos_y_;

      // current rotation 
      scm::math::mat4         rotation_euler_;

      // current distance
      float                   distance_;

      // current shift
      float                   shiftx_;
      float                   shifty_;
    };

  }
}

#endif  // GUA_TRACKBALL_HPP
