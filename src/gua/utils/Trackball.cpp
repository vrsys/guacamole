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
#include <gua/utils/Trackball.hpp>

#include <cmath>

namespace gua {
namespace utils {

  ////////////////////////////////////////////////////////////////////////////////
  Trackball::Trackball(float zoom,
    float shift,
    float rotation)
    :
    button_left_(false),
    button_middle_(false),
    button_right_(false),
    mousepos_x_(0),
    mousepos_y_(0),
    rotation_euler_(),
    distance_(0.0),
    shiftx_(0.0),
    shifty_(0.0)
  {
    mapping_zoom_ = zoom;
    mapping_shift_ = shift;
    mapping_rotate_ = rotation;

    reset();
  }



  ////////////////////////////////////////////////////////////////////////////////
  /* virtual */ Trackball::~Trackball()
  {}


  ////////////////////////////////////////////////////////////////////////////////
  scm::math::mat4
  Trackball::rotation() const
  {
    return rotation_euler_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  float
  Trackball::distance() const
  {
    return distance_;
  }


  ////////////////////////////////////////////////////////////////////////////////
  float
  Trackball::shiftx() const
  {
    return shiftx_;
  }


  ////////////////////////////////////////////////////////////////////////////////
  float
  Trackball::shifty() const
  {
    return shifty_;
  }


  ////////////////////////////////////////////////////////////////////////////////
  void
  Trackball::reset()
  {
    button_left_ = false;
    button_middle_ = false;
    button_right_ = false;

    mousepos_x_ = 0;
    mousepos_y_ = 0;

    scm::math::set_identity(rotation_euler_);

    distance_ = 0.0;
    shiftx_ = 0.0;
    shifty_ = 0.0;
  }

  ////////////////////////////////////////////////////////////////////////////////
  int Trackball::posx() const
  {
    return mousepos_x_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  int Trackball::posy() const
  {
    return mousepos_y_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void Trackball::mouse(enum button_type button, enum state_type state, int x, int y)
  {
    mousepos_x_ = x;
    mousepos_y_ = y;

    switch (button)
    {
    case left:
      button_left_ = (state == pressed);
      break;
    case right:
      button_right_ = (state == pressed);
      break;
    case middle:
      button_middle_ = (state == pressed);
      break;
    }
  }


  ////////////////////////////////////////////////////////////////////////////////
  void Trackball::motion(int x, int y)
  {
    if (button_left_)
    {
      float angle_y = (mapping_rotate_ * float(x - mousepos_x_));
      float angle_x = (mapping_rotate_ * float(-y - mousepos_y_));

      rotation_euler_ = scm::math::make_rotation(angle_x, scm::math::vec3(1, 0, 0)) * rotation_euler_;
      rotation_euler_ = scm::math::make_rotation(angle_y, scm::math::vec3(0, 1, 0)) * rotation_euler_;
    }

    if (button_right_)
    {
      distance_ += mapping_zoom_ * float(-y - mousepos_y_);
    }

    if (button_middle_)
    {
      shiftx_ += mapping_shift_ * float(x - mousepos_x_);
      shifty_ -= mapping_shift_ * float(-y - mousepos_y_);
    }

    mousepos_y_ = -y;
    mousepos_x_ = x;
  }

}
}
