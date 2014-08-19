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

#ifndef GUA_TRIMMEDSURFACE_CONVERTER_HPP
#define GUA_TRIMMEDSURFACE_CONVERTER_HPP

// header, system
#include <memory>

// header, external
#include <boost/noncopyable.hpp>

// header, project

namespace gua {

// forward declarations
class TrimmedBezierSurfaceObject;
class TrimmedNurbsSurfaceObject;
class TrimmedNurbsSurface;

class TrimmedSurfaceConverter : boost::noncopyable {
 public:  // ctors/dtor

  TrimmedSurfaceConverter();
  ~TrimmedSurfaceConverter();

 public:  // operators

  void convert(std::shared_ptr<TrimmedNurbsSurfaceObject> const& ns,
               std::shared_ptr<TrimmedBezierSurfaceObject> const& bs);

 private:  // methods

  void _convert(TrimmedNurbsSurface const& nurbssurface);
  void _fetch_task();

 private:  // attributes

  class impl_t;
  impl_t* _impl;

};

}  // namespace gua

#endif  // GUA_TRIMMEDSURFACE_CONVERTER_HPP
