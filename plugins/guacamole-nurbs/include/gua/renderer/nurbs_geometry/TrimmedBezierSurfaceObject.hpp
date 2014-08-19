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

#ifndef GUA_TRIMMEDBEZIERSURFACEOBJECT_HPP
#define GUA_TRIMMEDBEZIERSURFACEOBJECT_HPP

// header, system

// header, external
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <gua/renderer/nurbs_geometry/tml/axis_aligned_boundingbox.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/TrimmedBezierSurface.hpp>
#include <gua/renderer/nurbs_geometry/TrimDomainSerializer.hpp>

namespace gua {

class TrimmedBezierSurfaceObject {
 public:  // enums, typedefs

  typedef TrimmedBezierSurface::value_type value_type;
  typedef TrimmedBezierSurface surface_type;
  typedef boost::shared_ptr<surface_type> surface_ptr;
  typedef std::set<surface_ptr> surface_container;
  typedef surface_container::iterator surface_iterator;
  typedef surface_container::const_iterator const_surface_iterator;

  typedef TrimmedBezierSurface::curve_type curve_type;
  typedef boost::shared_ptr<curve_type> curve_ptr;

  typedef surface_type::trimdomain_ptr trimdomain_ptr;

  typedef tml::bbox3d bbox_t;

 public:  // c'tor / d'tor

  TrimmedBezierSurfaceObject();
  ~TrimmedBezierSurfaceObject();

 public:  // methods

  void add(surface_ptr const& surface);
  void remove(surface_ptr const& surface);

  bbox_t bbox() const;

  std::size_t size() const;
  const_surface_iterator begin() const;
  const_surface_iterator end() const;

  void print(std::ostream& os) const;

 private:  // auxilliary methods

 private:  // data members

  /////////////////////////////////////////////////////////////////////////////
  // attributes
  /////////////////////////////////////////////////////////////////////////////
  surface_container _surfaces;
};

}  // namespace gua

#endif  // GUA_TRIMMEDBEZIERSURFACEOBJECT_HPP
