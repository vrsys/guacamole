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

#ifndef GUA_TRIMMED_BEZIERSURFACE_HPP
#define GUA_TRIMMED_BEZIERSURFACE_HPP

// header, system
#include <vector>  // std::vector

// header, external
#include <boost/shared_ptr.hpp>

#include <gua/renderer/nurbs_geometry/tml/parametric/beziersurface.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/TrimDomain.hpp>

namespace gua {

class TrimmedBezierSurfaceObject;

class TrimmedBezierSurface : public tml::beziersurface3d {
 public:  // enums, typedefs

  typedef tml::beziersurface3d base_type;
  typedef tml::pointmesh2d<point_type> mesh_type;

  typedef TrimDomain::point_type curve_point_type;
  typedef TrimDomain::curve_type curve_type;
  typedef std::vector<TrimDomain::curve_type> curve_container;

  typedef boost::shared_ptr<TrimmedBezierSurfaceObject> host_ptr;
  typedef boost::shared_ptr<TrimDomain> trimdomain_ptr;

 public:  // c'tors and d'tor

  TrimmedBezierSurface();

  TrimmedBezierSurface(tml::beziersurface3d const& untrimmed_surface);

  TrimmedBezierSurface(TrimmedBezierSurface const& bs);

  ~TrimmedBezierSurface();

  void swap(TrimmedBezierSurface& swp);
  TrimmedBezierSurface& operator=(TrimmedBezierSurface const& cpy);
  void print(std::ostream& os) const;

 public:  // methods

  // add a trimming loop
  void add(curve_container const& trimloop);

  void trimtype(bool type);

  // set the parameter range of the nurbs surface the bezier surface belongs to
  // (there might be trimming curves that effect this surface)
  void nurbsdomain(TrimDomain::bbox_type const&);

  // set parameter range of this patch (u,v in [0.0,1.0])
  void bezierdomain(TrimDomain::bbox_type const&);
  TrimDomain::bbox_type const& bezierdomain() const;

  bool trimtype() const;
  std::size_t trimcurves() const;

  trimdomain_ptr const& domain() const;
  void domain(trimdomain_ptr const& domain);

  mesh_type const& points() const;

 private:  // attributes

  trimdomain_ptr _trimdomain;
  TrimDomain::bbox_type _bezierdomain;  // umin, umax, vmin, vmax
};

}  // namespace gua

#endif  // GUA_TRIMMED_BEZIERSURFACE_HPP
