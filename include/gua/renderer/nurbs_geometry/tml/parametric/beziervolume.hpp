/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : beziervolume.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_BEZIERVOLUME_HPP
#define TML_BEZIERVOLUME_HPP

#if WIN32
#pragma warning(disable : 4251)  // dll-interface
#endif

// header, system
#include <set>     // std::set
#include <vector>  // std::vector

// header, project
#include <gua/renderer/nurbs_geometry/tml/axis_aligned_boundingbox.hpp>
#include <gua/renderer/nurbs_geometry/tml/interval.hpp>
#include <gua/renderer/nurbs_geometry/tml/oriented_boundingbox.hpp>

#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/evaluator.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/decasteljau.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/pointmesh3d.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

#include <boost/array.hpp>

namespace tml {

template <typename T> class beziersurface;

/**
 * beziervolume - including 3d control point grid and 3d domain
 */
template <typename point_t> class beziervolume {
 public:  // typedefs

  enum boundary_t {
    umin = 0,
    umax = 1,
    vmin = 2,
    vmax = 3,
    wmin = 4,
    wmax = 5,
    count = 6
  };

  typedef point_t point_type;
  typedef typename point_t::value_type value_type;
  typedef point<value_type, 3> parameter_type;
  typedef typename std::vector<point_t>::iterator iterator;
  typedef typename std::vector<point_t>::const_iterator const_iterator;
  typedef axis_aligned_boundingbox<point_t> boundingbox_type;
  typedef oriented_boundingbox<point_t> obbox_type;
  typedef pointmesh3d<point_t> meshtype;

  typedef boost::array<boost::array<boost::array<beziervolume<point_t>, 2>, 2>,
                       2> array_type;

 public:  // c'tors and d'tor

  beziervolume();
  beziervolume(beziervolume const& bs);
  beziervolume(pointmesh3d<point_t> const& bs);
  virtual ~beziervolume();

  virtual void swap(beziervolume& swp);
  virtual beziervolume& operator=(beziervolume const& cpy);

 public:  // methods

  // copy control points into surface
  template <typename iterator_t>
  void set_points(iterator_t beg, iterator_t end);

  iterator begin();
  const_iterator begin() const;

  iterator end();
  const_iterator end() const;

  // set degree/order in all parameter directions
  // necessary property : number of control points = order_u * order_v *
  // order_w
  void degree_u(std::size_t deg);
  void degree_v(std::size_t deg);
  void degree_w(std::size_t deg);

  void order_u(std::size_t order);
  void order_v(std::size_t order);
  void order_w(std::size_t order);

  // elevate degree in desired dimension
  void elevate_u();
  void elevate_v();
  void elevate_w();

  /////////////////////////////////////////////////////////////////////////////
  // evaluation, information and print out
  /////////////////////////////////////////////////////////////////////////////

  // getter, information and evaluation
  std::size_t degree_u() const;
  std::size_t degree_v() const;
  std::size_t degree_w() const;

  std::size_t order_u() const;
  std::size_t order_v() const;
  std::size_t order_w() const;

  void evaluate(value_type const& u,
                value_type const& v,
                value_type const& w,
                point_t& point,
                point_t& du,
                point_t& dv,
                point_t& dw,
                evaluator<point_t> const& evaluator_method =
                    decasteljau<point_t>()) const;

  void evaluate(value_type const& u,
                value_type const& v,
                value_type const& w,
                point_t& point,
                evaluator<point_t> const& evaluator_method =
                    decasteljau<point_t>()) const;

  point_t evaluate(value_type const& u,
                   value_type const& v,
                   value_type const& w,
                   evaluator<point_t> const& evaluator_method =
                       decasteljau<point_t>()) const;

  boundingbox_type bbox() const;

  template <template <typename T> class policy>
  obbox_type obbox_by_mesh(policy<point_t> const&) const;

  template <template <typename T> class policy>
  obbox_type obbox(policy<point_t> const&) const;

  meshtype const& mesh() const;

  void print(std::ostream& os) const;

  // number of control points in mesh
  std::size_t size() const;
  bool valid() const;

  beziersurface<point_t> slice(boundary_t b) const;

  beziersurface<point_t> slice(std::size_t dim, std::size_t slice) const;

  template <typename target_t>
  beziersurface<target_t> slice(std::size_t dim,
                                std::size_t slice,
                                target_t const& dummy = target_t()) const;

  void uvw_local(parameter_type const& uvwmin, parameter_type const& uvwmax);

  void uvw_global(parameter_type const& uvwmin, parameter_type const& uvwmax);

  parameter_type const& uvwmin_local() const;
  parameter_type const& uvwmax_local() const;

  parameter_type const& uvwmin_global() const;
  parameter_type const& uvwmax_global() const;

  virtual void write(std::ostream& os) const;
  virtual void read(std::istream& is);

  virtual array_type ocsplit() const;

 private:  // members

  meshtype _points;  // control points

  std::size_t _degree_u;  // degree in width
  std::size_t _degree_v;  // degree of height
  std::size_t _degree_w;  // degree of depth

  parameter_type _uvwmin_local;  // minimum of uvw domain of THIS patch
  parameter_type _uvwmax_local;  // maximum of uvw domain of THIS patch

  parameter_type
      _uvwmin_global;  // minimum of uvw domain of according NURBS patch
  parameter_type
      _uvwmax_global;  // maximum of uvw domain of according NURBS patch

  // example in 2D domain [u,v]:
  //
  //  uminglobal uminlocal umaxlocal           umaxglobal
  //  |----------------------------------|------| vmaxglobal
  //  |            |        |            |      |
  //  |-------------XXXXXXXX-------------|------| vmaxlocal
  //  |            |XXXXXXXX|            |      |
  //  |-------------XXXXXXXX-------------|------| vminlocal
  //  |            |        |            |      |
  //  |----------------------------------|------|
  //  |            |        |            |      |
  //  |----------------------------------|------| vminglobal
};

// ostream operator
template <typename point_t>
    std::ostream& operator<<(std::ostream& os,
                             beziervolume<point_t> const& rhs);

}  // namespace tml

#include "implementation/beziervolume_impl.hpp"

#endif  // TML_BEZIERVOLUME_HPP
