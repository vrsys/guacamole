/********************************************************************************
*
* Copyright (C) 2008 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : beziersurface.hpp
*  project    : gua
*  description:
*
********************************************************************************/
#ifndef TML_BEZIERSURFACE_HPP
#define TML_BEZIERSURFACE_HPP

// header, system
#include <vector>      // std::vector
#include <iomanip>     // std::setprecision
#include <functional>  // std::unary_function

// header, project
#include <gua/renderer/nurbs_geometry/tml/axis_aligned_boundingbox.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/pointmesh2d.hpp>
//#include <renderer/nurbs_geometry/tml/parametric/beziervolume.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/beziercurve.hpp>

#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/evaluator.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/decasteljau.hpp>

namespace tml {

template <typename T> class beziervolume;

////////////////////////////////////////////////////////////////////////////////
// beziersurface - including a control polygon, degree and maximum
//                 parameter values in case of trimming
////////////////////////////////////////////////////////////////////////////////
template <typename point_t> class beziersurface {
 public:  // enums, typedefs

  typedef point_t point_type;
  typedef typename point_t::value_type value_type;
  typedef axis_aligned_boundingbox<point_t> bbox_t;
  typedef pointmesh2d<point_t> mesh_type;
  typedef typename mesh_type::iterator iterator;
  typedef typename mesh_type::const_iterator const_iterator;

 public:  // c'tors and d'tor

  beziersurface();
  beziersurface(pointmesh2d<point_t> const& controlpoints);
  beziersurface(beziersurface const& bs);
  virtual ~beziersurface();

  virtual void swap(beziersurface& swp);
  virtual beziersurface& operator=(beziersurface const& cpy);

 public:  // methods

  /////////////////////////////////////////////////////////////////////////////
  // initialize and modify bezier surface
  /////////////////////////////////////////////////////////////////////////////

  // add a single point to the mesh
  void add(point_t const& point);

  // get const reference to mesh
  mesh_type const& mesh() const;

  // copy control points into surface
  template <typename iterator_t> void mesh(iterator_t beg, iterator_t end);

  // set degree/order in both parameter directions
  // necessary property : number of control points = order_u * order_v
  void degree_u(std::size_t deg);
  void degree_v(std::size_t deg);
  void order_u(std::size_t order);
  void order_v(std::size_t order);

  // degree elevation
  void elevate_u();
  void elevate_v();

  /////////////////////////////////////////////////////////////////////////////
  // evaluation, information and print out
  /////////////////////////////////////////////////////////////////////////////

  // getter, information and evaluation
  std::size_t degree_u() const;
  std::size_t degree_v() const;
  std::size_t order_u() const;
  std::size_t order_v() const;

  void evaluate(value_type const& u,
                value_type const& v,
                point_t& point,
                point_t& du,
                point_t& dv,
                evaluator<point_t> const& evaluator_method =
                    decasteljau<point_t>()) const;

  void evaluate(value_type const& u,
                value_type const& v,
                point_t& point,
                evaluator<point_t> const& evaluator_method =
                    decasteljau<point_t>()) const;

  point_t evaluate(value_type const& u,
                   value_type const& v,
                   evaluator<point_t> const& evaluator_method =
                       decasteljau<point_t>()) const;

  bbox_t bbox() const;

  virtual void split(beziersurface& bl,
                     beziersurface& tl,
                     beziersurface& br,
                     beziersurface& tr) const;

  template <typename insert_iterator> void split(insert_iterator ins) const;

  void split_u(value_type const& u,
               beziersurface& lhs,
               beziersurface& rhs) const;

  void transpose();
  beziervolume<point_t> extrude(beziercurve<point_t> const& extrusion) const;

  template <typename vertex_container, typename index_container>
  void tesselate(vertex_container& vertices, index_container& indices) const;

  virtual void print(std::ostream& os) const;

  // number of control points in mesh
  std::size_t size() const;

  iterator begin();
  const_iterator begin() const;
  iterator end();
  const_iterator end() const;

  bool valid() const;
  value_type curvature() const;

 protected:  // attributes

  mesh_type _points;

  std::size_t _degree_u;
  std::size_t _degree_v;
};

// ostream operator
template <typename point_t>
    std::ostream& operator<<(std::ostream& os,
                             beziersurface<point_t> const& rhs);

typedef beziersurface<point3f> beziersurface3f;
typedef beziersurface<point3d> beziersurface3d;

}  // namespace tml

#include "implementation/beziersurface_impl.hpp"

#endif  // TML_BEZIERSURFACE_HPP
