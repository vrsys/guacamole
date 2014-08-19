/********************************************************************************
*
* Copyright (C) 2008 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : beziersurface_impl.hpp
*  project    : gua
*  description:
*
********************************************************************************/
#ifndef TML_BEZIERSURFACE_IMPL_HPP
#define TML_BEZIERSURFACE_IMPL_HPP

// header, system
#include <iterator>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/converter.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/beziercurve.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/beziervolume.hpp>

namespace tml {

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziersurface<point_t>::beziersurface()
    : _points(), _degree_u(0), _degree_v(0) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziersurface<point_t>::beziersurface(
    pointmesh2d<point_t> const& controlpoints)
    : _points(controlpoints),
      _degree_u(controlpoints.width() - 1),
      _degree_v(controlpoints.height() - 1) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziersurface<point_t>::beziersurface(beziersurface<point_t> const& bs)
    : _points(bs._points), _degree_u(bs._degree_u), _degree_v(bs._degree_v) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline beziersurface<point_t>::~beziersurface() {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziersurface<point_t>::swap(beziersurface<point_t>& swp) {
  std::swap(_points, swp._points);
  std::swap(_degree_u, swp._degree_u);
  std::swap(_degree_v, swp._degree_v);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziersurface<point_t>& beziersurface<point_t>::operator=(
    beziersurface const& cpy) {
  beziersurface tmp(cpy);
  swap(tmp);
  return *this;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziersurface<point_t>::add(point_t const& point) {
  _points.push_back(point);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline pointmesh2d<point_t> const& beziersurface<point_t>::mesh() const {
  return _points;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
inline void beziersurface<point_t>::mesh(iterator_t beg, iterator_t end) {
  _points.clear();
  std::for_each(beg,
                end,
                [&](point_t const & p) {
    _points.push_back(p);
  });
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziersurface<point_t>::degree_u(std::size_t deg) {
  assert(deg > 0);
  _degree_u = deg;
  _points.width(deg + 1);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziersurface<point_t>::degree_v(std::size_t deg) {
  assert(deg > 0);
  _degree_v = deg;
  _points.height(deg + 1);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziersurface<point_t>::order_u(std::size_t order) {
  assert(order > 1);
  _degree_u = order - 1;
  _points.width(order);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziersurface<point_t>::order_v(std::size_t order) {
  assert(order > 1);
  _degree_v = order - 1;
  _points.height(order);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline void beziersurface<point_t>::elevate_u() {
  _points.width(order_u());
  _points.height(order_v());

  assert(_points.valid());

  pointmesh2d<point_t> elevated_mesh;

  beziercurve<point_t> old;
  beziercurve<point_t> neu;

  for (std::size_t u = 0; u <= _degree_v; ++u) {
    beziercurve<point_t> bc(_points.row(u));

    //old.add(bc.evaluate(0.3, horner<point_t>()));
    bc.elevate();
      //neu.add(bc.evaluate(0.3, horner<point_t>()));

    std::for_each(bc.begin(),
                  bc.end(),
                  [&](point_t const & p) {
      elevated_mesh.push_back(p);
    });
    //std::copy(bc.begin(), bc.end(), std::inserter<pointmesh2d<point_t>
    //>(elevated_mesh, elevated_mesh.end()));
  }

  elevated_mesh.width(order_u() + 1);
  elevated_mesh.height(order_v());

  _points.swap(elevated_mesh);
  ++_degree_u;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline void beziersurface<point_t>::elevate_v() {
  _points.width(order_u());
  _points.height(order_v());

  assert(_points.valid());

  pointmesh2d<point_t> elevated_mesh;
  _points.transpose();

  for (std::size_t v = 0; v <= _degree_u; ++v) {
    beziercurve<point_t> bc(_points.row(v));
    bc.elevate();
    elevated_mesh.add_row(bc.begin(), bc.end());
  }

  elevated_mesh.transpose();
  elevated_mesh.width(order_u());
  elevated_mesh.height(order_v() + 1);

  ++_degree_v;
  _points.swap(elevated_mesh);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziersurface<point_t>::degree_u() const {
  return _degree_u;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziersurface<point_t>::degree_v() const {
  return _degree_v;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziersurface<point_t>::order_u() const {
  return _degree_u + 1;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziersurface<point_t>::order_v() const {
  return _degree_v + 1;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline point_t beziersurface<point_t>::evaluate(
    value_type const& u,
    value_type const& v,
    evaluator<point_t> const& eval) const {
  return eval(_points.begin(), _degree_u + 1, _degree_v + 1, u, v);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziersurface<point_t>::evaluate(
    value_type const& u,
    value_type const& v,
    point_t& p,
    evaluator<point_t> const& eval) const {
  eval(_points.begin(), _degree_u + 1, _degree_v + 1, u, v, p);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziersurface<point_t>::evaluate(
    value_type const& u,
    value_type const& v,
    point_t& p,
    point_t& du,
    point_t& dv,
    evaluator<point_t> const& eval) const {
  eval(_points.begin(), _degree_u + 1, _degree_v + 1, u, v, p, du, dv);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline axis_aligned_boundingbox<point_t> beziersurface<point_t>::bbox() const {
  point_t pmin, pmax;

  for (std::size_t axis = 0; axis != point_t::coordinates; ++axis) {
    pmin[axis] = std::for_each(
        _points.begin(), _points.end(), minimal_coordinates<point_t>(axis))
        .result();
    pmax[axis] = std::for_each(
        _points.begin(), _points.end(), maximal_coordinates<point_t>(axis))
        .result();
  }

  return axis_aligned_boundingbox<point_t>(pmin, pmax);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline /* virtual */ void beziersurface<point_t>::split(
    beziersurface& lower_left,
    beziersurface& upper_left,
    beziersurface& lower_right,
    beziersurface& upper_right) const {
  converter<point_t> conv;

  // split into left and right surface
  beziersurface left, right;
  split_u(value_type(0.5), left, right);

  // transpose left and right surface and split again
  left.transpose();
  right.transpose();

  left.split_u(value_type(0.5), lower_left, upper_left);
  right.split_u(value_type(0.5), lower_right, upper_right);

  // transpose split surfaces back to original orientation
  lower_left.transpose();
  upper_left.transpose();
  lower_right.transpose();
  upper_right.transpose();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename insert_iterator>
inline void beziersurface<point_t>::split(insert_iterator ins) const {
  beziersurface lower_left, upper_left, lower_right, upper_right;

  split(lower_left, upper_left, lower_right, upper_right);

  (*ins) = lower_left;
  (*ins) = upper_left;
  (*ins) = lower_right;
  (*ins) = upper_right;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
void beziersurface<point_t>::split_u(value_type const& u,
                                     beziersurface<point_t>& lhs,
                                     beziersurface<point_t>& rhs) const {
  converter<point_t> conv;

  // make sure control point grid is empty
  lhs._points.clear();
  rhs._points.clear();

  for (std::size_t r = 0; r != _points.height(); ++r) {
    // split row-wise
    beziercurve<point_t> left, right;
    beziercurve<point_t> row(_points.row(r));

    row.split(value_type(u), left, right);
    // generate submeshes
    lhs._points.add_row(left);
    rhs._points.add_row(right);
  }

  // set degree of subsurfaces
  lhs._degree_u = _degree_u;
  lhs._degree_v = _degree_v;

  rhs._degree_u = _degree_u;
  rhs._degree_v = _degree_v;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline void beziersurface<point_t>::transpose() {
  _points.transpose();
  std::swap(_degree_u, _degree_v);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziervolume<point_t> beziersurface<point_t>::extrude(
    beziercurve<point_t> const& extrusion_curve) const {
  pointmesh3d<point_t> mesh;

  point_t basepoint = extrusion_curve.front();

  for (typename beziercurve<point_t>::const_point_iterator i =
           extrusion_curve.begin();
       i != extrusion_curve.end();
       ++i) {
    point_t diff = (*i) - basepoint;
    for (typename beziercurve<point_t>::const_point_iterator s =
             _points.begin();
         s != _points.end();
         ++s) {
      mesh.push_back(*s + diff);
    }
  }

  mesh.width(_points.width());
  mesh.height(_points.height());
  mesh.depth(extrusion_curve.order());

  return beziervolume<point_t>(mesh);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename vertex_container, typename index_container>
inline void beziersurface<point_t>::tesselate(vertex_container& vertices,
                                              index_container& indices) const {
  typedef typename vertex_container::value_type vertex_type;
  typedef typename index_container::value_type index_type;

  index_type vertex_offset = index_type(vertices.size());
  std::transform(_points.begin(),
                 _points.end(),
                 std::back_inserter(vertices),
                 [&](point_t const & p) {
    return vertex_type(p);
  });

  for (index_type row = 0; row != degree_v(); ++row) {
    for (index_type col = 0; col != degree_u(); ++col) {
      // c-d
      // |/|
      // a-b
      index_type a = vertex_offset + row * index_type(order_v()) + col;
      index_type b = vertex_offset + row * index_type(order_v()) + col + 1;
      index_type c = vertex_offset + (row + 1) * index_type(order_v()) + col;
      index_type d =
          vertex_offset + (row + 1) * index_type(order_v()) + col + 1;

      indices.push_back(a);
      indices.push_back(b);
      indices.push_back(d);

      indices.push_back(a);
      indices.push_back(d);
      indices.push_back(c);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziersurface<point_t>::print(std::ostream& os) const {
  os << "<=================================================>" << std::endl;
  os << "beziersurface : " << std::endl;
  os << "Order U : " << _degree_u + 1 << "  Order V : " << _degree_v + 1
     << std::endl;
  os << "Number of control points : " << _points.size() << std::endl;
  os << "Control Points : \n";
  std::copy(
      _points.begin(), _points.end(), std::ostream_iterator<point_t>(os, "\n"));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziersurface<point_t>::size() const {
  return _points.size();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline bool beziersurface<point_t>::valid() const {
  return (_points.size() == order_u() + order_v());
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename beziersurface<point_t>::iterator
beziersurface<point_t>::begin() {
  return _points.begin();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename beziersurface<point_t>::const_iterator
beziersurface<point_t>::begin() const {
  return _points.begin();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename beziersurface<point_t>::iterator beziersurface<point_t>::end() {
  return _points.end();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename beziersurface<point_t>::const_iterator
beziersurface<point_t>::end() const {
  return _points.end();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename beziersurface<point_t>::value_type
beziersurface<point_t>::curvature() const {
  if (_degree_u < 2 || _degree_v < 2)
    return 0;

  value_type alpha = 0;
  for (std::size_t r = 0; r != _points.height(); ++r) {
    tml::beziercurve<point_t> row_curve(_points.row(r));
    alpha = std::max(alpha, row_curve.curvature());
  }

  for (std::size_t c = 0; c != _points.width(); ++c) {
    tml::beziercurve<point_t> col_curve(_points.column(c));
    alpha = std::max(alpha, col_curve.curvature());
  }

  return alpha;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
    inline std::ostream& operator<<(std::ostream& os,
                                    beziersurface<point_t> const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace tml

#endif  // TML_BEZIERSURFACE_IMPL_HPP
