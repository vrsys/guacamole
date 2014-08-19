/********************************************************************************
*
* Copyright (C) 2008 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : beziervolume_impl.hpp
*  project    : gua
*  description:
*
********************************************************************************/
// header, system
#include <numeric>

#if WIN32
#pragma warning(disable \
                : 4996)  // boost::multi_array warnings for checked iterators
#endif

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/beziercurve.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/beziersurface.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/pointmesh2d.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/converter.hpp>
#include <gua/renderer/nurbs_geometry/tml/util/concatenate.hpp>

namespace tml {

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziervolume<point_t>::beziervolume()
    : _points(),
      _degree_u(0),
      _degree_v(0),
      _degree_w(0),
      _uvwmin_local(0, 0, 0),
      _uvwmax_local(1, 1, 1),
      _uvwmin_global(0, 0, 0),
      _uvwmax_global(1, 1, 1) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziervolume<point_t>::beziervolume(beziervolume<point_t> const& bs)
    : _points(bs._points),
      _degree_u(bs._degree_u),
      _degree_v(bs._degree_v),
      _degree_w(bs._degree_w),
      _uvwmin_local(bs._uvwmin_local),
      _uvwmax_local(bs._uvwmax_local),
      _uvwmin_global(bs._uvwmin_global),
      _uvwmax_global(bs._uvwmax_global) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziervolume<point_t>::beziervolume(pointmesh3d<point_t> const& mesh)
    : _points(mesh),
      _degree_u(std::max(int(0), int(mesh.width()) - 1)),
      _degree_v(std::max(int(0), int(mesh.height()) - 1)),
      _degree_w(std::max(int(0), int(mesh.depth()) - 1)),
      _uvwmin_local(0, 0, 0),
      _uvwmax_local(1, 1, 1),
      _uvwmin_global(0, 0, 0),
      _uvwmax_global(1, 1, 1) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline beziervolume<point_t>::~beziervolume() {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziervolume<point_t>::swap(beziervolume<point_t>& swp) {
  std::swap(_points, swp._points);
  std::swap(_degree_u, swp._degree_u);
  std::swap(_degree_v, swp._degree_v);
  std::swap(_degree_w, swp._degree_w);
  std::swap(_uvwmin_local, swp._uvwmin_local);
  std::swap(_uvwmax_local, swp._uvwmax_local);
  std::swap(_uvwmin_global, swp._uvwmin_global);
  std::swap(_uvwmax_global, swp._uvwmax_global);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziervolume<point_t>& beziervolume<point_t>::operator=(
    beziervolume const& cpy) {
  beziervolume tmp(cpy);
  swap(tmp);
  return *this;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
inline void beziervolume<point_t>::set_points(iterator_t beg, iterator_t end) {
  _points.clear();
  std::copy(beg, end, std::back_inserter(_points));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
typename beziervolume<point_t>::iterator beziervolume<point_t>::begin() {
  return _points.begin();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
typename beziervolume<point_t>::const_iterator
beziervolume<point_t>::begin() const {
  return _points.begin();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
typename beziervolume<point_t>::iterator beziervolume<point_t>::end() {
  return _points.end();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
typename beziervolume<point_t>::const_iterator
beziervolume<point_t>::end() const {
  return _points.end();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziervolume<point_t>::degree_u(std::size_t deg) {
  assert(deg > 0);
  _degree_u = deg;
  _points.width(deg + 1);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziervolume<point_t>::degree_v(std::size_t deg) {
  assert(deg > 0);
  _degree_v = deg;
  _points.height(deg + 1);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziervolume<point_t>::degree_w(std::size_t deg) {
  assert(deg > 0);
  _degree_w = deg;
  _points.depth(deg + 1);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziervolume<point_t>::order_u(std::size_t order) {
  assert(order > 1);
  _degree_u = --order;
  _points.width(order);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziervolume<point_t>::order_v(std::size_t order) {
  assert(order > 1);
  _degree_v = --order;
  _points.height(order);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziervolume<point_t>::order_w(std::size_t order) {
  assert(order > 1);
  _degree_w = --order;
  _points.depth(order);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziervolume<point_t>::degree_u() const {
  return _degree_u;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziervolume<point_t>::degree_v() const {
  return _degree_v;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziervolume<point_t>::degree_w() const {
  return _degree_w;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziervolume<point_t>::order_u() const {
  return _degree_u + 1;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziervolume<point_t>::order_v() const {
  return _degree_v + 1;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziervolume<point_t>::order_w() const {
  return _degree_w + 1;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> void beziervolume<point_t>::elevate_u() {
  // increase degree in elevated parameter direction
  ++_degree_u;

  // generate new control point mesh
  pointmesh3d<point_t> mesh_elevated(
      _points.width() + 1, _points.height(), _points.depth());

  // get subcurves and elevate curve-wise and copy elevated curves to new mesh
  for (unsigned d = 0; d != _points.depth(); ++d) {
    for (unsigned h = 0; h != _points.height(); ++h) {
      std::vector<point_t> subcurve_mesh = _points.submesh(0, h, d);
      beziercurve<point_t> subcurve(subcurve_mesh.begin(), subcurve_mesh.end());

      subcurve.elevate();
      mesh_elevated.submesh(subcurve.begin(), 0, h, d);
    }
  }

  // swap elevated mesh and old mesh
  std::swap(_points, mesh_elevated);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> void beziervolume<point_t>::elevate_v() {
  // increase degree in elevated parameter direction
  ++_degree_v;

  // generate new control point mesh
  pointmesh3d<point_t> mesh_elevated(
      _points.width(), _points.height() + 1, _points.depth());

  // get subcurves and elevate curve-wise and copy elevated curves to new mesh
  for (unsigned d = 0; d != _points.depth(); ++d) {
    for (unsigned w = 0; w != _points.width(); ++w) {
      std::vector<point_t> subcurve_mesh = _points.submesh(1, w, d);
      beziercurve<point_t> subcurve(subcurve_mesh.begin(), subcurve_mesh.end());

      subcurve.elevate();
      mesh_elevated.submesh(subcurve.begin(), 1, w, d);
    }
  }

  // swap elevated mesh and old mesh
  std::swap(_points, mesh_elevated);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> void beziervolume<point_t>::elevate_w() {
  // increase degree in elevated parameter direction
  ++_degree_w;

  // generate new control point mesh
  pointmesh3d<point_t> mesh_elevated(
      _points.width(), _points.height(), _points.depth() + 1);

  // get subcurves and elevate curve-wise and copy elevated curves to new mesh
  for (unsigned h = 0; h != _points.height(); ++h) {
    for (unsigned w = 0; w != _points.width(); ++w) {
      std::vector<point_t> subcurve_mesh = _points.submesh(2, w, h);
      beziercurve<point_t> subcurve(subcurve_mesh.begin(), subcurve_mesh.end());

      subcurve.elevate();
      mesh_elevated.submesh(subcurve.begin(), 2, w, h);
    }
  }

  // swap elevated mesh and old mesh
  std::swap(_points, mesh_elevated);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline point_t beziervolume<point_t>::evaluate(
    value_type const& u,
    value_type const& v,
    value_type const& w,
    evaluator<point_t> const& eval) const {
  return eval(
      _points.begin(), _degree_u + 1, _degree_v + 1, _degree_w + 1, u, v, w);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziervolume<point_t>::evaluate(
    value_type const& u,
    value_type const& v,
    value_type const& w,
    point_t& p,
    evaluator<point_t> const& eval) const {
  eval(
      _points.begin(), _degree_u + 1, _degree_v + 1, _degree_w + 1, u, v, w, p);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziervolume<point_t>::evaluate(
    value_type const& u,
    value_type const& v,
    value_type const& w,
    point_t& p,
    point_t& du,
    point_t& dv,
    point_t& dw,
    evaluator<point_t> const& eval) const {
  eval(_points.begin(),
       _degree_u + 1,
       _degree_v + 1,
       _degree_w + 1,
       u,
       v,
       w,
       p,
       du,
       dv,
       dw);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline axis_aligned_boundingbox<point_t> beziervolume<point_t>::bbox() const {
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
template <template <typename T> class build_policy>
inline oriented_boundingbox<point_t> beziervolume<point_t>::obbox_by_mesh(
    build_policy<point_t> const& policy) const {
  return oriented_boundingbox<point_t>(_points, policy);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <template <typename T> class build_policy>
inline oriented_boundingbox<point_t> beziervolume<point_t>::obbox(
    build_policy<point_t> const& policy) const {
  return oriented_boundingbox<point_t>(_points.begin(), _points.end(), policy);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename beziervolume<point_t>::meshtype const&
beziervolume<point_t>::mesh() const {
  return _points;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziervolume<point_t>::print(std::ostream& os) const {
  os << "<=================================================>" << std::endl;
  os << "beziervolume : " << std::endl;
  os << "Order U : " << _degree_u + 1 << "  Order V : " << _degree_v + 1
     << "  Order W : " << _degree_w + 1 << std::endl;
  os << "Number of control points : " << _points.size() << std::endl;
  os << "Control Points : \n";
  std::copy(
      _points.begin(), _points.end(), std::ostream_iterator<point_t>(os, "\n"));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t beziervolume<point_t>::size() const {
  return _points.size();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline bool beziervolume<point_t>::valid() const {
  return (_points.size() == order_u() + order_v());
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziersurface<point_t> beziervolume<point_t>::slice(boundary_t b) const {
  switch (b) {
    case umin:
      return slice(point_type::u, 0);
    case umax:
      return slice(point_type::u, _points.width() - 1);
    case vmin:
      return slice(point_type::v, 0);
    case vmax:
      return slice(point_type::v, _points.height() - 1);
    case wmin:
      return slice(point_type::w, 0);
    case wmax:
      return slice(point_type::w, _points.depth() - 1);
    default:
      throw std::runtime_error("invalid boundary type");
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline beziersurface<point_t> beziervolume<point_t>::slice(
    std::size_t dim,
    std::size_t n) const {
  return beziersurface<point_t>(_points.submesh(dim, n));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename target_t>
inline beziersurface<target_t> beziervolume<point_t>::slice(
    std::size_t dim,
    std::size_t slice,
    target_t const& dummy) const {
  pointmesh2d<target_t> s;
  s = _points.conv_submesh(dim, slice, dummy);
  return beziersurface<target_t>(s);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziervolume<point_t>::uvw_local(
    parameter_type const& knot_uvwmin,
    parameter_type const& knot_uvwmax) {
  _uvwmin_local = knot_uvwmin;
  _uvwmax_local = knot_uvwmax;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void beziervolume<point_t>::uvw_global(
    parameter_type const& nurbs_uvwmin,
    parameter_type const& nurbs_uvwmax) {
  _uvwmin_global = nurbs_uvwmin;
  _uvwmax_global = nurbs_uvwmax;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename beziervolume<point_t>::parameter_type const&
beziervolume<point_t>::uvwmin_local() const {
  return _uvwmin_local;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename beziervolume<point_t>::parameter_type const&
beziervolume<point_t>::uvwmax_local() const {
  return _uvwmax_local;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename beziervolume<point_t>::parameter_type const&
beziervolume<point_t>::uvwmin_global() const {
  return _uvwmin_global;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename beziervolume<point_t>::parameter_type const&
beziervolume<point_t>::uvwmax_global() const {
  return _uvwmax_global;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
/* virtual */ inline void beziervolume<point_t>::write(std::ostream& os) const {
  _points.write(os);

  os.write(reinterpret_cast<char const*>(&_degree_u), sizeof(std::size_t));
  os.write(reinterpret_cast<char const*>(&_degree_v), sizeof(std::size_t));
  os.write(reinterpret_cast<char const*>(&_degree_w), sizeof(std::size_t));

  os.write(reinterpret_cast<char const*>(&_uvwmin_local),
           sizeof(parameter_type));
  os.write(reinterpret_cast<char const*>(&_uvwmax_local),
           sizeof(parameter_type));
  os.write(reinterpret_cast<char const*>(&_uvwmin_global),
           sizeof(parameter_type));
  os.write(reinterpret_cast<char const*>(&_uvwmax_global),
           sizeof(parameter_type));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
/* virtual */ inline void beziervolume<point_t>::read(std::istream& is) {
  _points.read(is);

  is.read(reinterpret_cast<char*>(&_degree_u), sizeof(std::size_t));
  is.read(reinterpret_cast<char*>(&_degree_v), sizeof(std::size_t));
  is.read(reinterpret_cast<char*>(&_degree_w), sizeof(std::size_t));

  is.read(reinterpret_cast<char*>(&(_uvwmin_local[0])), sizeof(parameter_type));
  is.read(reinterpret_cast<char*>(&(_uvwmax_local[0])), sizeof(parameter_type));
  is.read(reinterpret_cast<char*>(&(_uvwmin_global[0])),
          sizeof(parameter_type));
  is.read(reinterpret_cast<char*>(&(_uvwmax_global[0])),
          sizeof(parameter_type));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
typename beziervolume<point_t>::array_type
beziervolume<point_t>::ocsplit() const {
  array_type result;

  std::size_t new_size_u = order_u() + degree_u();
  std::size_t new_size_v = order_v() + degree_v();
  std::size_t new_size_w = order_w() + degree_w();

  // generate point-meshes for different steps of split
  pointmesh3d<point_t> cpsplit_u(new_size_u, order_v(), order_w());
  pointmesh3d<point_t> cpsplit_uv(new_size_u, new_size_v, order_w());
  pointmesh3d<point_t> cpsplit_uvw(new_size_u, new_size_v, new_size_w);

  converter<point_t> converter;

  // split in u
  for (std::size_t v = 0; v != order_v(); ++v) {
    for (std::size_t w = 0; w != order_w(); ++w) {
      // elevate to virtual nurbscurve to apply knot insertion
      std::vector<point_t> tmp_curve = _points.submesh(0, v, w);

      // create knot vector
      std::vector<value_type> kv_min(order_u(), 0);
      std::vector<value_type> kv_max(order_u(), 1);
      std::multiset<value_type> kv;
      kv.insert(kv_min.begin(), kv_min.end());
      kv.insert(kv_max.begin(), kv_max.end());

      converter.knot_insertion(tmp_curve, kv, order_u(), value_type(0.5));
      cpsplit_u.submesh(tmp_curve.begin(), 0, v, w);
    }
  }

  // split in v
  for (std::size_t u = 0; u != new_size_u; ++u) {
    for (std::size_t w = 0; w != order_w(); ++w) {
      // elevate to virtual nurbscurve to apply knot insertion
      std::vector<point_t> tmp_curve = cpsplit_u.submesh(1, u, w);

      // create knot vector
      std::vector<value_type> kv_min(order_v(), 0);
      std::vector<value_type> kv_max(order_v(), 1);
      std::multiset<value_type> kv;
      kv.insert(kv_min.begin(), kv_min.end());
      kv.insert(kv_max.begin(), kv_max.end());

      converter.knot_insertion(tmp_curve, kv, order_v(), value_type(0.5));
      cpsplit_uv.submesh(tmp_curve.begin(), 1, u, w);
    }
  }

  // split in w
  for (std::size_t u = 0; u != new_size_u; ++u) {
    for (std::size_t v = 0; v != new_size_v; ++v) {
      // elevate to virtual nurbscurve to apply knot insertion
      std::vector<point_t> tmp_curve = cpsplit_uv.submesh(2, u, v);

      // create knot vector
      std::vector<value_type> kv_min(order_w(), 0);
      std::vector<value_type> kv_max(order_w(), 1);
      std::multiset<value_type> kv;
      kv.insert(kv_min.begin(), kv_min.end());
      kv.insert(kv_max.begin(), kv_max.end());

      converter.knot_insertion(tmp_curve, kv, order_w(), value_type(0.5));
      cpsplit_uvw.submesh(tmp_curve.begin(), 2, u, v);
    }
  }

  // generate subvolumes
  for (std::size_t w = 0; w != 2; ++w) {
    for (std::size_t v = 0; v != 2; ++v) {
      for (std::size_t u = 0; u != 2; ++u) {
        pointmesh3d<point_t> submesh = cpsplit_uvw.submesh(u * degree_u(),
                                                           v * degree_v(),
                                                           w * degree_w(),
                                                           order_u(),
                                                           order_v(),
                                                           order_w());
        beziervolume subvolume(submesh);
        //result[array_index(u)][array_index(v)][array_index(w)] = subvolume;
        result[u][v][w] = subvolume;
      }
    }
  }

  return result;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
    inline std::ostream& operator<<(std::ostream& os,
                                    beziervolume<point_t> const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace tml
