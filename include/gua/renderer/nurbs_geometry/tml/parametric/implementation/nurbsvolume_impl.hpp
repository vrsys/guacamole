/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : nurbsvolume_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_NURBSVOLUME_IMPL_HPP
#define TML_NURBSVOLUME_IMPL_HPP

// header, system
#include <cassert>   // std::assert
#include <iterator>  // std::ostream_iterator, std::iterator_traits

// header, project
#include <gua/renderer/nurbs_geometry/tml/util/copy_constructor_adapter.hpp>

namespace tml {

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
nurbsvolume<point_t>::nurbsvolume()
    : _degree_u(),
      _degree_v(),
      _degree_w(),
      _points(),
      _knotvector_u(),
      _knotvector_v(),
      _knotvector_w() {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
nurbsvolume<point_t>::nurbsvolume(nurbsvolume const& rhs)
    : _degree_u(rhs._degree_u),
      _degree_v(rhs._degree_v),
      _degree_w(rhs._degree_w),
      _points(rhs._points),
      _knotvector_u(rhs._knotvector_u),
      _knotvector_v(rhs._knotvector_v),
      _knotvector_w(rhs._knotvector_w) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> nurbsvolume<point_t>::~nurbsvolume() {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbsvolume<point_t>::swap(nurbsvolume& rhs) {
  std::swap(_degree_u, rhs._degree_u);
  std::swap(_degree_v, rhs._degree_v);
  std::swap(_degree_w, rhs._degree_w);

  std::swap(_points, rhs._points);

  std::swap(_knotvector_u, rhs._knotvector_u);
  std::swap(_knotvector_v, rhs._knotvector_v);
  std::swap(_knotvector_w, rhs._knotvector_w);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline nurbsvolume<point_t>& nurbsvolume<point_t>::operator=(
    nurbsvolume const& rhs) {
  nurbsvolume tmp(rhs);
  swap(tmp);
  return *this;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline /* virtual */ void nurbsvolume<point_t>::clear() {
  _points.clear();

  _knotvector_u.clear();
  _knotvector_v.clear();
  _knotvector_w.clear();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline bool nurbsvolume<point_t>::verify() const {
  return ((_knotvector_u.size() - _points.width() - _degree_u - 1 == 0) &&
          (_knotvector_v.size() - _points.height() - _degree_v - 1 == 0) &&
          (_knotvector_w.size() - _points.depth() - _degree_w - 1 == 0));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline size_t nurbsvolume<point_t>::degree_u() const {
  return _degree_u;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline size_t nurbsvolume<point_t>::order_u() const {
  return _degree_u + 1;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline size_t nurbsvolume<point_t>::degree_v() const {
  return _degree_v;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline size_t nurbsvolume<point_t>::order_v() const {
  return _degree_v + 1;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline size_t nurbsvolume<point_t>::degree_w() const {
  return _degree_w;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline size_t nurbsvolume<point_t>::order_w() const {
  return _degree_w + 1;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbsvolume<point_t>::degree_u(size_t deg) {
  _degree_u = deg;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbsvolume<point_t>::degree_v(size_t deg) {
  _degree_v = deg;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbsvolume<point_t>::degree_w(size_t deg) {
  _degree_w = deg;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
inline void nurbsvolume<point_t>::knotvector_u(iterator_t beg, iterator_t end) {
  typedef typename std::iterator_traits<iterator_t>::value_type input_value_t;

  _knotvector_u.resize(std::distance(beg, end));
  std::transform(
      beg,
      end,
      _knotvector_u.begin(),
      tml::util::copy_constructor_adapter<input_value_t, value_type>());

  //std::set<value_type> unique_knots_u (_knotvector_u.begin(),
  //_knotvector_u.end());
  //_knotspans_u = std::max(0, int(unique_knots_u.size() - 1));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
inline void nurbsvolume<point_t>::knotvector_v(iterator_t beg, iterator_t end) {
  typedef typename std::iterator_traits<iterator_t>::value_type input_value_t;

  _knotvector_v.resize(std::distance(beg, end));
  std::transform(
      beg,
      end,
      _knotvector_v.begin(),
      tml::util::copy_constructor_adapter<input_value_t, value_type>());

  //std::set<value_type> unique_knots_v (_knotvector_v.begin(),
  //_knotvector_v.end());
  //_knotspans_v = std::max(0, int(unique_knots_v.size() - 1));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
inline void nurbsvolume<point_t>::knotvector_w(iterator_t beg, iterator_t end) {
  typedef typename std::iterator_traits<iterator_t>::value_type input_value_t;

  _knotvector_w.resize(std::distance(beg, end));
  std::transform(
      beg,
      end,
      _knotvector_w.begin(),
      tml::util::copy_constructor_adapter<input_value_t, value_type>());

  //std::set<value_type> unique_knots_w (_knotvector_w.begin(),
  //_knotvector_w.end());
  //_knotspans_w = std::max(0, int(unique_knots_w.size() - 1));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
inline void nurbsvolume<point_t>::set_points(iterator_t begin, iterator_t end) {
  _points.clear();
  std::copy(begin, end, std::back_inserter(_points));
  //std::copy(begin, end, std::inserter(_points, _points.end()));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
void nurbsvolume<point_t>::set_point(size_t pos_u,
                                     size_t pos_v,
                                     size_t pos_w,
                                     point_t const& p) {
  _points(pos_u, pos_v, pos_w) = p;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
void nurbsvolume<point_t>::resize(std::size_t size_u,
                                  std::size_t size_v,
                                  std::size_t size_w) {
  _points.resize(size_u * size_v * size_w);

  _points.width(size_u);
  _points.height(size_v);
  _points.depth(size_w);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbsvolume<point_t>::transpose(std::size_t target_dim_u,
                                            std::size_t target_dim_v,
                                            std::size_t target_dim_w) {
  nurbsvolume transposed(*this);

  transposed._points.transpose(target_dim_u, target_dim_v, target_dim_w);

  switch (target_dim_u) {
    case 0: {
      transposed._degree_u = _degree_u;
      transposed._knotvector_u = _knotvector_u;
      break;
    }
    case 1: {
      transposed._degree_v = _degree_u;
      transposed._knotvector_v = _knotvector_u;
      break;
    }
    case 2: {
      transposed._degree_w = _degree_u;
      transposed._knotvector_w = _knotvector_u;
      break;
    }
  }

  switch (target_dim_v) {
    case 0: {
      transposed._degree_u = _degree_v;
      transposed._knotvector_u = _knotvector_v;
      break;
    }
    case 1: {
      transposed._degree_v = _degree_v;
      transposed._knotvector_v = _knotvector_v;
      break;
    }
    case 2: {
      transposed._degree_w = _degree_v;
      transposed._knotvector_w = _knotvector_v;
      break;
    }
  }

  switch (target_dim_w) {
    case 0: {
      transposed._degree_u = _degree_w;
      transposed._knotvector_u = _knotvector_w;
      break;
    }
    case 1: {
      transposed._degree_v = _degree_w;
      transposed._knotvector_v = _knotvector_w;
      break;
    }
    case 2: {
      transposed._degree_w = _degree_w;
      transposed._knotvector_w = _knotvector_w;
      break;
    }
  }

  swap(transposed);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbsvolume<point_t>::numberofpoints_u(size_t n) {
  _points.width(n);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbsvolume<point_t>::numberofpoints_v(size_t n) {
  _points.height(n);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbsvolume<point_t>::numberofpoints_w(size_t n) {
  _points.depth(n);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t nurbsvolume<point_t>::numberofpoints_u() const {
  return _points.width();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t nurbsvolume<point_t>::numberofpoints_v() const {
  return _points.height();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t nurbsvolume<point_t>::numberofpoints_w() const {
  return _points.depth();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline pointmesh3d<point_t> const& nurbsvolume<point_t>::points() const {
  return _points;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename nurbsvolume<point_t>::boundingbox_type
nurbsvolume<point_t>::bbox() const {
  return _points.bbox();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::vector<typename point_t::value_type> const&
nurbsvolume<point_t>::knotvector_u() const {
  return _knotvector_u;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::vector<typename point_t::value_type> const&
nurbsvolume<point_t>::knotvector_v() const {
  return _knotvector_v;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::vector<typename point_t::value_type> const&
nurbsvolume<point_t>::knotvector_w() const {
  return _knotvector_w;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> unsigned nurbsvolume<point_t>::knotspans_u() const {
  std::set<value_type> unique_knots_u(_knotvector_u.begin(),
                                      _knotvector_u.end());
  return std::max(0, int(unique_knots_u.size() - 1));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> unsigned nurbsvolume<point_t>::knotspans_v() const {
  std::set<value_type> unique_knots_v(_knotvector_v.begin(),
                                      _knotvector_v.end());
  return std::max(0, int(unique_knots_v.size() - 1));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> unsigned nurbsvolume<point_t>::knotspans_w() const {
  std::set<value_type> unique_knots_w(_knotvector_w.begin(),
                                      _knotvector_w.end());
  return std::max(0, int(unique_knots_w.size() - 1));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename point_t::value_type nurbsvolume<point_t>::umin() const {
  return _knotvector_u.front();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename point_t::value_type nurbsvolume<point_t>::umax() const {
  return _knotvector_u.back();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename point_t::value_type nurbsvolume<point_t>::vmin() const {
  return _knotvector_v.front();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename point_t::value_type nurbsvolume<point_t>::vmax() const {
  return _knotvector_v.back();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename point_t::value_type nurbsvolume<point_t>::wmin() const {
  return _knotvector_w.front();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename point_t::value_type nurbsvolume<point_t>::wmax() const {
  return _knotvector_w.back();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline void nurbsvolume<point_t>::normalize() {
  _points.normalize();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbsvolume<point_t>::scale(value_type const& s) {
  _points.scale(s);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbsvolume<point_t>::print(std::ostream& os) const {
  os << "Order U : " << _degree_u + 1 << "  Order V : " << _degree_v + 1
     << "  Order W : " << _degree_w + 1 << std::endl;
  os << "#U  : " << _points.width() << " #V  : " << _points.height()
     << " #W  : " << _points.depth() << std::endl;

  os << "knotU : ";
  std::copy(_knotvector_u.begin(),
            _knotvector_u.end(),
            std::ostream_iterator<value_type>(os, " "));

  os << std::endl << "knotV : ";
  std::copy(_knotvector_v.begin(),
            _knotvector_v.end(),
            std::ostream_iterator<value_type>(os, " "));

  os << std::endl << "knotW : ";
  std::copy(_knotvector_w.begin(),
            _knotvector_w.end(),
            std::ostream_iterator<value_type>(os, " "));

  os << "Control Points : \n";
  std::copy(
      _points.begin(), _points.end(), std::ostream_iterator<point_t>(os, "\n"));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline /* virtual */ void nurbsvolume<point_t>::write(std::ostream& os) const {
  os.write(reinterpret_cast<char const*>(&_degree_u), sizeof(std::size_t));
  os.write(reinterpret_cast<char const*>(&_degree_v), sizeof(std::size_t));
  os.write(reinterpret_cast<char const*>(&_degree_w), sizeof(std::size_t));

  _points.write(os);

  std::size_t knots_u = _knotvector_u.size();
  std::size_t knots_v = _knotvector_v.size();
  std::size_t knots_w = _knotvector_w.size();

  os.write(reinterpret_cast<char const*>(&knots_u), sizeof(std::size_t));
  os.write(reinterpret_cast<char const*>(&knots_v), sizeof(std::size_t));
  os.write(reinterpret_cast<char const*>(&knots_w), sizeof(std::size_t));

  os.write(reinterpret_cast<char const*>(&_knotvector_u.front()),
           sizeof(value_type) * knots_u);
  os.write(reinterpret_cast<char const*>(&_knotvector_v.front()),
           sizeof(value_type) * knots_v);
  os.write(reinterpret_cast<char const*>(&_knotvector_w.front()),
           sizeof(value_type) * knots_w);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline /* virtual */ void nurbsvolume<point_t>::read(std::istream& is) {
  is.read(reinterpret_cast<char*>(&_degree_u), sizeof(std::size_t));
  is.read(reinterpret_cast<char*>(&_degree_v), sizeof(std::size_t));
  is.read(reinterpret_cast<char*>(&_degree_w), sizeof(std::size_t));

  _points.read(is);

  std::size_t knots_u;
  std::size_t knots_v;
  std::size_t knots_w;

  is.read(reinterpret_cast<char*>(&knots_u), sizeof(std::size_t));
  is.read(reinterpret_cast<char*>(&knots_v), sizeof(std::size_t));
  is.read(reinterpret_cast<char*>(&knots_w), sizeof(std::size_t));

  _knotvector_u.resize(knots_u);
  _knotvector_v.resize(knots_v);
  _knotvector_w.resize(knots_w);

  is.read(reinterpret_cast<char*>(&_knotvector_u.front()),
          sizeof(value_type) * _knotvector_u.size());
  is.read(reinterpret_cast<char*>(&_knotvector_v.front()),
          sizeof(value_type) * _knotvector_v.size());
  is.read(reinterpret_cast<char*>(&_knotvector_w.front()),
          sizeof(value_type) * _knotvector_w.size());
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
    inline std::ostream& operator<<(std::ostream& os,
                                    nurbsvolume<point_t> const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace tml

#endif  // TML_NURBSVOLUME_IMPL_HPP
