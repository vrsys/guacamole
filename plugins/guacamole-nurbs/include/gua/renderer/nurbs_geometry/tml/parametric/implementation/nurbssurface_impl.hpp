/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : nurbssurface_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_NURBSSURFACE_IMPL_HPP
#define TML_NURBSSURFACE_IMPL_HPP

// header, system
#include <cassert>   // std::assert
#include <iterator>  // std::ostreamiterator

// header, project

namespace tml {

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
nurbssurface<point_t>::nurbssurface()
    : _degree_u(),
      _degree_v(),
      _npoints_u(),
      _npoints_v(),
      _points(),
      _knotvector_u(),
      _knotvector_v() {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
nurbssurface<point_t>::nurbssurface(nurbssurface const& rhs)
    : _degree_u(rhs._degree_u),
      _degree_v(rhs._degree_v),
      _npoints_u(rhs._npoints_u),
      _npoints_v(rhs._npoints_v),
      _points(rhs._points),
      _knotvector_u(rhs._knotvector_u),
      _knotvector_v(rhs._knotvector_v) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> nurbssurface<point_t>::~nurbssurface() {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbssurface<point_t>::swap(nurbssurface& rhs) {
  std::swap(_degree_u, rhs._degree_u);
  std::swap(_degree_v, rhs._degree_v);
  std::swap(_npoints_u, rhs._npoints_u);
  std::swap(_npoints_v, rhs._npoints_v);

  std::swap(_points, rhs._points);

  std::swap(_knotvector_u, rhs._knotvector_u);
  std::swap(_knotvector_v, rhs._knotvector_v);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline nurbssurface<point_t>& nurbssurface<point_t>::operator=(
    nurbssurface const& rhs) {
  nurbssurface tmp(rhs);
  swap(tmp);
  return *this;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline bool nurbssurface<point_t>::verify() const {
  return ((_knotvector_u.size() - _npoints_u - _degree_u - 1 == 0) &&
          (_knotvector_v.size() - _npoints_v - _degree_v - 1 == 0));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline size_t nurbssurface<point_t>::degree_u() const {
  return _degree_u;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline size_t nurbssurface<point_t>::order_u() const {
  return _degree_u + 1;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline size_t nurbssurface<point_t>::degree_v() const {
  return _degree_v;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline size_t nurbssurface<point_t>::order_v() const {
  return _degree_v + 1;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbssurface<point_t>::degree_u(size_t deg) {
  _degree_u = deg;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbssurface<point_t>::degree_v(size_t deg) {
  _degree_v = deg;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
inline void nurbssurface<point_t>::knotvector_u(iterator_t beg,
                                                iterator_t end) {
  _knotvector_u.resize(std::distance(beg, end));
  std::copy(beg, end, _knotvector_u.begin());
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
inline void nurbssurface<point_t>::knotvector_v(iterator_t beg,
                                                iterator_t end) {
  _knotvector_v.resize(std::distance(beg, end));
  std::copy(beg, end, _knotvector_v.begin());
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
inline void nurbssurface<point_t>::set_points(iterator_t begin,
                                              iterator_t end) {
  _points.clear();
  std::copy(begin, end, std::back_inserter(_points));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbssurface<point_t>::numberofpoints_u(size_t n) {
  _npoints_u = n;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbssurface<point_t>::numberofpoints_v(size_t n) {
  _npoints_v = n;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t nurbssurface<point_t>::numberofpoints_u() const {
  return _npoints_u;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t nurbssurface<point_t>::numberofpoints_v() const {
  return _npoints_v;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::vector<point_t> const& nurbssurface<point_t>::points() const {
  return _points;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::vector<typename point_t::value_type> const&
nurbssurface<point_t>::knotvector_u() const {
  return _knotvector_u;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::vector<typename point_t::value_type> const&
nurbssurface<point_t>::knotvector_v() const {
  return _knotvector_v;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename point_t::value_type nurbssurface<point_t>::umin() const {
  return _knotvector_u.front();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename point_t::value_type nurbssurface<point_t>::umax() const {
  return _knotvector_u.back();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename point_t::value_type nurbssurface<point_t>::vmin() const {
  return _knotvector_v.front();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename point_t::value_type nurbssurface<point_t>::vmax() const {
  return _knotvector_v.back();
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void nurbssurface<point_t>::print(std::ostream& os) const {
  os << "Order U : " << _degree_u + 1 << "  Order V : " << _degree_v + 1
     << std::endl;
  os << "Cols U  : " << _npoints_u << "  Rows V  : " << _npoints_v << std::endl;

  os << "knotU : ";
  std::copy(_knotvector_u.begin(),
            _knotvector_u.end(),
            std::ostream_iterator<value_type>(os, " "));

  os << std::endl << "knotV : ";
  std::copy(_knotvector_v.begin(),
            _knotvector_v.end(),
            std::ostream_iterator<value_type>(os, " "));

  os << "Control Points : \n";
  std::copy(
      _points.begin(), _points.end(), std::ostream_iterator<point_t>(os, "\n"));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
    inline std::ostream& operator<<(std::ostream& os,
                                    nurbssurface<point_t> const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace tml

#endif  // TML_NURBSSURFACE_IMPL_HPP
