/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : pointmesh2d_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_POINTMESH2D_IMPL_HPP
#define TML_POINTMESH2D_IMPL_HPP

// header, system
#include <cassert>   // std::assert
#include <vector>    // std::vector
#include <iterator>  // std::ostream_iterator
#include <numeric>
#include <algorithm>

// header, project
#include <gua/renderer/nurbs_geometry/tml/axis_aligned_boundingbox.hpp>

namespace tml {

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline pointmesh2d<point_t>::pointmesh2d()
    : _width(0), _height(0), _points() {}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline pointmesh2d<point_t>::pointmesh2d(std::size_t width, std::size_t height)
    : _width(width), _height(height), _points(_width * _height) {}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
inline pointmesh2d<point_t>::pointmesh2d(iterator_t beg,
                                         iterator_t end,
                                         std::size_t width,
                                         std::size_t height)
    : _width(width), _height(height), _points() {
  std::copy(beg, end, back_inserter(_points));
  assert(_points.size() == _width * _height);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline pointmesh2d<point_t>::pointmesh2d(pointmesh2d const& copy)
    : _width(copy._width), _height(copy._height), _points(copy._points) {}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline pointmesh2d<point_t>::~pointmesh2d() {}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline pointmesh2d<point_t>& pointmesh2d<point_t>::operator=(
    pointmesh2d const& cp) {
  pointmesh2d tmp(cp);
  swap(tmp);
  return *this;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh2d<point_t>::swap(pointmesh2d& swp) {
  std::swap(_width, swp._width);
  std::swap(_height, swp._height);
  std::swap(_points, swp._points);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
inline void pointmesh2d<point_t>::add_row(iterator_t begin, iterator_t end) {
  std::size_t oldsize(_points.size());

  std::copy(begin, end, std::back_inserter(_points));

  _width = _points.size() - oldsize;
  ++_height;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename container_t>
inline void pointmesh2d<point_t>::add_row(container_t const& c) {
  std::size_t oldsize(_points.size());

  std::copy(c.begin(), c.end(), std::back_inserter(_points));

  _width = _points.size() - oldsize;
  ++_height;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::vector<point_t> pointmesh2d<point_t>::row(std::size_t row) const {
  typename std::vector<point_t>::const_iterator row_begin = _points.begin();
  typename std::vector<point_t>::const_iterator row_end = _points.begin();

  // make sure the row is valid
  assert(row < _height);

  // set to correct position
  std::advance(row_begin, row * _width);
  std::advance(row_end, (row + 1) * _width);

  // copy correct range
  return std::vector<point_t>(row_begin, row_end);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::vector<point_t> pointmesh2d<point_t>::column(
    std::size_t col) const {
  // make sure the column is valid
  assert(col < _width);

  typename std::vector<point_t>::const_iterator p = _points.begin();
  std::vector<point_t> result;

  // put iterator to correct column
  std::advance(p, col);

  // copy points
  for (std::size_t i = 0; i < _height; ++i) {
    result.push_back(*p);
    if (i < (_height - 1))
      std::advance(p, _width);
  }

  return result;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh2d<point_t>::push_back(point_t const& p) {
  _points.push_back(p);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline bool pointmesh2d<point_t>::valid() const {
  return _points.size() == _height * _width;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline void pointmesh2d<point_t>::clear() {
  _points.clear();
  _height = 0;
  _width = 0;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t pointmesh2d<point_t>::size() const {
  return _points.size();
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t pointmesh2d<point_t>::width() const {
  return _width;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh2d<point_t>::width(std::size_t w) {
  _width = w;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t pointmesh2d<point_t>::height() const {
  return _height;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh2d<point_t>::height(std::size_t h) {
  _height = h;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename pointmesh2d<point_t>::iterator pointmesh2d<point_t>::begin() {
  return _points.begin();
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename pointmesh2d<point_t>::const_iterator
pointmesh2d<point_t>::begin() const {
  return _points.begin();
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename pointmesh2d<point_t>::iterator pointmesh2d<point_t>::end() {
  return _points.end();
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename pointmesh2d<point_t>::const_iterator
pointmesh2d<point_t>::end() const {
  return _points.end();
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline bool pointmesh2d<point_t>::equals(pointmesh2d const& o,
                                         value_type const& epsilon) const {
  /*if ( o.size() != size() || _points.empty() ) {
    return false;
  }
  else */
  {
    // heuristic : if min + max + mean + #poinst meshes are equal
    point_type mean_a =
        std::accumulate(_points.begin(), _points.end(), point_type());
    point_type mean_b =
        std::accumulate(o._points.begin(), o._points.end(), point_type());

    axis_aligned_boundingbox<point_type> box_a(_points.begin(), _points.end());
    axis_aligned_boundingbox<point_type> box_b(o._points.begin(),
                                               o._points.end());

    return mean_a.distance(mean_b) < epsilon &&
           box_a.min.distance(box_b.min) < epsilon &&
           box_a.max.distance(box_b.max) < epsilon;
  }
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline void pointmesh2d<point_t>::transpose() {
  // make sure polygon has correct parameters
  assert(_points.size() == _width * _height);

  pointmesh2d tmp;
  for (std::size_t i = 0; i < _width; ++i) {
    tmp.add_row(column(i));
  }

  swap(tmp);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline pointmesh2d<point_t> pointmesh2d<point_t>::subpatch(
    std::size_t left,
    std::size_t right,
    std::size_t top,
    std::size_t bottom) const {
  assert(right < _width);
  assert(bottom < _height);

  assert(right >= left);
  assert(bottom >= top);

  typename std::vector<point_t>::const_iterator linestart(_points.begin());
  typename std::vector<point_t>::const_iterator lineend(_points.begin());

  std::advance(linestart, left);  // put in correct column
  std::advance(lineend,
               right + 1);  // put in correct column (+1 because end-iterator)

  std::advance(linestart, top * _width);  // put in correct row
  std::advance(lineend, top * _width);    // put in correct row

  pointmesh2d subpatch;
  for (std::size_t row = top; row <= bottom; ++row) {
    subpatch.add_row(linestart, lineend);
    if (row < bottom) {
      std::advance(linestart, _width);
      std::advance(lineend, _width);
    }
  }
  return subpatch;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh2d<point_t>::print(std::ostream& os) const {
  std::size_t i = 0;

  for (const_iterator p = _points.begin(); p != _points.end(); ++p) {
    os << *p;

    if (i % _width == 0) {
      os << std::endl;
    }
  }

}

}  // namespace tml

#endif  // TML_POINTMESH_IMPL_HPP
