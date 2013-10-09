/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : pointmesh3d_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_POINTMESH3D_IMPL_HPP
#define TML_POINTMESH3D_IMPL_HPP

// header, system
#include <cassert>    // std::assert
#include <vector>     // std::vector
#include <algorithm>  // std::for_each
#include <iterator>   // std::ostream_iterator

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/minimal_coordinates.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/maximal_coordinates.hpp>

namespace tml {

/////////////////////////////////////////////////////////////////////////////
template <typename point_t> struct normalize_adapter {
  typedef typename point_t::value_type float_type;

  normalize_adapter(point_t const& offset, float_type const& scale)
      : _offset(offset), _scale(scale) {}

  void operator()(point_t& p) const {
    float_type weight = p.weight();

    // transform into unit space
    p -= _offset;
    p /= _scale;

    // keep weight of original
    p.weight(weight);
  }

  point_t _offset;
  float_type _scale;
};

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline pointmesh3d<point_t>::pointmesh3d()
    : _width(0), _height(0), _depth(0), _points() {}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline pointmesh3d<point_t>::pointmesh3d(std::size_t width,
                                         std::size_t height,
                                         std::size_t depth)
    : _width(width),
      _height(height),
      _depth(depth),
      _points(width * height * depth) {}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
inline pointmesh3d<point_t>::pointmesh3d(iterator_t beg,
                                         iterator_t end,
                                         std::size_t width,
                                         std::size_t height,
                                         std::size_t depth)
    : _width(width), _height(height), _depth(depth), _points() {
  assert(std::size_t(std::distance(beg, end)) == width * height * depth);
  std::copy(beg, end, back_inserter(_points));
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline pointmesh3d<point_t>::pointmesh3d(pointmesh3d const& rhs)
    : _width(rhs._width),
      _height(rhs._height),
      _depth(rhs._depth),
      _points(rhs._points) {}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline pointmesh3d<point_t>::~pointmesh3d() {}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline pointmesh3d<point_t>& pointmesh3d<point_t>::operator=(
    pointmesh3d const& cp) {
  pointmesh3d tmp(cp);
  swap(tmp);
  return *this;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline point_t& pointmesh3d<point_t>::operator()(std::size_t u,
                                                 std::size_t v,
                                                 std::size_t w) {
  std::size_t index = u + _width * v + _width * _height * w;
  assert(index < _points.size());
  return _points[index];
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline point_t const& pointmesh3d<point_t>::operator()(std::size_t u,
                                                       std::size_t v,
                                                       std::size_t w) const {
  std::size_t index = u + _width * v + _width * _height * w;
  assert(index < _points.size());
  return _points[index];
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh3d<point_t>::swap(pointmesh3d& swp) {
  std::swap(_width, swp._width);
  std::swap(_height, swp._height);
  std::swap(_depth, swp._depth);
  std::swap(_points, swp._points);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh3d<point_t>::push_back(point_t const& p) {
  _points.push_back(p);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline bool pointmesh3d<point_t>::valid() const {
  return _points.size() == _width * _height * _depth;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline void pointmesh3d<point_t>::clear() {
  _points.clear();
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t pointmesh3d<point_t>::size() const {
  return _points.size();
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh3d<point_t>::resize(std::size_t n) {
  _points.resize(n);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh3d<point_t>::resize(std::size_t width,
                                         std::size_t height,
                                         std::size_t depth) {
  _points.resize(width * height * depth);
  _width = width;
  _height = height;
  _depth = depth;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh3d<point_t>::width(std::size_t n) {
  _width = n;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh3d<point_t>::height(std::size_t n) {
  _height = n;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh3d<point_t>::depth(std::size_t n) {
  _depth = n;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh3d<point_t>::transpose(std::size_t width,
                                            std::size_t height,
                                            std::size_t depth) {
  pointmesh3d tmp;
  tmp.resize(_points.size());

  switch (width) {
    case 0:
      tmp._width = _width;
      break;
    case 1:
      tmp._height = _width;
      break;
    case 2:
      tmp._depth = _width;
      break;
  }

  switch (height) {
    case 0:
      tmp._width = _height;
      break;
    case 1:
      tmp._height = _height;
      break;
    case 2:
      tmp._depth = _height;
      break;
  }

  switch (depth) {
    case 0:
      tmp._width = _depth;
      break;
    case 1:
      tmp._height = _depth;
      break;
    case 2:
      tmp._depth = _depth;
      break;
  }

  for (unsigned int w = 0; w != _depth; ++w) {
    for (unsigned int v = 0; v != _height; ++v) {
      for (unsigned int u = 0; u != _width; ++u) {
        std::size_t source_index = u + v * _width + w * _width * _height;
        std::size_t target_index = 0;

        switch (width) {
          case 0:
            target_index += u;
            break;
          case 1:
            target_index += u * tmp._width;
            break;
          case 2:
            target_index += u * tmp._width * tmp._height;
            break;
        }

        switch (height) {
          case 0:
            target_index += v;
            break;
          case 1:
            target_index += v * tmp._width;
            break;
          case 2:
            target_index += v * tmp._width * tmp._height;
            break;
        }

        switch (depth) {
          case 0:
            target_index += w;
            break;
          case 1:
            target_index += w * tmp._width;
            break;
          case 2:
            target_index += w * tmp._width * tmp._height;
            break;
        }

        tmp._points[target_index] = _points[source_index];
      }
    }
  }

  swap(tmp);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t> inline void pointmesh3d<point_t>::normalize() {
  float_type epsilon = float_type(0.00001);

  boundingbox_type b = bbox();
  float_type rescale_factor = (b.max - b.min).abs();

  if (rescale_factor > epsilon) {
    std::for_each(_points.begin(),
                  _points.end(),
                  normalize_adapter<point_t>(b.min, rescale_factor));
  }
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh3d<point_t>::scale(float_type const& s) {
  boundingbox_type b = bbox();
  std::for_each(_points.begin(),
                _points.end(),
                normalize_adapter<point_t>(point_t(), float_type(1) / s));
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh3d<point_t>::displace(pointmesh3d const& offset_mesh) {
  assert(offset_mesh.size() == _points.size());

  auto i = _points.begin();
  auto j = offset_mesh.begin();

  for (; i != _points.end(); ++i, ++j) {
    auto w = i->weight();
    (*i) += *j;
    i->weight(w);
  }
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t pointmesh3d<point_t>::width() const {
  return _width;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t pointmesh3d<point_t>::height() const {
  return _height;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::size_t pointmesh3d<point_t>::depth() const {
  return _depth;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename pointmesh3d<point_t>::iterator pointmesh3d<point_t>::begin() {
  return _points.begin();
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename pointmesh3d<point_t>::const_iterator
pointmesh3d<point_t>::begin() const {
  return _points.begin();
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename pointmesh3d<point_t>::iterator pointmesh3d<point_t>::end() {
  return _points.end();
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename pointmesh3d<point_t>::const_iterator
pointmesh3d<point_t>::end() const {
  return _points.end();
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline std::vector<point_t> pointmesh3d<point_t>::submesh(
    std::size_t dim,
    std::size_t val0,
    std::size_t val1) const {
  std::vector<point_t> result;

  switch (dim) {
    case 0: {
      result.resize(_width);
      std::size_t offset = val0 * _width + val1 * _width * _height;
      std::copy(_points.begin() + offset,
                _points.begin() + offset + _width,
                result.begin());
      break;
    }
    case 1: {
      result.resize(_height);
      std::size_t offset = val0 + val1 * _width * _height;
      std::size_t stride = _width;
      for (unsigned int i = 0; i != _height; ++i) {
        result[i] = _points[offset + i * stride];
      }
      break;
    }
    case 2: {
      result.resize(_depth);
      std::size_t offset = val0 + val1 * _width;
      std::size_t stride = _width * _height;
      for (unsigned int i = 0; i != _depth; ++i) {
        result[i] = _points[offset + i * stride];
      }
      break;
    }
  }

  return result;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
inline void pointmesh3d<point_t>::submesh(iterator_t begin,
                                          std::size_t dim,
                                          std::size_t val0,
                                          std::size_t val1) {
  switch (dim) {
    case 0: {
      std::size_t offset = val0 * _width + val1 * _width * _height;
      std::size_t stride = 1;
      for (unsigned int i = 0; i != _width; ++i) {
        _points[offset + i * stride] = *(begin++);
      }
      break;
    }
    case 1: {
      std::size_t offset = val0 + val1 * _width * _height;
      std::size_t stride = _width;
      for (unsigned int i = 0; i != _height; ++i) {
        _points[offset + i * stride] = *(begin++);
      }
      break;
    }
    case 2: {
      std::size_t offset = val0 + val1 * _width;
      std::size_t stride = _width * _height;
      for (unsigned int i = 0; i != _depth; ++i) {
        _points[offset + i * stride] = *(begin++);
      }
      break;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline pointmesh2d<point_t> pointmesh3d<point_t>::submesh(std::size_t dim,
                                                          std::size_t n) const {
  return conv_submesh<point_t>(dim, n);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename target_t>
inline pointmesh2d<target_t> pointmesh3d<point_t>::conv_submesh(
    std::size_t dim,
    std::size_t n,
    target_t const& /*dummy*/) const {
  std::size_t dx = 1;
  std::size_t dy = _width;
  std::size_t dz = _width * _height;

  switch (dim) {
    case u: {
      std::size_t target_size = _height * _depth;
      std::size_t source_index = n;
      pointmesh2d<target_t> mesh2d;

      for (std::size_t i = 0; i != target_size; ++i) {
        mesh2d.push_back(_points[source_index]);
        source_index += dy;
      }

      mesh2d.width(_height);
      mesh2d.height(_depth);

      return mesh2d;
    }

    case v: {
      std::size_t source_index = n * dy;
      pointmesh2d<target_t> mesh2d;

      for (std::size_t i = 0; i != _depth; ++i) {
        for (std::size_t j = 0; j != _width; ++j) {
          mesh2d.push_back(_points[source_index]);
          source_index += dx;
        }
        source_index += dz - _width;
      }

      mesh2d.width(_width);
      mesh2d.height(_depth);

      return mesh2d;
    }

    case w: {
      std::size_t target_size = _width * _height;
      std::size_t source_index = n * dz;
      pointmesh2d<target_t> mesh2d;

      for (std::size_t i = 0; i != target_size; ++i) {
        mesh2d.push_back(_points[source_index]);
        source_index += dx;
      }

      mesh2d.width(_width);
      mesh2d.height(_height);

      return mesh2d;
    }

    default:
      throw std::runtime_error("Dimension not allowed.\n");
  }

#if !WIN32
  // gcc workaround ... cannot reach this point
  return pointmesh2d<target_t>();
#endif
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline pointmesh3d<point_t> pointmesh3d<point_t>::submesh(
    std::size_t offset_u,
    std::size_t offset_v,
    std::size_t offset_w,
    std::size_t npoints_u,
    std::size_t npoints_v,
    std::size_t npoints_w) const {
  assert(offset_u + npoints_u <= _width && offset_v + npoints_v <= _height &&
         offset_w + npoints_w <= _depth);

  pointmesh3d mesh(npoints_u, npoints_v, npoints_w);

  for (std::size_t w = offset_w; w != offset_w + npoints_w; ++w) {
    for (std::size_t v = offset_v; v != offset_v + npoints_v; ++v) {
      for (std::size_t u = offset_u; u != offset_u + npoints_u; ++u) {
        mesh(u - offset_u, v - offset_v, w - offset_w) = (*this)(u, v, w);
      }
    }
  }

  return mesh;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline typename pointmesh3d<point_t>::boundingbox_type
pointmesh3d<point_t>::bbox() const {
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

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh3d<point_t>::print(std::ostream& os) const {
  std::copy(
      _points.begin(), _points.end(), std::ostream_iterator<point_t>(os, "\n"));
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
inline void pointmesh3d<point_t>::write(std::ostream& os) const {
  os.write(reinterpret_cast<char const*>(&_width), sizeof(std::size_t));
  os.write(reinterpret_cast<char const*>(&_height), sizeof(std::size_t));
  os.write(reinterpret_cast<char const*>(&_depth), sizeof(std::size_t));

  assert(_width != 0 && _height != 0 && _depth != 0);

  os.write(reinterpret_cast<char const*>(&_points.front()),
           sizeof(point_type) * _points.size());
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t> void pointmesh3d<point_t>::read(std::istream& is) {
  is.read(reinterpret_cast<char*>(&_width), sizeof(std::size_t));
  is.read(reinterpret_cast<char*>(&_height), sizeof(std::size_t));
  is.read(reinterpret_cast<char*>(&_depth), sizeof(std::size_t));

  _points.resize(_width * _height * _depth);

  is.read(reinterpret_cast<char*>(&_points.front()),
          sizeof(point_type) * _points.size());
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_t>
    inline std::ostream& operator<<(std::ostream& os,
                                    pointmesh3d<point_t> const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace tml

#endif  // TML_POINTMESH3D_IMPL_HPP
