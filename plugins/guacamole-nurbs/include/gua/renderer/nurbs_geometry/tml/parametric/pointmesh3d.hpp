/********************************************************************************
*
* Copyright (C) 2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : pointmesh3d.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_POINTMESH3D_HPP
#define TML_POINTMESH3D_HPP

// header, system
#include <cassert>   // std::assert
#include <vector>    // std::vector
#include <iterator>  // std::ostream_iterator

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/pointmesh2d.hpp>
#include <gua/renderer/nurbs_geometry/tml/axis_aligned_boundingbox.hpp>

namespace tml {

template <typename point_t> class pointmesh3d {

 public:  // typedefs

  typedef point_t point_type;
  typedef point_type value_type;
  typedef typename point_t::value_type float_type;
  typedef typename std::vector<point_t> container_type;
  typedef typename container_type::reference reference;
  typedef typename container_type::const_reference const_reference;
  typedef typename container_type::iterator iterator;
  typedef typename container_type::const_iterator const_iterator;

  typedef axis_aligned_boundingbox<point_t> boundingbox_type;

  static size_t const u = 0;
  static size_t const v = 1;
  static size_t const w = 2;

 public:  // c'tors and d'tor

  pointmesh3d();

  pointmesh3d(std::size_t width, std::size_t height, std::size_t depth);

  template <typename iterator_t>
  pointmesh3d(iterator_t begin,
              iterator_t end,
              std::size_t width,
              std::size_t height,
              std::size_t depth);

  pointmesh3d(pointmesh3d const& copy);

  ~pointmesh3d();

 public:  // methods

  pointmesh3d& operator=(pointmesh3d const& cp);

  point_t& operator()(std::size_t u, std::size_t v, std::size_t w);
  point_t const& operator()(std::size_t u, std::size_t v, std::size_t w) const;

  void swap(pointmesh3d& swp);

  void push_back(point_t const& p);

  bool valid() const;

  void clear();

  std::size_t size() const;
  void resize(std::size_t n);
  void resize(std::size_t width, std::size_t height, std::size_t depth);

  void width(std::size_t n);
  void height(std::size_t n);
  void depth(std::size_t n);

  void transpose(std::size_t width, std::size_t height, std::size_t depth);
  void normalize();
  void scale(float_type const& s);

  void displace(pointmesh3d const& offset);

  std::size_t width() const;
  std::size_t height() const;
  std::size_t depth() const;

  iterator begin();
  const_iterator begin() const;

  iterator end();
  const_iterator end() const;

  // get a sub-curve in direction dim along the fixed dimensions val0, val1
  std::vector<point_t> submesh(std::size_t dim,
                               std::size_t val0,
                               std::size_t val1) const;

  // set a subcurve in pointmesh along axis dim at the fixed dimensions val0,
  // val1
  template <typename iterator_t>
  void submesh(iterator_t begin,
               std::size_t dim,
               std::size_t val0,
               std::size_t val1);

  // get a subsurface in pointmesh along axis dim
  pointmesh2d<point_t> submesh(std::size_t dim,
                               /* 0 for width, 1 for height, 2 for depth */
                               std::size_t slice) const;

  // get a subsurface in pointmesh along axis dim for an arbitrary point_type
  template <typename target_t>
  pointmesh2d<target_t> conv_submesh(
      std::size_t dim, /* 0 for width, 1 for height, 2 for depth */
      std::size_t slice,
      target_t const& dummy = target_t()) const;

  pointmesh3d submesh(std::size_t offset_u,
                      std::size_t offset_v,
                      std::size_t offset_w,
                      std::size_t width,
                      std::size_t height,
                      std::size_t depth) const;

  boundingbox_type bbox() const;

  void print(std::ostream& os) const;
  void write(std::ostream& os) const;
  void read(std::istream& is);

 private:  // members

  std::size_t _width;
  std::size_t _height;
  std::size_t _depth;
  std::vector<point_type> _points;
};

template <typename point_t>
    std::ostream& operator<<(std::ostream& os, pointmesh3d<point_t> const& rhs);

}  // namespace tml

#include "implementation/pointmesh3d_impl.hpp"

#endif  // TML_POINTMESH_HPP
