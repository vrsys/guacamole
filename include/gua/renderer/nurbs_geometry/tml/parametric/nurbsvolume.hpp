/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : nurbsvolume.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_NURBSVOLUME_HPP
#define TML_NURBSVOLUME_HPP

// header, system
#include <cassert>  // std::assert
#include <vector>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/pointmesh3d.hpp>

namespace tml {

template <typename point_t> class nurbsvolume {
 public:  // typedefs, enums

  typedef point_t point_type;
  typedef typename point_t::value_type value_type;
  typedef std::vector<value_type> knotvector_type;

  typedef pointmesh3d<point_type> mesh_type;
  typedef typename mesh_type::iterator iterator;
  typedef typename mesh_type::const_iterator const_iterator;

  typedef typename mesh_type::boundingbox_type boundingbox_type;

 public:  // c'tor / d'tor

  nurbsvolume();
  nurbsvolume(nurbsvolume const& rhs);
  virtual ~nurbsvolume();

 public:  // methods

  virtual void swap(nurbsvolume& rhs);
  virtual nurbsvolume& operator=(nurbsvolume const& rhs);

  virtual void clear();

  virtual void print(std::ostream& os) const;
  virtual void write(std::ostream& os) const;
  virtual void read(std::istream& is);

  bool verify() const;

  size_t degree_u() const;
  size_t order_u() const;
  size_t degree_v() const;
  size_t order_v() const;
  size_t degree_w() const;
  size_t order_w() const;

  void degree_u(size_t deg);
  void degree_v(size_t deg);
  void degree_w(size_t deg);

  template <typename iterator_t>
  void knotvector_u(iterator_t beg, iterator_t end);

  template <typename iterator_t>
  void knotvector_v(iterator_t beg, iterator_t end);

  template <typename iterator_t>
  void knotvector_w(iterator_t beg, iterator_t end);

  // point operations
  template <typename iterator_t>
  void set_points(iterator_t beg, iterator_t end);
  void set_point(std::size_t pos_u,
                 std::size_t pos_v,
                 std::size_t pos_w,
                 point_t const& p);
  void resize(std::size_t size_u, std::size_t size_v, std::size_t size_w);

  void transpose(std::size_t target_dim_u,
                 std::size_t target_dim_v,
                 std::size_t target_dim_w);

  void numberofpoints_u(size_t n);
  void numberofpoints_v(size_t n);
  void numberofpoints_w(size_t n);

  std::size_t numberofpoints_u() const;
  std::size_t numberofpoints_v() const;
  std::size_t numberofpoints_w() const;

  mesh_type const& points() const;
  boundingbox_type bbox() const;

  knotvector_type const& knotvector_u() const;
  knotvector_type const& knotvector_v() const;
  knotvector_type const& knotvector_w() const;

  unsigned knotspans_u() const;
  unsigned knotspans_v() const;
  unsigned knotspans_w() const;

  value_type umin() const;
  value_type umax() const;
  value_type vmin() const;
  value_type vmax() const;
  value_type wmin() const;
  value_type wmax() const;

  void normalize();
  void scale(value_type const& s);

 protected:  // data members

  // degree for parameter
  size_t _degree_u;
  size_t _degree_v;
  size_t _degree_w;

  // control points
  mesh_type _points;

  // knot vectors
  knotvector_type _knotvector_u;
  knotvector_type _knotvector_v;
  knotvector_type _knotvector_w;

  //unsigned                _knotspans_u;
  //unsigned                _knotspans_v;
  //unsigned                _knotspans_w;
};

template <typename point_t>
    std::ostream& operator<<(std::ostream& os, nurbsvolume<point_t> const& rhs);

typedef nurbsvolume<point3f> nurbsvolume3f;
typedef nurbsvolume<point3d> nurbsvolume3d;

}  // namespace tml

#include "implementation/nurbsvolume_impl.hpp"

#endif  // TML_NURBSVOLUME_HPP
