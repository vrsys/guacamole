/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : nurbssurface.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_NURBSSURFACE_HPP
#define TML_NURBSSURFACE_HPP

// header, system
#include <cassert>  // std::assert
#include <vector>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

namespace tml {

template <typename point_t> class nurbssurface {
 public:  // typedefs, enums

  typedef point_t point_type;
  typedef typename point_t::value_type value_type;
  typedef std::vector<point_t> container_type;
  typedef std::vector<value_type> knotvector_type;

 public:  // c'tor / d'tor

  nurbssurface();
  nurbssurface(nurbssurface const& rhs);
  virtual ~nurbssurface();

 public:  // methods

  virtual void swap(nurbssurface& rhs);
  virtual nurbssurface& operator=(nurbssurface const& rhs);

  virtual void print(std::ostream& os) const;

  bool verify() const;

  size_t degree_u() const;
  size_t order_u() const;
  size_t degree_v() const;
  size_t order_v() const;

  void degree_u(size_t deg);
  void degree_v(size_t deg);

  template <typename iterator_t>
  void knotvector_u(iterator_t beg, iterator_t end);

  template <typename iterator_t>
  void knotvector_v(iterator_t beg, iterator_t end);

  template <typename iterator_t>
  void set_points(iterator_t beg, iterator_t end);

  void numberofpoints_u(size_t n);
  void numberofpoints_v(size_t n);

  std::size_t numberofpoints_u() const;
  std::size_t numberofpoints_v() const;

  std::vector<point_type> const& points() const;

  knotvector_type const& knotvector_u() const;
  knotvector_type const& knotvector_v() const;

  value_type umin() const;
  value_type umax() const;
  value_type vmin() const;
  value_type vmax() const;

 private:  // data members

  // degree for parameter
  size_t _degree_u;
  size_t _degree_v;

  // number of control points for parameter
  size_t _npoints_u;
  size_t _npoints_v;

  // control points
  container_type _points;

  // knot vectors
  knotvector_type _knotvector_u;
  knotvector_type _knotvector_v;
};

template <typename point_t>
    std::ostream& operator<<(std::ostream& os,
                             nurbssurface<point_t> const& rhs);

typedef nurbssurface<point3f> nurbssurface3f;
typedef nurbssurface<point3d> nurbssurface3d;

}  // namespace tml

#include "implementation/nurbssurface_impl.hpp"

#endif  // TML_NURBSSURFACE_HPP
