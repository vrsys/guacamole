/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : nurbscurve.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_NURBSCURVE_HPP
#define TML_NURBSCURVE_HPP

// header, system
#include <algorithm>  // std::sort
#include <cmath>      // std::max
#include <iosfwd>     // std::cout
#include <iterator>   // std::ostream_iterator
#include <vector>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

namespace tml {
template <typename point_t> class nurbscurve {
 public:

  typedef point_t point_type;
  typedef typename point_t::value_type value_type;
  typedef std::vector<point_t> point_container;
  typedef typename point_container::iterator point_iterator;
  typedef typename point_container::const_iterator const_point_iterator;

 public:  // c'tor / d'tor

  nurbscurve();
  nurbscurve(nurbscurve const& rhs);
  virtual ~nurbscurve();

 public:  // methods

  virtual void swap(nurbscurve& rhs);
  virtual nurbscurve& operator=(nurbscurve const& rhs);

  virtual void print(std::ostream& os) const;

  void add(point_t const& point);

  void add_knot(value_type knot);
  void add_knot(value_type knot, std::size_t m);

  void clear();

  bool verify() const;

  size_t degree() const;
  size_t order() const;
  size_t size() const;

  void degree(size_t deg);

  void normalize_knotvector();

  template <typename iterator_t>
  void set_knotvector(iterator_t begin, iterator_t end);

  template <typename iterator_t>
  void set_points(iterator_t begin, iterator_t end);

  std::vector<point_t> const& points() const;
  std::vector<value_type> const& knots() const;

  point_iterator begin();
  point_iterator end();
  const_point_iterator begin() const;
  const_point_iterator end() const;

  point_t const& front() const;
  point_t const& back() const;

  point_t& operator[](std::size_t index);
  point_t const& operator[](std::size_t index) const;

 private:  // atttributes

  size_t _degree;
  std::vector<point_t> _points;
  std::vector<value_type> _knots;
};

template <typename point_t>
    std::ostream& operator<<(std::ostream& os, nurbscurve<point_t> const& nc);

typedef nurbscurve<point2f> nurbscurve2f;
typedef nurbscurve<point2d> nurbscurve2d;
typedef nurbscurve<point3f> nurbscurve3f;
typedef nurbscurve<point3d> nurbscurve3d;

}  // namespace tml

#include "implementation/nurbscurve_impl.hpp"

#endif  // TML_NURBSCURVE_HPP
