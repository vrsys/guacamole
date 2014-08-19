/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : abstract_boundingbox.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_ABSTRACT_BOUNDING_BOX_HPP
#define TML_ABSTRACT_BOUNDING_BOX_HPP

// header, system
#include <list>

#include <boost/foreach.hpp>
#include <boost/numeric/conversion/bounds.hpp>
#include <boost/shared_ptr.hpp>

// header, project

#include <gua/renderer/nurbs_geometry/tml/matrix.hpp>

namespace tml {

template <typename point_t> class abstract_boundingbox {
 public:  // typedefs

  static unsigned const N = point_t::coordinates;

  typedef abstract_boundingbox<point_t> type;
  typedef boost::shared_ptr<type> pointer_type;

  typedef typename point_t::value_type value_type;
  typedef point_t point_type;
  typedef matrix<value_type, N, N> matrix_type;

 public:  // ctor/dtor

  abstract_boundingbox() {}
  virtual ~abstract_boundingbox() {}

 public:  // methods

  virtual point_t center() const = 0;
  virtual value_type volume() const = 0;
  virtual value_type surface() const = 0;

  virtual value_type distance(abstract_boundingbox<point_t> const& a) const;

  virtual void generate_corners(std::list<point_t>&) const = 0;
  virtual void generate_corners(std::vector<point_t>&) const = 0;

  virtual bool is_inside(point_t const& p) const = 0;

  virtual void uniform_split(std::list<pointer_type>&) const = 0;

  virtual void print(std::ostream& os) const = 0;

  virtual void write(std::ostream& os) const = 0;
  virtual void read(std::istream& is) = 0;

 protected:  // attributes

};

///////////////////////////////////////////////////////////////////////////////
template <typename point_t>
typename point_t::value_type abstract_boundingbox<point_t>::distance(
    abstract_boundingbox<point_t> const& b) const {
  // 1. compute ray box center(A)->center(B) and vice versa (normalized
  // direction)
  point_t d = b.center() - center();
  value_type center_distance = d.abs();
  point_t dAB = d;
  point_t dBA = value_type(-1) * d;

  // normalize direction
  dAB /= center_distance;
  dBA /= center_distance;

  // 2. project all corners onto ray and find maximum (nearest point to other
  // box)
  std::vector<point_t> cornersA;
  std::vector<point_t> cornersB;

  generate_corners(cornersA);
  b.generate_corners(cornersB);

  value_type dA = boost::numeric::bounds<value_type>::highest();
  value_type dB = boost::numeric::bounds<value_type>::highest();

  BOOST_FOREACH(point_t const & c, cornersA) {
    dA = std::min(dA, std::max(value_type(0), dot(dAB, c)));
  }

  BOOST_FOREACH(point_t const & c, cornersB) {
    dB = std::min(dB, std::max(value_type(0), dot(dBA, c)));
  }

  return std::max(value_type(0), center_distance - dA - dB);
}

}  // namespace tml

#endif  // TML_ABSTRACT_BOUNDING_BOX_HPP
