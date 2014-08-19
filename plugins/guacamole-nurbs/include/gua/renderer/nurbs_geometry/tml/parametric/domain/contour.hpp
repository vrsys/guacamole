/********************************************************************************
*
* Copyright (C) 2013 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : contour.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_CONTOUR_HPP
#define TML_CONTOUR_HPP

// includes, system
#include <cassert>

#include <boost/shared_ptr.hpp>

// includes. projects
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/beziercurve.hpp>
#include <gua/renderer/nurbs_geometry/tml/axis_aligned_boundingbox.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/contour_segment.hpp>

namespace tml {

template <typename value_t> class contour {
 public:  // enums / typedefs

  typedef value_t value_type;
  typedef point<value_type, 2> point_type;
  typedef axis_aligned_boundingbox<point_type> bbox_type;

  typedef beziercurve<point_type> curve_type;
  typedef boost::shared_ptr<curve_type> curve_ptr;

  typedef contour_segment<value_type> contour_segment_type;
  typedef boost::shared_ptr<contour_segment_type> contour_segment_ptr;

  typedef std::vector<curve_ptr> curve_container;
  typedef typename curve_container::iterator curve_iterator;
  typedef typename curve_container::const_iterator const_curve_iterator;

 public:  // c'tor / d'tor

  template <typename curve_ptr_iterator_t>
  contour(curve_ptr_iterator_t begin, curve_ptr_iterator_t end);

  ~contour();

 public:  // methods

  // check if contour defines a non-overlapping. piecewise continous and closed
  // boundary
  bool valid() const;

  bool empty() const;
  std::size_t size() const;

  const_curve_iterator begin() const;
  const_curve_iterator end() const;

  // split contour into bi-monotonic pieces
  template <typename contour_segment_ptr_container>
  void monotonize(contour_segment_ptr_container& target_container) const;

  // print to output stream
  void print(std::ostream& os) const;

 private:

  curve_container _curves;
};

template <typename value_t>
    std::ostream& operator<<(std::ostream& os, tml::contour<value_t> const&);

}  // namespace tml

#include "contour_impl.hpp"

#endif  // TML_CONTOUR_HPP
