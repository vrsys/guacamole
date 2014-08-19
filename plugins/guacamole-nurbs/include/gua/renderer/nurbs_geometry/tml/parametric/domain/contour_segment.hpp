/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : contour_segment.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_CONTOUR_SEGMENT_HPP
#define TML_CONTOUR_SEGMENT_HPP

// includes, system
#include <cassert>

#include <boost/shared_ptr.hpp>

// includes. projects

namespace tml {

enum monotony_t {
  unclassified = 0,
  u_monotonic = 1,
  v_monotonic = 2,
  bi_monotonic = 3
};

template <typename value_t> class contour_segment {
 public:  // enums / typedefs

  typedef value_t value_type;
  typedef point<value_type, 2> point_type;
  typedef axis_aligned_boundingbox<point_type> bbox_type;

  typedef beziercurve<point_type> curve_type;
  typedef boost::shared_ptr<curve_type> curve_ptr;

  typedef std::vector<curve_ptr> curve_container;
  typedef typename curve_container::iterator curve_iterator;
  typedef typename curve_container::const_iterator const_curve_iterator;

 public:  // c'tor / d'tor

  template <typename curve_ptr_iterator_t>
  contour_segment(curve_ptr_iterator_t begin, curve_ptr_iterator_t end);

  ~contour_segment();

 public:  // methods

  // accessing attributes
  monotony_t monotony() const;

  bbox_type const& bbox() const;

  bool continous() const;
  bool increasing(typename point_type::coordinate_type const&) const;
  bool is_constant(typename point_type::coordinate_type const&) const;

  std::size_t size() const;
  const_curve_iterator begin() const;
  const_curve_iterator end() const;

  void clip_horizontal();

  // modifying methods
  void invert();

  // print out
  void print(std::ostream& os) const;

 private:  // methods

  void _determine_monotony();
  void _determine_continuity();
  void _update_bbox();

 private:  // attributes

  curve_container _curves;
  bbox_type _bbox;
  monotony_t _monotony;
  bool _continous;
};

template <typename point_type>
    std::ostream& operator<<(std::ostream& os,
                             tml::contour_segment<point_type> const&);

}  // namespace tml

#include "contour_segment_impl.hpp"

#endif  // TML_CONTOUR_SEGMENT_HPP
