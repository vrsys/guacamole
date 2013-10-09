/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : beziercurve_segment.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_BEZIERCURVE_SEGMENT_HPP
#define TML_BEZIERCURVE_SEGMENT_HPP

// includes, system
#include <cassert>

#include <boost/shared_ptr.hpp>

// includes. projects
#include <gua/renderer/nurbs_geometry/tml/parametric/beziercurve.hpp>
#include <gua/renderer/nurbs_geometry/tml/axis_aligned_boundingbox.hpp>
#include <gua/renderer/nurbs_geometry/tml/interval.hpp>

namespace tml {

template <typename point_type> class beziercurve_segment {
 public:  // enums / typedefs

  typedef typename point_type::value_type value_type;
  typedef axis_aligned_boundingbox<point_type> bbox_type;
  typedef interval<value_type> interval_type;

  typedef beziercurve<point_type> curve_type;
  typedef boost::shared_ptr<curve_type> curve_ptr_type;

 public:  // c'tor / d'tor

  beziercurve_segment();

  beziercurve_segment(curve_ptr_type curve,
                      value_type tmin,
                      value_type tmax,
                      point_type const& pmin,
                      point_type const& pmax);

  ~beziercurve_segment();

 public:  // methods

  curve_ptr_type curve() const;

  value_type tmin() const;
  value_type tmax() const;

  point_type const& front() const;
  point_type const& back() const;

  value_type minimum(std::size_t axis) const;
  value_type maximum(std::size_t axis) const;

  void apply(curve_ptr_type curve,
             value_type tmin,
             value_type tmax,
             point_type const& pmin,
             point_type const& pmax);

  void split(value_type t,
             beziercurve_segment& less,
             beziercurve_segment& more) const;

  void split(value_type t, curve_ptr_type less, curve_ptr_type more) const;

  bbox_type boundingbox() const;

  interval_type delta(std::size_t axis) const;

  value_type distance_to_bbox(point_type const& p) const;

  bool is_constant(std::size_t axis) const;
  bool is_increasing(std::size_t axis) const;

  void print(std::ostream& os, std::string const& additional_info = "") const;

 private:

  curve_ptr_type _curve;

  value_type _tmin;
  value_type _tmax;

  point_type _pmin;
  point_type _pmax;
};

template <typename point_type>
    std::ostream& operator<<(std::ostream& os,
                             tml::beziercurve_segment<point_type> const&);

}  // namespace tml

#include <gua/beziercurve_segment_impl.hpp>

#endif  // TML_BEZIERCURVE_SEGMENT_HPP
