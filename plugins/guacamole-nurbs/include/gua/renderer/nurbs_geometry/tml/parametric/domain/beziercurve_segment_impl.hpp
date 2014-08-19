/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : beziercurve_segment_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/decasteljau.hpp>

namespace tml {

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
beziercurve_segment<point_type>::beziercurve_segment()
    : _curve(), _tmin(), _tmax(), _pmin(), _pmax() {}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
beziercurve_segment<point_type>::beziercurve_segment(curve_ptr_type curve,
                                                     value_type tmin,
                                                     value_type tmax,
                                                     point_type const& pmin,
                                                     point_type const& pmax)
    : _curve(curve), _tmin(tmin), _tmax(tmax), _pmin(pmin), _pmax(pmax) {}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
beziercurve_segment<point_type>::~beziercurve_segment() {}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename beziercurve_segment<point_type>::curve_ptr_type
beziercurve_segment<point_type>::curve() const {
  return _curve;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename beziercurve_segment<point_type>::value_type
beziercurve_segment<point_type>::tmin() const {
  return _tmin;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename beziercurve_segment<point_type>::value_type
beziercurve_segment<point_type>::tmax() const {
  return _tmax;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
point_type const& beziercurve_segment<point_type>::front() const {
  return _pmin;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
point_type const& beziercurve_segment<point_type>::back() const {
  return _pmax;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename point_type::value_type beziercurve_segment<point_type>::minimum(
    std::size_t axis) const {
  return _pmin[axis] < _pmax[axis] ? _pmin[axis] : _pmax[axis];
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename point_type::value_type beziercurve_segment<point_type>::maximum(
    std::size_t axis) const {
  return _pmin[axis] > _pmax[axis] ? _pmin[axis] : _pmax[axis];
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
void beziercurve_segment<point_type>::apply(
    typename beziercurve_segment<point_type>::curve_ptr_type curve_ptr,
    typename beziercurve_segment<point_type>::value_type parameter_min,
    typename beziercurve_segment<point_type>::value_type parameter_max,
    point_type const& point_min,
    point_type const& point_max) {
  _curve = curve_ptr;
  _tmin = parameter_min;
  _tmax = parameter_max;
  _pmin = point_min;
  _pmax = point_max;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
void beziercurve_segment<point_type>::split(
    typename beziercurve_segment<point_type>::value_type t,
    beziercurve_segment<point_type>& less,
    beziercurve_segment<point_type>& more) const {
  assert(t > _tmin && t < _tmax && _curve);

  using namespace tml;

  decasteljau<point_type> evaluate_decasteljau;
  point_type split_point;

  evaluate_decasteljau(_curve->begin(), _curve->order(), t, split_point);

  less.apply(_curve, _tmin, t, _pmin, split_point);
  more.apply(_curve, t, _tmax, split_point, _pmax);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
void beziercurve_segment<point_type>::split(
    typename beziercurve_segment<point_type>::value_type t,
    curve_ptr_type less,
    curve_ptr_type more) const {
  assert(t > _tmin && t < _tmax && _curve);

  using namespace tml;

  decasteljau<point_type> evaluate_decasteljau;
  point_type split_point;

  evaluate_decasteljau(curve->begin(), curve->order(), t, split_point);

  less->apply(_curve, _tmin, t, _pmin, split_point);
  more->apply(_curve, t, _tmax, split_point, _pmax);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename beziercurve_segment<point_type>::bbox_type
beziercurve_segment<point_type>::boundingbox() const {
  return bbox_type(tml::elementwise_min(_pmin, _pmax),
                   tml::elementwise_max(_pmin, _pmax));
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename beziercurve_segment<point_type>::interval_type
beziercurve_segment<point_type>::delta(std::size_t axis) const {
  return interval_type(_pmax[axis], _pmin[axis], tml::included, tml::included);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename beziercurve_segment<point_type>::value_type
beziercurve_segment<point_type>::distance_to_bbox(point_type const& p) const {
  if (this->axis_aligned_boundingbox().is_inside(p)) {
    return value_type(0);
  } else {
    point_type distance;
    for (std::size_t i = 0; i != point_type::size; ++i) {
      distance[i] = std::min(fabs(p[i] - _pmin[i]), fabs(p[i] - _pmax[i]));
    }
    return distance.abs();
  }
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
bool beziercurve_segment<point_type>::is_constant(std::size_t axis) const {
  return _curve->is_constant(axis);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
bool beziercurve_segment<point_type>::is_increasing(std::size_t axis) const {
  return _curve->is_increasing(axis);
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
void beziercurve_segment<point_type>::print(std::ostream& os,
                                            std::string const& addinfo) const {
  os << "beziercurve segment: \n";
  os << *_curve << std::endl;
  os << "t = " << _tmin << " - " << _tmax << std::endl;
  os << "p : " << _pmin << " to " << _pmax << addinfo;
}

/////////////////////////////////////////////////////////////////////////////
template <typename point_type>
    std::ostream& operator<<(std::ostream& os,
                             tml::beziercurve_segment<point_type> const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace tml
