/********************************************************************************
*
* Copyright (C) 2013 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : contour_segment_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/

namespace tml {

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
template <typename curve_ptr_iterator_t>
contour_segment<value_t>::contour_segment(curve_ptr_iterator_t begin,
                                          curve_ptr_iterator_t end)
    : _curves(begin, end), _bbox(), _monotony(unclassified), _continous(false) {
  _determine_monotony();
  _update_bbox();
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t> contour_segment<value_t>::~contour_segment() {}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
monotony_t contour_segment<value_t>::monotony() const {
  return _monotony;
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
typename contour_segment<value_t>::bbox_type const&
contour_segment<value_t>::bbox() const {
  return _bbox;
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t> bool contour_segment<value_t>::continous() const {
  if (_curves.empty()) {
    return false;
  } else {
    if (_curves.size() > 1) {
      auto fst = _curves.begin();
      auto snd = _curves.begin();
      std::advance(snd, 1);
      while (snd != _curves.end()) {
        if ((**fst).front() != (**snd).back() &&
            (**fst).back() != (**snd).front()) {
          return false;
        }
      }
      return true;
    } else {
      return true;
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
bool contour_segment<value_t>::increasing(
    typename point_type::coordinate_type const& c) const {
  BOOST_FOREACH(auto const & curve, _curves) {
    if (!curve->is_increasing(c) && !curve->is_constant(c)) {
      return false;
    }
  }
  return true;
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
bool contour_segment<value_t>::is_constant(
    typename point_type::coordinate_type const& c) const {
  for (auto curve = _curves.begin(); curve != _curves.end(); ++curve) {
    if (!(**curve).is_constant(c)) {
      return false;
    }
  }
  return true;
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t> std::size_t contour_segment<value_t>::size() const {
  return _curves.size();
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
typename contour_segment<value_t>::const_curve_iterator
contour_segment<value_t>::begin() const {
  return _curves.begin();
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
typename contour_segment<value_t>::const_curve_iterator
contour_segment<value_t>::end() const {
  return _curves.end();
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t> void contour_segment<value_t>::clip_horizontal() {
  curve_iterator c = _curves.begin();
  while (c != _curves.end()) {
    if ((*c)->is_constant(point_type::v)) {
      c = _curves.erase(c);
    } else {
      ++c;
    }
  }

  _update_bbox();
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t> void contour_segment<value_t>::invert() {
  std::reverse(_curves.begin(), _curves.end());
  std::for_each(_curves.begin(),
                _curves.end(),
                std::bind(&curve_type::invert, std::placeholders::_1));
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
void contour_segment<value_t>::print(std::ostream& os) const {
  os << "contour segment consisting of " << _curves.size()
     << " curves : " << std::endl;
  for (auto c = _curves.begin(); c != _curves.end(); ++c) {
    (**c).print(os);
  }
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
void contour_segment<value_t>::_determine_monotony() {}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t> void contour_segment<value_t>::_update_bbox() {
  if (_curves.empty())
    return;

  _curves.front()->bbox_simple(_bbox);

  for (auto c = _curves.begin(); c != _curves.end(); ++c) {
    tml::axis_aligned_boundingbox<point_type> bbox;
    (**c).bbox_simple(bbox);
    _bbox.merge(bbox);
  }
}

/////////////////////////////////////////////////////////////////////////////
template <typename value_t>
    std::ostream& operator<<(std::ostream& os,
                             tml::contour_segment<value_t> const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace tml
