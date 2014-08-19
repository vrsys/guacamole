/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : determine_splits_from_endpoints.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_DETERMINE_SPLITS_FROM_ENDPOINTS
#define TML_DETERMINE_SPLITS_FROM_ENDPOINTS

// includes, system

namespace tml {

/////////////////////////////////////////////////////////////////////////////
template <typename value_t, std::size_t AXIS>
struct determine_splits_from_endpoints {
  /////////////////////////////////////////////////////////////////////////////
  determine_splits_from_endpoints(value_t offset = 0,
                                  bool insert_value_itself = true)
      : _partition(), _offset(offset), _value_itself(insert_value_itself) {}

  /////////////////////////////////////////////////////////////////////////////
  template <typename curve_ptr_type> void operator()(curve_ptr_type bc) {
    // insert value itself
    if (_value_itself) {
      _partition.insert(bc->front()[AXIS]);
      _partition.insert(bc->back()[AXIS]);
    }

    // insert boundaries
    if (bc->is_increasing(AXIS)) {
      _partition.insert(bc->front()[AXIS] - _offset);
      _partition.insert(bc->back()[AXIS] + _offset);
    } else {
      _partition.insert(bc->front()[AXIS] + _offset);
      _partition.insert(bc->back()[AXIS] - _offset);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  std::size_t size() const { return _partition.size(); }

  /////////////////////////////////////////////////////////////////////////////
  bool empty() const { return _partition.empty(); }

  /////////////////////////////////////////////////////////////////////////////
  typename std::set<value_t>::const_iterator begin() const {
    return _partition.begin();
  }

  /////////////////////////////////////////////////////////////////////////////
  typename std::set<value_t>::const_iterator end() const {
    return _partition.end();
  }

 private:

  std::set<value_t> _partition;
  value_t _offset;
  bool _value_itself;
};

}  // namespace tml

#endif  // TML_GENERATE_INTERVALS_FROM_ENDPOINTS_HPP
