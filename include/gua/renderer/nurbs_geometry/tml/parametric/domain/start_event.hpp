/********************************************************************************
*
* Copyright (C) 2013 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : start_event.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_START_EVENT_HPP
#define TML_START_EVENT_HPP

#include <gua/renderer/nurbs_geometry/tml/parametric/domain/contour.hpp>
#include <gua/renderer/nurbs_geometry/tml/interval.hpp>

namespace tml {

template <typename value_t> class start_event : public status_event<value_t> {
 public:

  typedef value_t value_type;
  typedef interval<value_type> interval_type;
  typedef typename contour<value_type>::contour_segment_ptr contour_segment_ptr;

 public:

  start_event(value_type const& u,
              interval_type const& interval_v,
              contour_segment_ptr const& c)
      : status_event(u, interval_v), _segment(c) {}

  void update(status_structure<point_t>& L, event_structure<point_t>& Q) {
    L.add(Q, location().x, line_);
  }

  virtual void print(std::ostream& os) const {
    os << "start event : ";
    status_event<point_t>::print(os);
  }

  virtual unsigned priority() const { return 2; }

  ~start_event() {}

 private:

  contour_segment_ptr _segment;

};

}  // namespace tml

#endif  //TML_START_EVENT_HPP
