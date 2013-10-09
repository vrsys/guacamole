/********************************************************************************
*
* Copyright (C) 2013 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : end_event.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_END_EVENT_HPP
#define TML_END_EVENT_HPP

#include <gua/renderer/nurbs_geometry/tml/parametric/domain/status_event.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/contour.hpp>
#include <gua/renderer/nurbs_geometry/tml/interval.hpp>

namespace tml {

template <typename value_t> class end_event : public status_event<value_t> {
 public:

  typedef value_t value_type;
  typedef interval<value_type> interval_type;
  using typename contour<value_type>::contour_segment_ptr;

 public:

  end_event(value_type const& u,
            interval_type const& interval_v,
            contour_segment_ptr const& s)
      : _vertical_interval(u, interval_v), _segment(s) {}

  void update(status_structure<point_t>& L, event_structure<point_t>& Q) {
    L.remove(Q, location().x, line_);
  }

  virtual void print(std::ostream& os) const {
    os << "end event : ";
    status_event<point_t>::print(os);
  }

  virtual unsigned priority() const { return 2; }

  ~end_event() {}

 private:

  contour_segment_ptr _segment;

};

}  // namespace tml

#endif  //TML_END_EVENT_HPP
