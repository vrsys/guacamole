/********************************************************************************
*
* Copyright (C) 2013 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : event_structure.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_EVENT_STRUCTURE_HPP
#define TML_EVENT_STRUCTURE_HPP

#include <gua/renderer/nurbs_geometry/tml/parametric/domain/contour.hpp>

namespace tml {

/*
template <typename value_t>
struct sort_events
{
public :

  typedef point<value_t,2> point_type;

  bool operator()( status_event<value_t>* lhs, status_event<value_t>* rhs )
const
  {
    // 1st : x coordinate
    if ( lhs->interval().minimum() < rhs->interval().minimum()[point_type::u] )
    {
      return true;
    } else if ( lhs->interval().start()[point_type::u] >
rhs->interval().start()[point_type::u] ) {
      return false;
    } else { // else compare priority
      return lhs->priority() < rhs->priority();
    }
  }
};
*/

template <typename value_t> class event_structure {
 public:

  typedef value_t value_type;
  typedef point<value_type, 2> point_type;
  typedef status_event<value_type> event_t;

 public:

  event_structure() : _queue() {}

  virtual ~event_structure() {}

  bool empty() const { return _queue.empty(); }

  // insert start and end event for a line
  void add(typename contour<value_type>::contour_segment_ptr const& c) {
    /*
    if ( l->start().x < l->end().x ) // start.x < end.x
    {
      queue_.insert ( new start_event<point_t> ( l->start(), l ) );
      queue_.insert ( new end_event<point_t>   ( l->end(), l ) );
    } else if ( l->start().x > l->end().x ) // start.x > end.x
    {
      queue_.insert ( new end_event<point_t>   ( l->start(), l ) );
      queue_.insert ( new start_event<point_t> ( l->end(), l ) );
    } else if ( l->start().y < l->end().y )  // start.x == end.x -> use y
    coordinate
    {
      queue_.insert ( new start_event<point_t> ( l->start(), l ) );
      queue_.insert ( new end_event<point_t>   ( l->end(), l ) );
    } else {
      queue_.insert ( new end_event<point_t>   ( l->start(), l ) );
      queue_.insert ( new start_event<point_t> ( l->end(), l ) );
    }
    */
  }

  // remove event from queue -> queue responsible for deletion
  void remove(event_t* e) {
    auto i = _queue.lower_bound(e);
    if (i != _queue.end()) {
      delete* i;
      _queue.erase(i);
    }
  }

  // get top most event
  event_t* get_max() const { return *_queue.begin(); }

  void print(std::ostream& os) const {
    os << "Event Queue : " << std::endl;
    //std::for_each( queue_.begin(), queue_.end(), boost::bind (
    //&status_event<point_t>::print, _1, boost::ref(os)));
    for (auto i = queue_.begin(); i != queue_.end(); ++i) {
      (**i).print(os);
    }
  }

 private:

  //std::multiset<event_t*, sort_events<value_type> > _queue;
  std::multiset<event_t*> _queue;

};

}  // namespace tml

#endif  // TML_EVENT_STRUCTURE_HPP
