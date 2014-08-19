/********************************************************************************
*
* Copyright (C) 2013 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : status_structure.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_STATUS_STRUCTURE_HPP
#define TML_STATUS_STRUCTURE_HPP

#include <gua/renderer/nurbs_geometry/tml/parametric/domain/line.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/face.hpp>

namespace tml {

template <typename value_t> class status_structure {
 public:

  typedef value_t value_type;
  typedef line<value_type> line_type;
  typedef face<value_type> face_type;

 public:

  status_structure() : _open_faces(), _faces() {}

  virtual ~status_structure() {}

  // add line into status queue
  void add(event_structure<value_type>& Q, value_type const& x, line_type* l) {
    /*
    // determine insert position in binary tree
    std::size_t pos = find_insert_position ( x, l );

    // insert line and store index in additional index map
    queue_.insert ( queue_.begin() + pos, l );
    line_indices_.insert(std::make_pair(l, pos));

    std::cout << "add : " << std::endl;
    print(std::cout);

    // update index map
    for ( std::size_t k = pos+1; k != queue_.size(); ++k )
    {
      line_indices_.find(queue_[k])->second++;
    }

    intersection_event<point_t>* t0 = 0;
    intersection_event<point_t>* t1 = 0;

    // check new left neighbor
    if ( pos > 0 )
    {
      t0 = check ( x, queue_[pos-1], queue_[pos] );
    }

    // check new right neighbor
    if ( pos+1 < queue_.size() )
    {
      t1 = check ( x, queue_[pos], queue_[pos+1] );
    }

    process_intersections(x, Q, t0, t1);
    */
  }

  // remove line from status queue
  void remove(event_structure<value_type>& Q,
              value_type const& x,
              line_type* l) {
    /*
    if ( queue_.empty() ) return;

    // find and delete line from queue
    std::size_t pos = find_line ( x, l );
    queue_.erase ( queue_.begin() + pos );
    line_indices_.erase(line_indices_.find(l));

    std::cout << "remove : " << std::endl;
    print(std::cout);

    // update index map
    for ( std::size_t k = pos; k != queue_.size(); ++k )
    {
      line_indices_.find(queue_[k])->second--;
    }

    // no intersections for removals at outer positions
    if ( pos == 0 || pos == queue_.size() ) return;

    intersection_event<point_t>* t0 = 0;
    intersection_event<point_t>* t1 = 0;

    // check new left neighbor
    if ( pos > 0 && pos < queue_.size() )
    {
      t0 = check ( x, queue_[pos-1], queue_[pos] );
    }

    // check new right neighbor
    if ( pos+1 < queue_.size() )
    {
      t1 = check ( x, queue_[pos], queue_[pos+1] );
    }

    process_intersections ( x, Q, t0, t1 );
    */
  }

  // print current status queue
  void print(std::ostream& os) {
    os << "Status Queue : " << std::endl;
    for (auto i = _queue.begin(); i != _queue.end(); ++i) {
      (**i).print(os);
    }
    os << std::endl;
  }

 private:

  std::vector<face_type*> _open_faces;
  std::vector<face_type*> _faces;

};

}  // namespace tml

#endif  // TML_STATUS_STRUCTURE_HPP
