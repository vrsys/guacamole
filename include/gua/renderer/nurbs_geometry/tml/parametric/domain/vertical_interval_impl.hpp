/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : vertical_interval_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
// includes, system
#include <iomanip>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/iterator/transform_iterator.hpp>

// includes, project
#include <gua/renderer/nurbs_geometry/tml/interval.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/uniform_cell.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/determine_splits_from_endpoints.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/previous_next_set.hpp>

namespace tml {

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
vertical_interval<value_type>::vertical_interval(interval_type const& i,
                                                 partition_ptr_type parent)
    : _interval(i),
      _partition(parent),
      _previous(),
      _next(),
      _segments(),
      _cell_set(),
      _area() {}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
vertical_interval<value_type>::vertical_interval(interval_type const& i,
                                                 partition_ptr_type parent,
                                                 shared_ptr_type prec,
                                                 shared_ptr_type succ)
    : _interval(i),
      _partition(parent),
      _previous(prec),
      _next(succ),
      _segments(),
      _cell_set(),
      _area() {}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
vertical_interval<value_type>::~vertical_interval() {}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
void vertical_interval<value_type>::previous(shared_ptr_type p) {
  _previous = p;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
void vertical_interval<value_type>::next(shared_ptr_type s) {
  _next = s;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::shared_ptr_type
vertical_interval<value_type>::previous() const {
  return _previous;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::shared_ptr_type
vertical_interval<value_type>::next() const {
  return _next;
}

template <typename value_type>
interval<value_type> const&
vertical_interval<value_type>::get_vertical_interval() const {
  return _interval;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
interval<value_type>
vertical_interval<value_type>::get_horizontal_interval() const {
  if (_cell_set.empty()) {
    return interval_type(0, 0);
  } else {
    return interval_type(
        (*_cell_set.begin())->get_horizontal_interval().minimum(),
        (*_cell_set.rbegin())->get_horizontal_interval().maximum());
  }
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
void vertical_interval<value_type>::add(
    curve_segment_ptr const& curve_segment) {
  _segments.insert(curve_segment);
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
void vertical_interval<value_type>::add(cell_ptr_type const& cell) {
  _cell_set.insert(cell);
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
std::size_t vertical_interval<value_type>::curve_segments() const {
  return _segments.size();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
void vertical_interval<value_type>::print(std::ostream& os,
                                          std::string const& addinfo) const {
  os << "vertical interval " << _interval
     << " includes cells : " << _cell_set.size() << std::endl;
  std::for_each(_cell_set.begin(),
                _cell_set.end(),
                boost::bind(&cell_type::print, _1, boost::ref(os), "\n"));
  os << "curves : " << std::endl;
  std::for_each(
      _segments.begin(),
      _segments.end(),
      boost::bind(&curve_segment_type::print, _1, boost::ref(os), "\n"));
  os << addinfo;
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
std::size_t vertical_interval<value_type>::size() const {
  return _cell_set.size();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
bool vertical_interval<value_type>::empty() const {
  return _cell_set.empty();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type> void vertical_interval<value_type>::clear() {
  return _cell_set.clear();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::iterator
vertical_interval<value_type>::begin() {
  return _cell_set.begin();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::iterator
vertical_interval<value_type>::end() {
  return _cell_set.end();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::const_iterator
vertical_interval<value_type>::begin() const {
  return _cell_set.begin();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::const_iterator
vertical_interval<value_type>::end() const {
  return _cell_set.end();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::reverse_iterator
vertical_interval<value_type>::rbegin() {
  return _cell_set.rbegin();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::reverse_iterator
vertical_interval<value_type>::rend() {
  return _cell_set.rend();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::const_reverse_iterator
vertical_interval<value_type>::rbegin() const {
  return _cell_set.rbegin();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::const_reverse_iterator
vertical_interval<value_type>::rend() const {
  return _cell_set.rend();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::cell_ptr_type const&
vertical_interval<value_type>::front() const {
  if (_cell_set.empty()) {
    throw std::out_of_range(
        "vertical_interval<value_type>::front (). Container empty.");
  } else {
    return *_cell_set.begin();
  }
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::cell_ptr_type const&
vertical_interval<value_type>::back() const {
  if (_cell_set.empty()) {
    throw std::out_of_range(
        "vertical_interval<value_type>::back (). Container empty.");
  } else {
    return *_cell_set.rbegin();
  }
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::cell_ptr_type
vertical_interval<value_type>::find(value_type const& value) const {
  BOOST_FOREACH(cell_ptr_type const & cell, _cell_set) {
    if (cell->get_horizontal_interval().in(value)) {
      return cell;
    }
  }
  return cell_ptr_type();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::curve_segment_const_iterator
vertical_interval<value_type>::segment_begin() const {
  return _segments.begin();
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
typename vertical_interval<value_type>::curve_segment_const_iterator
vertical_interval<value_type>::segment_end() const {
  return _segments.end();
}

///////////////////////////////////////////////////////////////////////////
/*template <typename value_type>
void
vertical_interval<value_type>::create_horizontal_partition(shared_ptr_type
vertical_interval)
{
  typedef determine_splits_from_endpoints<value_type, point_type::u>
horizontal_splitter;
  horizontal_splitter u_splits = std::for_each(segment_begin(), segment_end(),
horizontal_splitter() );

  if (u_splits.size() > 1)
  {
    std::set<value_type>::const_iterator interval_begin =   u_splits.begin();
    std::set<value_type>::const_iterator interval_end   = ++u_splits.begin();

    while (interval_end != u_splits.end())
    {
      interval_type horizontal_interval   = interval_type(*interval_begin,
*interval_end, tml::excluded, tml::excluded);
      _cell_set[horizontal_interval]      = cell_ptr_type(new
cell_type(vertical_interval, horizontal_interval, _interval));

      ++interval_begin; ++interval_end;
    }

    std::for_each(boost::make_transform_iterator(_cell_set.begin(),
util::second_adaptor<std::pair<interval_type, cell_ptr_type> >),
                  boost::make_transform_iterator(  _cell_set.end(),
util::second_adaptor<std::pair<interval_type, cell_ptr_type> >),
                  previous_next_set<cell_ptr_type>());

  } else {
    // to implement : no partition in horizontal direction
  }
}


///////////////////////////////////////////////////////////////////////////
template <typename value_type>
void
vertical_interval<value_type>::precompute_intersections ( value_type distance )
{
  BOOST_FOREACH(cell_ptr_pair const& cell_pair, _cell_set)
  {
    // get cell's horizontal extension and the pointer to the cell
    interval_type   horizontal_interval  = cell_pair.first;
    cell_ptr_type   cell                 = cell_pair.second;

    BOOST_FOREACH(curve_segment_ptr_type segment, _segments)
    {
      interval_type curve_horizontal_interval =
interval_type(segment->minimum(point_type::u), segment->maximum(point_type::u));
      interval_type curve_vertical_interval   =
interval_type(segment->minimum(point_type::v), segment->maximum(point_type::v));

      curve_horizontal_interval.extend(distance);

      if (segment->is_constant(point_type::v))
      {
        // horizontal curves should be considered for anti-aliasing only
      }
      else
      {
        // 1. definite intersection : vertical curves in positive u-direction
(>sample_size) suggest an implicitly known intersection
        if ( segment->minimum(point_type::u) - distance >=
horizontal_interval.maximum() &&
             _interval.overlap(curve_vertical_interval) )
        {
          cell->intersections( cell->intersections() + 1 ); // increase number
of implicitly known intersections
        }
        else
        {
          // 2. possible intersection : curve's horizontal range and extended
horizontal range overlap
          if ( horizontal_interval.overlap(curve_horizontal_interval) )
          {
            cell->add(segment);
          }

          // 3. no intersection : curve is in negative u-direction : distance <
-sample_size
          if ( segment->maximum(point_type::u) <= horizontal_interval.minimum()
)
          {
            // do nothing
          }
        }
      }
    }
  }
}
*/

///////////////////////////////////////////////////////////////////////////////
template <typename value_type>
void vertical_interval<value_type>::update_vicinity() const {
  std::for_each(
      _cell_set.begin(), _cell_set.end(), previous_next_set<cell_ptr_type>());
}

///////////////////////////////////////////////////////////////////////////
template <typename value_type>
    std::ostream& operator<<(std::ostream& os,
                             tml::vertical_interval<value_type> const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace tml
