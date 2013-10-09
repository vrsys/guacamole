/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : partition_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
// includes, system
#include <list>

#include <boost/bind.hpp>

// includes, project
#include <gua/renderer/nurbs_geometry/tml/compare.hpp>
#include <gua/renderer/nurbs_geometry/tml/util/erase_if.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/vertical_interval.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/uniform_cell.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/determine_dimension_from_endpoints.hpp>

namespace tml {

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
partition<point_type>::partition()
    : _monotonic_curves(),
      _horizontal_interval(0, 0),
      _vertical_interval(0, 0),
      _vertical_partition() {}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
template <typename curve_ptr_iterator_type>
partition<point_type>::partition(curve_ptr_iterator_type begin,
                                 curve_ptr_iterator_type end)
    : _monotonic_curves(),
      _horizontal_interval(0, 0),
      _vertical_interval(0, 0),
      _vertical_partition() {
  set(begin, end);
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type> partition<point_type>::~partition() {}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
template <typename curve_ptr_iterator_type>
void partition<point_type>::set(curve_ptr_iterator_type begin,
                                curve_ptr_iterator_type end) {
  _monotonic_curves.clear();
  _vertical_partition.clear();

  if (std::distance(begin, end) > 0) {
    // split curves at extrema and align into positive vertical direction
    _monotonize(begin, end);

    update_size_from_curves();

  } else {
    _horizontal_interval = interval_type(0, 0);
    _vertical_interval = interval_type(0, 0);
    //std::cerr << "partition<point_type>::partition(): Trying to create a
    //partition without any boundary curves\n";
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type> void partition<point_type>::initialize() {
  if (!_monotonic_curves.empty()) {
    // 1. step : switch curves that they point into positive vertical direction
    _align_vertically_increasing();

    // 2. step : create a basic vertical partition as suggested by the curve's
    // end points
    initial_vertical_partition();

    // 3. step : create curve segments for each vertical interval so that no
    // segment overlaps more than one interval and add them to intervals
    _create_curve_segments();

    // 4. step : partition each vertical interval into cells
    BOOST_FOREACH(vertical_interval_ptr const & v, _vertical_partition) {
      initial_horizontal_partition(
          v);  // no bind possible since virtual function called on dynamic type
    }

    // 5. step : precompute known intersections with curve segments
    std::for_each(
        _vertical_partition.begin(),
        _vertical_partition.end(),
        boost::bind(
            &partition<point_type>::_precompute_known_intersections, this, _1));

    // 6. step : precompute possibly intersected intersections with curve
    // segments for cells
    BOOST_FOREACH(vertical_interval_ptr const & v, _vertical_partition) {
      precompute_curves_to_intersect(
          v);  // no bind possible since virtual function called on dynamic type
    }

    update_size_from_intervals();
  }

  //print(std::cout);
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type> std::size_t partition<point_type>::size() const {
  return _vertical_partition.size();
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
std::size_t partition<point_type>::curves() const {
  return _monotonic_curves.size();
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type> bool partition<point_type>::empty() const {
  return _vertical_partition.empty();
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
void partition<point_type>::split(vertical_interval_ptr const& to_split,
                                  value_type const& split_value) {
  // if vertical interval is part of partition and passed parameter is in that
  // interval -> split into two intervals and erase the old interval
  if (_vertical_partition.find(to_split) != _vertical_partition.end() &&
      to_split->get_vertical_interval().in(split_value)) {
    // create mathematical intervals
    interval_type top_interval(split_value,
                               to_split->get_vertical_interval().maximum(),
                               tml::excluded,
                               tml::excluded);
    interval_type bottom_interval(to_split->get_vertical_interval().minimum(),
                                  split_value,
                                  tml::excluded,
                                  tml::excluded);

    // vertical_interval objects for top and bottom interval
    vertical_interval_ptr top(new vertical_interval_type(top_interval, this));
    vertical_interval_ptr bottom(
        new vertical_interval_type(bottom_interval, this));

    // connect to neighbor intervals
    top->next(to_split->next());
    top->previous(bottom);
    bottom->next(top);
    bottom->previous(to_split->previous());

    if (to_split->next()) {
      to_split->next()->previous(top);
    }

    if (to_split->previous()) {
      to_split->previous()->next(bottom);
    }

    // split all curves
    for (typename segment_set::const_iterator i = to_split->segment_begin();
         i != to_split->segment_end();
         ++i) {
      // deal with horizontal curve segments
      if ((*i)->is_constant(point_type::v)) {
        // if horizontal curve segment is on upper limit
        if ((*i)->front()[point_type::v] ==
            to_split->get_vertical_interval().maximum()) {
          top->add(*i);  // push it into top vertical interval
        } else {
          bottom->add(*i);  // push segment into bottom interval
        }
      } else {  // curve segment has to be split -> intersect with horizontal
                // ray
                // at v=split_value

        bool intersects;
        value_type intersection_parameter;
        point_type intersection_point;

        (*i)->curve()->bisect(
            point_type::v, split_value, intersects, intersection_parameter);

        // determine split point for vertical lines analytically
        if ((*i)->is_constant(point_type::u)) {
          intersection_point = point_type(
              (*i)->minimum(point_type::u),  // use u-parameter of minimum since
              // u is uniform on curve
              split_value);                  // use v-parameter of split
        } else {
          // determine intersection using t-parameter of intersection
          intersection_point = (*i)->curve()->evaluate(intersection_parameter);
          intersection_point[point_type::v] = split_value;
        }

        top->add(
            curve_segment_ptr(new curve_segment_type((*i)->curve(),
                                                     intersection_parameter,
                                                     (*i)->tmax(),
                                                     intersection_point,
                                                     (*i)->back())));
        bottom->add(
            curve_segment_ptr(new curve_segment_type((*i)->curve(),
                                                     (*i)->tmin(),
                                                     intersection_parameter,
                                                     (*i)->front(),
                                                     intersection_point)));
      }
    }

    // remove old interval and insert parts of split
    _vertical_partition.erase(to_split);
    _vertical_partition.insert(top);
    _vertical_partition.insert(bottom);

  } else {
    throw std::runtime_error("partition<point_type>::split(): No such interval "
                             "or split parameter not in interval.");
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename partition<point_type>::iterator partition<point_type>::begin() {
  return _vertical_partition.begin();
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename partition<point_type>::iterator partition<point_type>::end() {
  return _vertical_partition.end();
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename partition<point_type>::const_iterator
partition<point_type>::begin() const {
  return _vertical_partition.begin();
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename partition<point_type>::const_iterator
partition<point_type>::end() const {
  return _vertical_partition.end();
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename partition<point_type>::reverse_iterator
partition<point_type>::rbegin() {
  return _vertical_partition.rbegin();
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename partition<point_type>::reverse_iterator partition<point_type>::rend() {
  return _vertical_partition.rend();
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename partition<point_type>::const_reverse_iterator
partition<point_type>::rbegin() const {
  return _vertical_partition.rbegin();
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename partition<point_type>::const_reverse_iterator
partition<point_type>::rend() const {
  return _vertical_partition.rend();
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename partition<point_type>::interval_type const&
partition<point_type>::get_horizontal_interval() const {
  return _horizontal_interval;
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename partition<point_type>::interval_type const&
partition<point_type>::get_vertical_interval() const {
  return _vertical_interval;
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
void partition<point_type>::update_vicinity() const {
  std::for_each(_vertical_partition.begin(),
                _vertical_partition.end(),
                previous_next_set<vertical_interval_ptr>());
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
void partition<point_type>::update_size_from_curves() {
  if (!_monotonic_curves.empty()) {
    // iterate all curves and use end points to compute limits in all parameter
    // dimensions
    determine_dimension_from_endpoints<curve_ptr_type> extends =
        std::for_each(_monotonic_curves.begin(),
                      _monotonic_curves.end(),
                      determine_dimension_from_endpoints<curve_ptr_type>());

    _horizontal_interval = extends.max_interval[point_type::u];
    _vertical_interval = extends.max_interval[point_type::v];

  } else {
    std::cerr << "partition<point_type>::_compute_bounds (): Cannot compute "
                 "bounds, no curves in partition." << std::endl;
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
void partition<point_type>::update_size_from_intervals() {
  if (_vertical_partition.empty()) {
    // throw exception?
    _horizontal_interval = _vertical_interval = interval_type(0, 0);
  } else {
    _horizontal_interval =
        (*_vertical_partition.begin())->get_horizontal_interval();
    _vertical_interval =
        (*_vertical_partition.begin())->get_vertical_interval();

    BOOST_FOREACH(vertical_interval_ptr const & v, _vertical_partition) {
      _vertical_interval.merge(v->get_vertical_interval());
      _horizontal_interval.merge(v->get_horizontal_interval());
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
typename partition<point_type>::cell_ptr_type partition<point_type>::find(
    point_type const& p) const {
  BOOST_FOREACH(vertical_interval_ptr const & v, _vertical_partition) {
    if (v->get_vertical_interval().distance(p[point_type::v]) == 0) {
      BOOST_FOREACH(cell_ptr_type const & cell, *v) {
        if (cell->get_horizontal_interval().distance(p[point_type::u]) == 0) {
          return cell;
        }
      }
    }
  }
  return cell_ptr_type();
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
void partition<point_type>::print(std::ostream& os) const {
  os << "partition:\n";
  os << "monotonic curves in partition : " << std::endl;

  std::for_each(_monotonic_curves.begin(),
                _monotonic_curves.end(),
                boost::bind(&curve_type::print, _1, boost::ref(os), "\n"));

  os << "vertical intervals in partition : " << std::endl;
  for (typename vertical_interval_set::const_iterator m =
           _vertical_partition.begin();
       m != _vertical_partition.end();
       ++m) {
    os << **m << std::endl;
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
/* virtual */ void partition<point_type>::initial_horizontal_partition(
    vertical_interval_ptr const& v) {
  typedef determine_splits_from_endpoints<value_type, point_type::u>
      horizontal_splitter;

  // gather horizontal coordinates of all segment's end points in vertical
  // interval
  horizontal_splitter u_splits = std::for_each(
      v->segment_begin(), v->segment_end(), horizontal_splitter());

  if (u_splits.size() > 1) {
    typename std::set<value_type>::const_iterator interval_begin =
        u_splits.begin();
    typename std::set<value_type>::const_iterator interval_end =
        ++u_splits.begin();

    // for each interval in horizontal direction create a cell with uniform
    // properties
    while (interval_end != u_splits.end()) {
      // mathematical interval range excluding its limits
      interval_type horizontal_interval = interval_type(
          *interval_begin, *interval_end, tml::excluded, tml::excluded);

      // insert cell into vertical interval
      v->add(cell_ptr_type(
          new cell_type(v, horizontal_interval, v->get_vertical_interval())));

      ++interval_begin;
      ++interval_end;
    }

    // connect cells to a double linked list
    update_vicinity();

  } else {
    // to implement : no partition in horizontal direction
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
/* virtual */ void partition<point_type>::precompute_curves_to_intersect(
    vertical_interval_ptr const& p) {
  for (typename cell_ptr_set::iterator cell = p->begin(); cell != p->end();
       ++cell) {
    // get cell's horizontal extension and the pointer to the cell
    interval_type horizontal_interval = (*cell)->get_horizontal_interval();

    for (typename segment_set::const_iterator segment = p->segment_begin();
         segment != p->segment_end();
         ++segment) {
      interval_type curve_horizontal_interval =
          interval_type((*segment)->minimum(point_type::u),
                        (*segment)->maximum(point_type::u));
      interval_type curve_vertical_interval =
          interval_type((*segment)->minimum(point_type::v),
                        (*segment)->maximum(point_type::v));

      if ((*segment)->is_constant(point_type::v)) {
        // horizontal curves should be considered for anti-aliasing only
      } else {
        // 1. definite intersection : vertical curves in positive u-direction
        // (>sample_size) suggest an implicitly known intersection
        if ((*segment)->minimum(point_type::u) >=
                horizontal_interval.maximum() &&
            p->get_vertical_interval().overlap(curve_vertical_interval)) {
          // implicitly known intersection -> should already be added
        } else {
          // 2. possible intersection : curve's horizontal range and extended
          // horizontal range overlap
          if (horizontal_interval.overlap(curve_horizontal_interval)) {
            (*cell)->add(*segment);
          }
        }
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
/* virtual */ void partition<point_type>::initial_vertical_partition() {
  // get split values as suggested by the curves end points
  typedef determine_splits_from_endpoints<value_type, point_type::v>
      vertical_splitter;
  vertical_splitter vpartition = std::for_each(
      _monotonic_curves.begin(), _monotonic_curves.end(), vertical_splitter());

  // if there a partition is possible
  if (vpartition.size() > 1) {
    typename std::set<value_type>::const_iterator interval_begin =
        vpartition.begin();
    typename std::set<value_type>::const_iterator interval_end =
        ++vpartition.begin();

    while (interval_end != vpartition.end()) {
      // create an interval with borders excluded and a new vertical interval
      // and insert both into map
      interval_type v_interval(
          *interval_begin, *interval_end, tml::excluded, tml::excluded);
      _vertical_partition.insert(
          vertical_interval_ptr(new vertical_interval_type(v_interval, this)));

      ++interval_begin;
      ++interval_end;
    }

    // set previous and next interval after all intervals have been created
    std::for_each(_vertical_partition.begin(),
                  _vertical_partition.end(),
                  previous_next_set<vertical_interval_ptr>());

  } else {
    std::cerr
        << "partition<point_type>::_initial_vertical_partition () : Cannot "
           "generate partition with less than one interval in v-direction.\n";
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
template <typename curve_ptr_iterator_type>
void partition<point_type>::_monotonize(curve_ptr_iterator_type begin,
                                        curve_ptr_iterator_type end) {
  std::list<curve_type> monotonized_curves;

  // iterate all input curves
  curve_ptr_iterator_type i = begin;
  while (i != end) {
    std::set<value_type> extremas;

    // use dt-operator to assert local extrema -> use ten times the epsilon of
    // value type
    value_type const eps = 10 * std::numeric_limits<value_type>::epsilon();

    // find extrema in both parameter directions and insert monotonic segments
    // into container
    (*i)->extrema(point_type::u,
                  extremas,
                  partition<point_type>::MAX_ROOTFIND_ITERATIONS);
    tml::util::erase_if(
        extremas,
        boost::bind(&curve_type::assert_extremum, *i, point_type::u, _1, eps));

    (*i)->extrema(point_type::v,
                  extremas,
                  partition<point_type>::MAX_ROOTFIND_ITERATIONS);
    tml::util::erase_if(
        extremas,
        boost::bind(&curve_type::assert_extremum, *i, point_type::v, _1, eps));

    // split at local extremas
    if (extremas.empty()) {
      monotonized_curves.push_back(**i);
    } else {
      (*i)->split(extremas, monotonized_curves);
    }

    ++i;
  }

  // insert monotonized curves as shared_ptr as from now on they are referenced
  // by beziercurve_segments
  for (typename std::list<curve_type>::const_iterator i =
           monotonized_curves.begin();
       i != monotonized_curves.end();
       ++i) {
    _monotonic_curves.insert(boost::shared_ptr<curve_type>(new curve_type(*i)));
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
void partition<point_type>::_align_vertically_increasing() {
  for (const_curve_iterator c = _monotonic_curves.begin();
       c != _monotonic_curves.end();
       ++c) {
    if (!(*c)->is_increasing(point_type::v)) {
      (*c)->reverse();
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
void partition<point_type>::_create_curve_segments() {
  BOOST_FOREACH(curve_ptr_type const & c, _monotonic_curves) {
    // gather curve information
    interval_type curve_interval = interval_type(c->front()[point_type::v],
                                                 c->back()[point_type::v],
                                                 tml::excluded,
                                                 tml::excluded);
    bool curve_horizontally_linear = c->is_constant(point_type::v);

    // segment only curves that might have an intersection in vertical direction
    if (!curve_horizontally_linear) {
      BOOST_FOREACH(vertical_interval_ptr const & m, _vertical_partition) {
        // if intervals of curve and vertical partition overlap -> split curve
        // and apply according segment to vertical partition
        if (curve_interval.overlap(m->get_vertical_interval())) {
          // intersect curve and ray in horizontal direction to determine
          // segment
          bool intersects_at_min, intersects_at_max;
          value_type tmin, tmax;
          point_type pmin, pmax;

          // determine intersection points
          if (c->degree() > 1) {
            c->bisect(point_type::v,
                      m->get_vertical_interval().minimum(),
                      intersects_at_min,
                      tmin);
            c->bisect(point_type::v,
                      m->get_vertical_interval().maximum(),
                      intersects_at_max,
                      tmax);
          } else {
            intersects_at_min = c->front()[point_type::v] ==
                                m->get_vertical_interval().minimum();
            intersects_at_max = c->back()[point_type::v] ==
                                m->get_vertical_interval().maximum();
            tmin = (m->get_vertical_interval().minimum() -
                    c->front()[point_type::v]) /
                   (c->back()[point_type::v] - c->front()[point_type::v]);
            tmax = (m->get_vertical_interval().maximum() -
                    c->front()[point_type::v]) /
                   (c->back()[point_type::v] - c->front()[point_type::v]);
          }

          // create segment for vertical curve segment analytically
          if (c->is_constant(point_type::u)) {
            pmin = point_type(c->front()[point_type::u],
                              m->get_vertical_interval().minimum());
            pmax = point_type(c->front()[point_type::u],
                              m->get_vertical_interval().maximum());
          } else {
            // evaluate intersection points and make sure the vertical end
            // points equal the interval
            pmin = c->evaluate(tmin);
            pmax = c->evaluate(tmax);
            pmin[point_type::v] = m->get_vertical_interval().minimum();
            pmax[point_type::v] = m->get_vertical_interval().maximum();
          }

          // create curve segment
          curve_segment_ptr segment(new beziercurve_segment<point_type>);
          segment->apply(c, tmin, tmax, pmin, pmax);

          // add curve segment pointer to partition and vertical interval
          m->add(segment);
        }
      }
    } else {  // for horizontal curves create only one segment

      curve_segment_ptr horizontal_segment(new beziercurve_segment<point_type>);
      horizontal_segment->apply(c, 0, 1, c->front(), c->back());

      // insert horizontal curve to vertical intervals with same vertical
      // boundary
      BOOST_FOREACH(vertical_interval_ptr const & m, _vertical_partition) {
        if (curve_interval.minimum() == m->get_vertical_interval().minimum() ||
            curve_interval.minimum() == m->get_vertical_interval().maximum()) {
          m->add(horizontal_segment);
        }
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
void partition<point_type>::_precompute_known_intersections(
    vertical_interval_ptr const& p) {
  for (typename cell_ptr_set::iterator cell = p->begin(); cell != p->end();
       ++cell) {
    // get cell's horizontal extension and the pointer to the cell
    interval_type horizontal_interval = (*cell)->get_horizontal_interval();

    for (typename segment_set::const_iterator segment = p->segment_begin();
         segment != p->segment_end();
         ++segment) {
      interval_type curve_horizontal_interval =
          interval_type((*segment)->minimum(point_type::u),
                        (*segment)->maximum(point_type::u));
      interval_type curve_vertical_interval =
          interval_type((*segment)->minimum(point_type::v),
                        (*segment)->maximum(point_type::v));

      if (!(*segment)
              ->is_constant(point_type::v))  // horizontal curves should be
          // considered for anti-aliasing only
          {
        // definite intersection : vertical curves in positive u-direction
        // suggest an implicitly known intersection
        if ((*segment)->minimum(point_type::u) >=
                horizontal_interval
                    .maximum() &&          // curve segment is in positive
            // horizontal direction
            p->get_vertical_interval().overlap(
                curve_vertical_interval))  // curve and interval overlap in
            // vertical direction
            {
          (*cell)->intersections(
              (*cell)->intersections() +
              1);  // increase number of implicitly known intersections
        } else {
          // possible or no intersection, do not handle here
        }
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename point_type>
    std::ostream& operator<<(std::ostream& os,
                             partition<point_type> const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace tml
