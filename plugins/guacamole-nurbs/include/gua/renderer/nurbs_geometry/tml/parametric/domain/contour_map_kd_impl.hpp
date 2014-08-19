/********************************************************************************
*
* Copyright (C) 2013 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : contour_map_kd_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
// includes, system

// includes, project

namespace tml {

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
template <typename value_t>
void contour_map_kd<value_t>::contour_cell::print(std::ostream& os) const {
  os << "contour cell: v: " << interval_v << ", u: " << interval_u
     << ", contours: " << overlapping_segments.size() << " inside: " << inside
     << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_t>
void contour_map_kd<value_t>::status_queue::process(
    domain_start_event const& e) {
  interval_v = e.interval_v;

  splits.insert(e.interval_v.minimum());
  splits.insert(e.interval_v.maximum());

  intervals.clear();
  intervals.insert(std::make_pair(e.interval_v, e.value_u));
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_t>
void contour_map_kd<value_t>::status_queue::process(domain_end_event const& e) {
  assert(splits.size() == 2);
  assert(intervals.size() == 1);

  splits.erase(e.interval_v.minimum());
  splits.erase(e.interval_v.maximum());

  intervals.clear();
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_t>
void contour_map_kd<value_t>::status_queue::process(
    contour_start_event const& e) {
  segments.insert(e.segment);

  splits.insert(e.segment->bbox().min[point_type::v]);
  splits.insert(e.segment->bbox().max[point_type::v]);

  update(e.value_u);
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_t>
void contour_map_kd<value_t>::status_queue::process(
    contour_end_event const& e) {
  segments.erase(e.segment);

  splits.erase(e.segment->bbox().min[point_type::v]);
  splits.erase(e.segment->bbox().max[point_type::v]);

  // make sure bounds are kept
  splits.insert(interval_v.minimum());
  splits.insert(interval_v.maximum());

  update(e.value_u);
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_t>
void contour_map_kd<value_t>::status_queue::update(value_type const& u) {
  std::set<interval_type> updated_intervals;
  std::vector<interval_type> intervals_to_remove;
  std::vector<interval_type> intervals_to_add;

  _intervals_from_splits(splits, updated_intervals);

  // 1. identify new intervals
  BOOST_FOREACH(interval_type const & i, updated_intervals) {
    bool found = false;
    BOOST_FOREACH(auto const & j, intervals) { found |= (j.first == i); }

    if (!found) {
      intervals_to_add.push_back(i);
    }
  }

  // 2. close cells that do not match the new intervals
  BOOST_FOREACH(auto const & i, intervals) {
    bool found = false;
    BOOST_FOREACH(auto const & j, updated_intervals) {
      found |= (j == i.first);
    }

    if (!found) {
      // skip empty intervals
      if (u > i.second) {
        contour_cell cell;
        cell.interval_u =
            interval_type(i.second, u, tml::excluded, tml::excluded);
        cell.interval_v = i.first;
        result.push_back(cell);
      }
      intervals_to_remove.push_back(i.first);
    }
  }
  /*
  if ( intervals_to_remove.empty() && intervals_to_add.empty() )
  {
    contour_cell cell;
    cell.interval_u = interval_type ( i.second, u, tml::excluded, tml::excluded
  );
    cell.interval_v = i.first;
    result.push_back ( cell );
  }*/

  // 3. delete deprecated intervals
  BOOST_FOREACH(interval_type const & i, intervals_to_remove) {
    intervals.erase(i);
  }

  // 4. add new intervals
  BOOST_FOREACH(interval_type const & i, intervals_to_add) {
    intervals.insert(std::make_pair(i, u));
  }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
template <typename value_t>
contour_map_kd<value_t>::contour_map_kd()
    : _cells(), _root(nullptr) {}

///////////////////////////////////////////////////////////////////////////////
template <typename value_t> contour_map_kd<value_t>::~contour_map_kd() {
  destroy(_root);
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_t> void contour_map_kd<value_t>::initialize() {
  _cells.clear();

  if (_contour_segments.empty())
    return;

  update_bounds();

  // add events to event queue
  event_queue Q;

  interval_type vbounds(_bounds.min[point_type::v],
                        _bounds.max[point_type::v],
                        tml::excluded,
                        tml::excluded);
  Q.events.insert(new domain_start_event(_bounds.min[point_type::u], vbounds));
  Q.events.insert(new domain_end_event(_bounds.max[point_type::u], vbounds));
  BOOST_FOREACH(contour_segment_ptr const & p, _contour_segments) {
    // only use contours that cause computations ( have area where curves need
    // to be evaluated )
    if (!p->is_constant(point_type::u) && !p->is_constant(point_type::v)) {
      Q.events.insert(new contour_start_event(p->bbox().min[point_type::u], p));
      Q.events.insert(new contour_end_event(p->bbox().max[point_type::u], p));
    }
  }

  status_queue S;
  while (!Q.events.empty()) {
    auto e = Q.events.begin();
    (**e).visit(S);
    delete* e;
    Q.events.erase(e);
  }

  _cells = S.result;

  // TODO: O(n^2)! improve performance by sorting
  BOOST_FOREACH(contour_cell & c, _cells) {
    BOOST_FOREACH(contour_segment_ptr const & p, _contour_segments) {
      interval_type segment_v(p->bbox().min[point_type::v],
                              p->bbox().max[point_type::v],
                              tml::excluded,
                              tml::excluded);
      interval_type segment_u(p->bbox().min[point_type::u],
                              p->bbox().max[point_type::u],
                              tml::excluded,
                              tml::excluded);
      if (c.interval_v.overlap(segment_v) && c.interval_u.overlap(segment_u)) {
        c.overlapping_segments.push_back(p);
      }
      unsigned intersections = 0;
      if (segment_v.in(c.interval_v.center()) &&
          c.interval_u.center() < segment_u.minimum()) {
        ++intersections;
      }
      c.inside = (intersections % 2 == 1);
    }
  }

  // create kd-tree
  _root = create(_bounds, _cells);

}

///////////////////////////////////////////////////////////////////////////////
template <typename value_t> void contour_map_kd<value_t>::destroy(kdnode* n) {
  if (n == nullptr) {
    return;
  } else {

    if (n->is_child()) {
      delete n->cell;
      delete n;
    } else {
      destroy(n->less);
      destroy(n->more);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_t>
typename contour_map_kd<value_t>::kdnode* contour_map_kd<value_t>::create(
    bbox_type const& bounds,
    std::vector<contour_cell> const& cells) {
  std::set<value_type> usplit = split_candidates(bounds, point_type::u, cells);
  std::set<value_type> vsplit = split_candidates(bounds, point_type::v, cells);

  if (usplit.empty() && vsplit.empty()) {
    assert(cells.size() == 1);
    return new kdnode(0, 0, new contour_cell(cells.front()), nullptr, nullptr);
  } else {
    if (usplit.size() > vsplit.size()) {
      return split(bounds, point_type::u, usplit, cells);
    } else {
      return split(bounds, point_type::v, vsplit, cells);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_t>
typename contour_map_kd<value_t>::kdnode* contour_map_kd<value_t>::split(
    bbox_type const& bounds,
    typename point_type::coordinate_type const& dim,
    std::set<value_type> const& candidates,
    std::vector<contour_cell> const& cells) {
  assert(!candidates.empty());

  // choose random split plane
  std::size_t random_offset = std::rand() % candidates.size();
  auto split_iter = candidates.begin();
  std::advance(split_iter, random_offset);
  value_type split = *split_iter;

  // sort cells
  std::vector<contour_cell> less_cells;
  std::vector<contour_cell> more_cells;

  for (auto c = cells.begin(); c != cells.end(); ++c) {
    switch (dim) {
      case point_type::u:
        if (c->interval_u.in(split)) {
          less_cells.push_back(*c);
          more_cells.push_back(*c);
        }
        if (c->interval_u.minimum() >= split) {
          more_cells.push_back(*c);
        }
        if (c->interval_u.maximum() <= split) {
          less_cells.push_back(*c);
        }
        break;
      case point_type::v:
        if (c->interval_v.in(split)) {
          less_cells.push_back(*c);
          more_cells.push_back(*c);
        }
        if (c->interval_v.minimum() >= split) {
          more_cells.push_back(*c);
        }
        if (c->interval_v.maximum() <= split) {
          less_cells.push_back(*c);
        }
        break;
    }
  }

  bbox_type less_bounds =
      (dim == point_type::u)
          ? bbox_type(point_type(bounds.min[point_type::u],
                                 bounds.min[point_type::v]),
                      point_type(split, bounds.max[point_type::v]))
          : bbox_type(point_type(bounds.min[point_type::u],
                                 bounds.min[point_type::v]),
                      point_type(bounds.max[point_type::u], split));

  bbox_type more_bounds =
      (dim == point_type::u)
          ? bbox_type(point_type(split, bounds.min[point_type::v]),
                      point_type(bounds.max[point_type::u],
                                 bounds.max[point_type::v]))
          : bbox_type(point_type(bounds.min[point_type::u], split),
                      point_type(bounds.max[point_type::u],
                                 bounds.max[point_type::v]));

  return new kdnode(split,
                    dim,
                    nullptr,
                    create(less_bounds, less_cells),
                    create(more_bounds, more_cells));
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_t>
std::set<value_t> contour_map_kd<value_t>::split_candidates(
    bbox_type const& bounds,
    typename point_type::coordinate_type const& dim,
    std::vector<contour_cell> const& cells) {
  std::set<value_type> splits;
  for (auto c = cells.begin(); c != cells.end(); c) {
    switch (dim) {
      case point_type::u:
        splits.insert(c->interval_u.minimum());
        splits.insert(c->interval_u.maximum());
        break;
      case point_type::v:
        splits.insert(c->interval_v.minimum());
        splits.insert(c->interval_v.maximum());
        break;
    }
    ;
  }

  splits.erase(bounds.min[dim]);
  splits.erase(bounds.max[dim]);

  return splits;
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_t>
void contour_map_kd<value_t>::print(std::ostream& os) const {
  base_type::print(os);
  os << "cells : " << _cells.size() << std::endl;
  BOOST_FOREACH(contour_cell const & cell, _cells) {
    os << "interval u of cell : " << cell.interval_u << std::endl;
    os << "interval v of cell : " << cell.interval_v << std::endl;
    os << "number of segments in cell : " << cell.overlapping_segments.size()
       << std::endl;
  }
}

///////////////////////////////////////////////////////////////////////////////
template <typename value_t>
    std::ostream& operator<<(std::ostream& os,
                             tml::contour_map_kd<value_t> const& rhs) {
  rhs.print(os);
  return os;
}

}  // namespace tml
