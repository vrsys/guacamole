/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : partition.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_PARTITION_HPP
#define TML_PARTITION_HPP

// includes, system
#include <set>
#include <map>

#include <boost/shared_ptr.hpp>

#include <gua/renderer/nurbs_geometry/tml/interval.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/beziercurve.hpp>
//#include
//<renderer/nurbs_geometry/tml/parametric/domain/vertical_interval.hpp>

// includes, project

namespace tml {
template <typename value_type> class vertical_interval;
template <typename point_type> class beziercurve_segment;
template <typename value_type> class uniform_cell;
}

namespace tml {

template <typename vertical_interval_ptr> struct compare_vertical_interval_ptr {
  bool operator()(vertical_interval_ptr const& a,
                  vertical_interval_ptr const& b) const {
    return a->get_vertical_interval() < b->get_vertical_interval();
  }
};

template <typename uniform_cell_ptr> struct compare_uniform_cell_ptr {
  bool operator()(uniform_cell_ptr const& a, uniform_cell_ptr const& b) const {
    return a->get_horizontal_interval() < b->get_horizontal_interval();
  }
};

template <typename point_t> class partition {
 private:  // internal parameters

  static std::size_t const MAX_ROOTFIND_ITERATIONS = 64;

 public:  // typedef / enums

  typedef typename point_t::value_type value_type;
  typedef point_t point_type;
  typedef interval<value_type> interval_type;

  typedef beziercurve<point_type> curve_type;
  typedef boost::shared_ptr<curve_type> curve_ptr_type;

  typedef std::set<curve_ptr_type> container_type;
  typedef typename container_type::iterator curve_iterator;
  typedef typename container_type::const_iterator const_curve_iterator;

  typedef vertical_interval<value_type> vertical_interval_type;
  typedef boost::shared_ptr<vertical_interval_type> vertical_interval_ptr;
  typedef std::set<vertical_interval_ptr,
                   compare_vertical_interval_ptr<vertical_interval_ptr> >
      vertical_interval_set;

  typedef typename vertical_interval_set::iterator iterator;
  typedef typename vertical_interval_set::const_iterator const_iterator;
  typedef typename vertical_interval_set::reverse_iterator reverse_iterator;
  typedef typename vertical_interval_set::const_reverse_iterator
      const_reverse_iterator;

  typedef beziercurve_segment<point_type> curve_segment_type;
  typedef boost::shared_ptr<curve_segment_type> curve_segment_ptr;
  typedef std::set<curve_segment_ptr> segment_set;

  typedef uniform_cell<value_type> cell_type;
  typedef boost::shared_ptr<cell_type> cell_ptr_type;
  typedef std::set<cell_ptr_type, compare_uniform_cell_ptr<cell_ptr_type> >
      cell_ptr_set;

 public:  // c'tor / d'tor

  partition();

  template <typename curve_ptr_iterator_type>
  partition(curve_ptr_iterator_type begin, curve_ptr_iterator_type end);

  virtual ~partition();

 public:  // methods

  template <typename curve_ptr_iterator_type>
  void set(curve_ptr_iterator_type begin, curve_ptr_iterator_type end);

  // initial partitioning scheme according to given monotonic curves
  void initialize();

  // number of vertical intervals the parameter domain is partitioned into
  std::size_t size() const;
  bool empty() const;

  // number of monotonic curves in partition
  std::size_t curves() const;

  void split(vertical_interval_ptr const& interval, value_type const& value);

  // iterator interface for vertical intervals the parameter domain contains
  iterator begin();
  iterator end();
  const_iterator begin() const;
  const_iterator end() const;

  reverse_iterator rbegin();
  reverse_iterator rend();
  const_reverse_iterator rbegin() const;
  const_reverse_iterator rend() const;

  // extends of parameter domain
  interval_type const& get_horizontal_interval() const;
  interval_type const& get_vertical_interval() const;

  void update_vicinity() const;
  void update_size_from_curves();
  void update_size_from_intervals();

  cell_ptr_type find(point_type const&) const;

  // stream output of domain
  virtual void print(std::ostream& os) const;

 protected:  // methods

  // after creating a basic vertical partition the
  // horizontal direction is partitioned into cells
  virtual void initial_horizontal_partition(vertical_interval_ptr const&);

  // precompute for all cells the curve segments that are:
  //    - may be intersected and have to be tested at runtime
  virtual void precompute_curves_to_intersect(vertical_interval_ptr const&);

  // create a vertical segmentation according to the
  // start and end points of the monotonic curves
  virtual void initial_vertical_partition();

 private:  // internal/auxilliary methods

  // split input curves at all extrema and
  // store the monotonic segments within this class
  template <typename curve_ptr_iterator_type>
  void _monotonize(curve_ptr_iterator_type begin, curve_ptr_iterator_type end);
  // compute boundary according to curves
  void _compute_size_from_curves();

  // point curves to vertically increasing direction
  void _align_vertically_increasing();

  // create curve segments by intersecting vertical intervals with
  // monotonic curves and assign segments to these vertical intervals
  void _create_curve_segments();

  // precompute for all cells the curve segments that are:
  //    - definitely intersected for rays originating in that cell
  void _precompute_known_intersections(vertical_interval_ptr const&);

 protected:  // attributes

  // geometrical data
  container_type _monotonic_curves;

  // size
  interval_type _horizontal_interval;
  interval_type _vertical_interval;

  // partition into vertical slices
  vertical_interval_set _vertical_partition;
};

template <typename point_type>
    std::ostream& operator<<(std::ostream& os,
                             tml::partition<point_type> const& rhs);

}  // namespace tml

#include <gua/partition_impl.hpp>

#endif  // TML_PARTITION_HPP
