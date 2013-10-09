/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : vertical_interval.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_VERTICAL_INTERVAL_HPP
#define TML_VERTICAL_INTERVAL_HPP

// includes, system
#include <set>
#include <map>

#include <boost/shared_ptr.hpp>

#include <gua/renderer/nurbs_geometry/tml/interval.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

// includes, project

// forward declarations
namespace tml {
template <typename point_type> class partition;
template <typename value_type> class uniform_cell;
template <typename point_type> class beziercurve_segment;
}

namespace tml {

template <typename value_t> class vertical_interval {
 public:  // typedefs / enums

  typedef value_t value_type;
  typedef interval<value_type> interval_type;
  typedef boost::shared_ptr<vertical_interval> shared_ptr_type;
  typedef point<value_type, 2> point_type;
  typedef interval<value_type> range_type;

  typedef partition<point_type> partition_type;
  typedef partition_type* partition_ptr_type;

  typedef typename partition_type::cell_type cell_type;
  typedef typename partition_type::cell_ptr_type cell_ptr_type;
  typedef typename partition_type::cell_ptr_set cell_ptr_set;

  typedef typename cell_ptr_set::iterator iterator;
  typedef typename cell_ptr_set::const_iterator const_iterator;
  typedef typename cell_ptr_set::reverse_iterator reverse_iterator;
  typedef typename cell_ptr_set::const_reverse_iterator const_reverse_iterator;

  typedef beziercurve_segment<point_type> curve_segment_type;
  typedef boost::shared_ptr<curve_segment_type> curve_segment_ptr;
  typedef std::set<curve_segment_ptr> curve_segment_set;
  typedef typename curve_segment_set::iterator curve_segment_iterator;
  typedef typename curve_segment_set::const_iterator
      curve_segment_const_iterator;

 public:  // c'tor / d'tor

  vertical_interval(interval_type const& interval, partition_ptr_type parent);

  vertical_interval(interval_type const& interval,
                    partition_ptr_type parent,
                    shared_ptr_type less,
                    shared_ptr_type more);

  ~vertical_interval();

 public:  // methods

  // set precessing or successing interval
  void previous(shared_ptr_type p);
  void next(shared_ptr_type s);

  shared_ptr_type previous() const;
  shared_ptr_type next() const;

  // limits of vertical interval
  interval_type const& get_vertical_interval() const;
  interval_type get_horizontal_interval() const;

  void add(curve_segment_ptr const& segment);
  void add(cell_ptr_type const& cell);

  std::size_t curve_segments() const;

  void print(std::ostream& os, std::string const& addinfo = "") const;

  // all STL-type methods concern map with uniform cells
  std::size_t size() const;
  bool empty() const;
  void clear();
  void insert(interval_type const& horizontal_interval, cell_ptr_type cell);

  // iterator interface
  iterator begin();
  iterator end();
  const_iterator begin() const;
  const_iterator end() const;

  reverse_iterator rbegin();
  reverse_iterator rend();
  const_reverse_iterator rbegin() const;
  const_reverse_iterator rend() const;

  cell_ptr_type const& front() const;
  cell_ptr_type const& back() const;
  cell_ptr_type find(value_type const& value) const;

  curve_segment_const_iterator segment_begin() const;
  curve_segment_const_iterator segment_end() const;

  void update_vicinity() const;

 private:  // internal methods

 private:  // attributes

  interval_type _interval;

  partition_ptr_type _partition;

  shared_ptr_type _previous;
  shared_ptr_type _next;

  curve_segment_set _segments;
  cell_ptr_set _cell_set;

  value_type _area;
};

template <typename value_type>
    std::ostream& operator<<(std::ostream& os,
                             tml::vertical_interval<value_type> const& rhs);

}  // namespace tml

#include <gua/vertical_interval_impl.hpp>

#endif  // TML_VERTICAL_INTERVAL_HPP
