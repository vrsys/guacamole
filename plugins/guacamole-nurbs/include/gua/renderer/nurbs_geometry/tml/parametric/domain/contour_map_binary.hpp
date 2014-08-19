/********************************************************************************
*
* Copyright (C) 2013 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : contour_map_binary.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_CONTOUR_MAP_BINARY_HPP
#define TML_CONTOUR_MAP_BINARY_HPP

// includes, system
#include <vector>

// includes, project
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/contour_map_base.hpp>

namespace tml {

template <typename value_t>
class contour_map_binary : public contour_map_base<value_t> {
 public:  // typedef / enums / types

  typedef contour_map_base<value_t> base_type;

  // stupid gcc workaround
  typedef typename base_type::value_type value_type;
  typedef typename base_type::point_type point_type;
  typedef typename base_type::interval_type interval_type;
  typedef typename base_type::contour_segment_ptr contour_segment_ptr;
  typedef typename base_type::contour_segment_container
      contour_segment_container;

  struct contour_cell {
    void print(std::ostream& os) const;

    contour_segment_container overlapping_segments;
    interval_type interval_u;
    interval_type interval_v;
    bool inside;
  };

  struct contour_interval {
    std::vector<contour_cell> cells;
    interval_type interval_u;
    interval_type interval_v;
  };

  typedef std::vector<contour_interval> cell_partition;

 public:  // c'tor / d'tor

  contour_map_binary();
  /* virtual */ ~contour_map_binary();

 public:  // methods

  /* virtual */ void initialize();

  cell_partition const& partition() const;

  /* virtual */ void print(std::ostream& os) const;

 protected:  // methods

 private:  // internal/auxilliary methods

 protected:  // attributes

  cell_partition _cells;

};

template <typename value_t>
    std::ostream& operator<<(std::ostream& os,
                             tml::contour_map_binary<value_t> const& rhs);

}  // namespace tml

#include "contour_map_binary_impl.hpp"

#endif  // TML_CONTOUR_MAP_BINARY_HPP
