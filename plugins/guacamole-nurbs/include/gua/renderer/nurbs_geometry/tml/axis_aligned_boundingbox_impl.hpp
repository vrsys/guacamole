/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : axis_aligned_axis_aligned_boundingbox_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_AXIS_ALIGNED_BOUNDINGBOX_IMPL_HPP
#define TML_AXIS_ALIGNED_BOUNDINGBOX_IMPL_HPP

#include <gua/renderer/nurbs_geometry/tml/interval.hpp>

// header, system
#include <vector>
#include <limits>

// header, project
#include <gua/renderer/nurbs_geometry/tml/pow.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/minimal_coordinates.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/maximal_coordinates.hpp>

namespace tml {

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
axis_aligned_boundingbox<point_t>::axis_aligned_boundingbox()
    : min(point_t::maximum()), max(point_t::minimum()) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
axis_aligned_boundingbox<point_t>::axis_aligned_boundingbox(point_t const& mn,
                                                            point_t const& mx)
    : min(mn), max(mx) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename iterator_t>
axis_aligned_boundingbox<point_t>::axis_aligned_boundingbox(iterator_t begin,
                                                            iterator_t end)
    : min(point_t::maximum()), max(point_t::minimum()) {
  while (begin != end) {
    min = elementwise_min(*begin, min);
    max = elementwise_max(*begin, max);
    ++begin;
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
axis_aligned_boundingbox<point_t>::axis_aligned_boundingbox(
    axis_aligned_boundingbox const& bb)
    : min(bb.min), max(bb.max) {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
axis_aligned_boundingbox<point_t>& axis_aligned_boundingbox<point_t>::operator=(
    axis_aligned_boundingbox const& rhs) {
  axis_aligned_boundingbox tmp(rhs);
  swap(tmp);
  return *this;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
axis_aligned_boundingbox<point_t>::~axis_aligned_boundingbox() {}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
void axis_aligned_boundingbox<point_t>::swap(axis_aligned_boundingbox& bb) {
  std::swap(min, bb.min);
  std::swap(max, bb.max);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
void axis_aligned_boundingbox<point_t>::merge(
    axis_aligned_boundingbox<point_t> const& a) {
  min = elementwise_min(min, a.min);
  max = elementwise_max(max, a.max);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
void axis_aligned_boundingbox<point_t>::merge(point_t const& p) {
  min = elementwise_min(min, p);
  max = elementwise_max(max, p);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename matrix_t>
void axis_aligned_boundingbox<point_t>::transform(matrix_t const& m) {
  std::vector<point_t> vertices;
  std::vector<point_t> transformed_vertices;

  generate_corners(vertices);

  std::transform(vertices.begin(),
                 vertices.end(),
                 std::back_inserter(transformed_vertices),
                 [&](point_t const & p) {
    return m * p;
  });

  swap(axis_aligned_boundingbox(transformed_vertices.begin(),
                                transformed_vertices.end()));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
void axis_aligned_boundingbox<point_t>::scale(value_type const& uniform_scale) {
  point_t d = max - min;
  min -= uniform_scale * d;
  max += uniform_scale * d;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
point_t axis_aligned_boundingbox<point_t>::center() const {
  return (min + max) / value_type(2);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
point_t axis_aligned_boundingbox<point_t>::size() const {
  return max - min;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
typename axis_aligned_boundingbox<point_t>::value_type
axis_aligned_boundingbox<point_t>::volume() const {
  value_type volume = 1;

  for (unsigned i = 0; i != point_t::coordinates; ++i) {
    volume *= max[i] - min[i];
  }

  return volume;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
typename axis_aligned_boundingbox<point_t>::value_type
axis_aligned_boundingbox<point_t>::surface() const {
  value_type surftotal = 0;

  for (unsigned i = 0; i != point_t::coordinates; ++i) {
    for (unsigned j = i; j != point_t::coordinates; ++j) {
      if (i != j) {
        surftotal += (max[i] - min[i]) * (max[j] - min[j]);
      }
    }
  }

  return surftotal;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
bool axis_aligned_boundingbox<point_t>::valid() const {
  for (std::size_t i = 0; i != point_t::coordinates; ++i) {
    if (min[i] > max[i]) {
      return false;
    }
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
bool axis_aligned_boundingbox<point_t>::is_inside(point_t const& p) const {
  for (unsigned i = 0; i != point_t::coordinates; ++i) {
    tml::interval<typename point_t::value_type> axis_range(
        min[i], max[i], tml::included, tml::included);
    if (!axis_range.in(p[i])) {
      return false;
    }
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
bool axis_aligned_boundingbox<point_t>::is_inside(
    axis_aligned_boundingbox<point_t> const& other) const {
  for (unsigned i = 0; i != point_t::coordinates; ++i) {
    tml::interval<typename point_t::value_type> interval_axis(
        min[i], max[i], tml::included, tml::included);
    tml::interval<typename point_t::value_type> interval_other(
        other.min[i], other.max[i], tml::included, tml::included);

    // if other bounding box exceeds dimensions in one axis it cannot be
    // included in this bbox
    if (!interval_axis.in(interval_other)) {
      return false;
    }
  }

  // other bounding box's lies within this bounding box in all dimensions
  return true;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
bool axis_aligned_boundingbox<point_t>::overlap(
    axis_aligned_boundingbox<point_t> const& a) const {
  bool overlap = true;

  for (std::size_t i = 0; i != point_t::coordinates; ++i) {
    tml::interval<typename point_t::value_type> range(
        min[i], max[i], tml::included, tml::included);
    tml::interval<typename point_t::value_type> range_other(
        a.min[i], a.max[i], tml::included, tml::included);
    overlap &= range.overlap(range_other);

    // early abort
    if (!overlap) {
      return overlap;
    }
  }
  return overlap;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
typename axis_aligned_boundingbox<point_t>::interval_type
axis_aligned_boundingbox<point_t>::extends(
    typename point_type::coordinate_type const& dimension,
    boundary_type min_bounds,
    boundary_type max_bounds) const {
  return interval_type(min[dimension], max[dimension], min_bounds, max_bounds);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename insert_iterator>
void axis_aligned_boundingbox<point_t>::generate_corners(
    insert_iterator ins) const {
  std::size_t vertices = tml::meta_pow<2, point_t::coordinates>::result;

  for (unsigned v = 0; v != vertices; ++v) {
    point_t corner;
    std::size_t minmax_bitmask = v;

    for (unsigned i = 0; i != point_t::coordinates; ++i) {
      corner[i] = (minmax_bitmask & 0x01) ? max[i] : min[i];
      minmax_bitmask >>= 1;
    }

    ins = corner;
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
void axis_aligned_boundingbox<point_t>::generate_corners(
    std::list<point_t>& result) const {
  generate_corners(std::back_inserter(result));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
void axis_aligned_boundingbox<point_t>::generate_corners(
    std::vector<point_t>& result) const {
  generate_corners(std::back_inserter(result));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename insert_iterator>
void axis_aligned_boundingbox<point_t>::uniform_split_ptr(
    insert_iterator ins) const {
  std::vector<axis_aligned_boundingbox> boxes;
  uniform_split(std::back_inserter(boxes));

  for (auto b = boxes.begin(); b != boxes.end(); ++b) {
    ins = axis_aligned_boundingbox::pointer_type(
        new axis_aligned_boundingbox<point_t>(*b));
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename insert_iterator>
void axis_aligned_boundingbox<point_t>::uniform_split(
    insert_iterator ins) const {
  std::size_t parts = tml::meta_pow<2, point_t::coordinates>::result;

  for (unsigned p = 0; p != parts; ++p) {
    axis_aligned_boundingbox subbox;
    std::size_t minmax_bitmask = p;

    for (unsigned i = 0; i != point_t::coordinates; ++i) {
      subbox.min[i] =
          (minmax_bitmask & 0x01) ? (min[i] + max[i]) / value_type(2.0)
                                  : min[i];
      subbox.max[i] = (minmax_bitmask & 0x01) ? max[i] : (min[i] + max[i]) /
                                                             value_type(2.0);

      value_type minweight =
          (minmax_bitmask & 0x01)
              ? (min.weight() + max.weight()) / value_type(2.0)
              : min.weight();
      value_type maxweight =
          (minmax_bitmask & 0x01)
              ? max.weight()
              : (min.weight() + max.weight()) / value_type(2.0);

      subbox.min.weight(minweight);
      subbox.max.weight(maxweight);

      minmax_bitmask >>= 1;
    }

    ins = subbox;
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
void axis_aligned_boundingbox<point_t>::uniform_split(
    std::list<typename abstract_boundingbox<point_t>::pointer_type>& l) const {
  uniform_split_ptr(std::back_inserter(l));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
/* virtual */ void axis_aligned_boundingbox<point_t>::print(
    std::ostream& os) const {
  os << "Axis aligned bounding box: "
     << "Min : " << min << " , Max : " << max;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
/* virtual */ void axis_aligned_boundingbox<point_t>::write(
    std::ostream& os) const {
  os.write(reinterpret_cast<char const*>(&min[0]), sizeof(point_type));
  os.write(reinterpret_cast<char const*>(&max[0]), sizeof(point_type));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
/* virtual */ void axis_aligned_boundingbox<point_t>::read(std::istream& is) {
  is.read(reinterpret_cast<char*>(&min[0]), sizeof(point_type));
  is.read(reinterpret_cast<char*>(&max[0]), sizeof(point_type));
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
axis_aligned_boundingbox<point_t> merge(
    axis_aligned_boundingbox<point_t> const& a,
    axis_aligned_boundingbox<point_t> const& b) {
  axis_aligned_boundingbox<point_t> tmp(a);
  tmp.merge(b);
  return tmp;
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t>
    std::ostream& operator<<(std::ostream& os,
                             axis_aligned_boundingbox<point_t> const& a) {
  a.print(os);
  return os;
}

}  // namespace tml

#endif  // TML_AXIS_ALIGNED_BOUNDINGBOX_IMPL_HPP
