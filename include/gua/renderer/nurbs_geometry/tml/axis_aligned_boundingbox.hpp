/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : axis_aligned_boundingbox.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_AXIS_ALIGNED_BOUNDING_BOX_HPP
#define TML_AXIS_ALIGNED_BOUNDING_BOX_HPP

// header, system
#include <list>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

#include <gua/renderer/nurbs_geometry/tml/abstract_boundingbox.hpp>
#include <gua/renderer/nurbs_geometry/tml/matrix.hpp>
#include <gua/renderer/nurbs_geometry/tml/interval.hpp>

namespace tml {

template <typename point_t>
class axis_aligned_boundingbox : public abstract_boundingbox<point_t> {
 public:  // typedefs

  typedef axis_aligned_boundingbox<point_t> type;
  typedef boost::shared_ptr<type> pointer_type;

  typedef typename point_t::value_type value_type;
  typedef point_t point_type;
  typedef interval<value_type> interval_type;

 public:  // ctor/dtor

  axis_aligned_boundingbox();

  axis_aligned_boundingbox(point_t const& min, point_t const& max);

  template <typename iterator_t>
  axis_aligned_boundingbox(iterator_t begin, iterator_t end);

  axis_aligned_boundingbox(axis_aligned_boundingbox const& bb);

 public:  // copy

  axis_aligned_boundingbox& operator=(axis_aligned_boundingbox const& rhs);
  virtual ~axis_aligned_boundingbox();

 public:  // methods

  void swap(axis_aligned_boundingbox& bb);

  void merge(axis_aligned_boundingbox<point_t> const& a);
  void merge(point_t const& p);
  void scale(value_type const& uniform_scale);

  template <typename matrix_t> void transform(matrix_t const& m);

  point_t center() const;
  point_t size() const;

  value_type volume() const;
  value_type surface() const;

  bool valid() const;

  bool is_inside(point_t const& p) const;
  bool is_inside(axis_aligned_boundingbox<point_t> const& a) const;

  bool overlap(axis_aligned_boundingbox<point_t> const& a) const;
  interval_type extends(typename point_type::coordinate_type const& dimension,
                        boundary_type min_bounds = excluded,
                        boundary_type max_bounds = excluded) const;

  template <typename insert_iterator>
  void generate_corners(insert_iterator i) const;
  void generate_corners(std::list<point_t>&) const;
  void generate_corners(std::vector<point_t>&) const;

  template <typename insert_iterator>
  void uniform_split_ptr(insert_iterator i) const;

  template <typename insert_iterator>
  void uniform_split(insert_iterator i) const;

  virtual void uniform_split(
      std::list<typename abstract_boundingbox<point_t>::pointer_type>& l) const;

  virtual void print(std::ostream& os) const;
  virtual void write(std::ostream& os) const;
  virtual void read(std::istream& is);

  point_t min;
  point_t max;
};

template <typename point_t>
axis_aligned_boundingbox<point_t> merge(
    axis_aligned_boundingbox<point_t> const& a,
    axis_aligned_boundingbox<point_t> const& b);

template <typename point_t>
    std::ostream& operator<<(std::ostream& os,
                             axis_aligned_boundingbox<point_t> const& a);

typedef axis_aligned_boundingbox<point2f> bbox2f;
typedef axis_aligned_boundingbox<point3f> bbox3f;
typedef axis_aligned_boundingbox<point2d> bbox2d;
typedef axis_aligned_boundingbox<point3d> bbox3d;

}  // namespace tml

#include <gua/renderer/nurbs_geometry/tml/axis_aligned_boundingbox_impl.hpp>

#endif  // TML_AXIS_ALIGNED_BOUNDING_BOX_HPP
