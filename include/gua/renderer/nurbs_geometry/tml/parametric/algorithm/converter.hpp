/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : converter.hpp
*  project    : gua
*  description:
*
********************************************************************************/
#ifndef TML_CONVERTER_HPP
#define TML_CONVERTER_HPP

// header, system
#include <vector>
#include <set>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

namespace tml {

template <typename point_t> class nurbsvolume;
template <typename point_t> class nurbssurface;
template <typename point_t> class nurbscurve;
template <typename point_t> class beziervolume;
template <typename point_t> class beziersurface;
template <typename point_t> class beziercurve;

template <typename point_t> struct beziersurface_from_nurbs {
 public:
  typedef typename point_t::value_type value_type;

  beziersurface<point_t> surface;

  value_type umin;
  value_type umax;
  value_type vmin;
  value_type vmax;
};

template <typename point_t> struct beziervolume_from_nurbs {
 public:
  typedef typename point_t::value_type value_type;

  beziervolume<point_t> volume;

  value_type umin;
  value_type umax;
  value_type vmin;
  value_type vmax;
  value_type wmin;
  value_type wmax;
};

struct beziervolumeindex {
  unsigned u;
  unsigned v;
  unsigned w;
};

template <typename point_t> class converter {
 public:  // typedefs and enums

  typedef point_t point_type;
  typedef typename point_t::value_type value_type;
  typedef std::vector<value_type> knotvec_type;

  typedef nurbsvolume<point_t> nurbsvolume_type;
  typedef nurbssurface<point_t> nurbsurface_type;
  typedef nurbscurve<point_t> nurbscurve_type;

  typedef beziervolume<point_t> beziervolume_type;
  typedef beziersurface<point_t> beziersurface_type;
  typedef beziercurve<point_t> beziercurve_type;

 public:  // operators

  converter() {}
  virtual ~converter() {}

  // convert a nurbscurve and insert into container
  template <typename insert_iterator_t>
  void convert(nurbscurve_type const& surface,
               insert_iterator_t ins_iter) const;

  // convert a nurbssurfvace and insert into container
  template <typename insert_iterator_t>
  void convert(nurbsurface_type const& surface,
               insert_iterator_t ins_iter) const;

  // convert a nurbsvolume and insert into container

  template <typename insert_iterator_t>
  void convert(nurbsvolume_type const& volume,
               insert_iterator_t ins_iter,
               std::list<beziervolumeindex>& indices =
                   std::list<beziervolumeindex>()) const;

  // do a complete knot insertion to convert into bezier pieces
  // insert all internal knots degree times
  template <typename point_container_t>
  void knot_insertion(point_container_t& points,
                      std::multiset<value_type>& knots,
                      std::size_t order) const;

  // insert one knot into knotvector
  template <typename point_container_t>
  void knot_insertion(point_container_t& points,
                      std::multiset<value_type>& knots,
                      std::size_t order,
                      value_type knot) const;
};

typedef converter<point2f> converter2f;
typedef converter<point2d> converter2d;
typedef converter<point3f> converter3f;
typedef converter<point3d> converter3d;

}  // namespace tml

#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/converter_impl.hpp>

#endif  // TML_CONVERTER_HPP
