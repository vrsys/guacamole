/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : converter_impl.hpp
*  project    : gua
*  description:
*
********************************************************************************/
// header, system
#include <vector>     // std::vector
#include <set>        // std::multiset
#include <algorithm>  // std::transform

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/pointmesh2d.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/beziercurve.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/beziersurface.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/beziervolume.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/nurbscurve.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/nurbssurface.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/nurbsvolume.hpp>

namespace tml {

////////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename insert_iterator_t>
inline void converter<point_t>::convert(tml::nurbscurve<point_t> const& nc,
                                        insert_iterator_t target) const {
  // copy control points into helper array
  std::vector<point_t> points(nc.begin(), nc.end());
  std::multiset<value_type> knots(nc.knots().begin(), nc.knots().end());

  std::size_t order = nc.order();
  std::size_t deg = nc.degree();

  if (order <= 1) {
    std::cerr << "converter::convert() : irregular order " << std::endl;
  }

  if (points.size() + order == knots.size()) {
    knot_insertion(points, knots, order);
  } else {
    std::cerr << "converter(): irregular knotvector" << std::endl;
  }

  if (points.size() + order != knots.size()) {
    std::cerr << "converter::convert() : knot insertion failed " << std::endl;
  }

  // compute how many parts result
  std::size_t parts = (knots.size() - 2 * order) / (order - 1) + 1;

  // copy bezier curves
  typename std::vector<point_t>::iterator start(points.begin());
  typename std::vector<point_t>::iterator end(points.begin());
  std::advance(end, order);

  // create bezier parts and store them
  for (std::size_t i = 0; i < parts; ++i) {
    beziercurve<point_t> bc(start, end);

    if (i < (parts - 1)) {
      std::advance(start, deg);
      std::advance(end, deg);
    }

    target = bc;
  }
}

////////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename insert_iterator_t>
inline void converter<point_t>::convert(tml::nurbssurface<point_t> const& nurbs,
                                        insert_iterator_t target) const {
  // copy control points
  std::size_t orig_cols = nurbs.numberofpoints_u();
  std::size_t orig_rows = nurbs.numberofpoints_v();

  std::multiset<value_type> orig_knotU(nurbs.knotvector_u().begin(),
                                       nurbs.knotvector_u().end());
  std::multiset<value_type> orig_knotV(nurbs.knotvector_v().begin(),
                                       nurbs.knotvector_v().end());

  std::size_t orderU = nurbs.order_u();
  std::size_t orderV = nurbs.order_v();

  // orignal polygon
  pointmesh2d<point_t> orig(
      nurbs.points().begin(), nurbs.points().end(), orig_cols, orig_rows);

  // helper polygons
  pointmesh2d<point_t> knotins_rows;
  pointmesh2d<point_t> knotins_complete;

  // do for each row
  std::multiset<value_type> new_knotU;
  std::multiset<value_type> new_knotV;

  for (std::size_t i = 0; i < orig_rows; ++i) {
    std::vector<point_t> row = orig.row(i);
    new_knotU = orig_knotU;
    knot_insertion(row, new_knotU, orderU);
    knotins_rows.add_row(row);
  }

  // transpose control polygon
  knotins_rows.transpose();

  for (std::size_t i = 0; i < knotins_rows.height(); ++i) {
    std::vector<point_t> row = knotins_rows.row(i);
    new_knotV = orig_knotV;
    knot_insertion(row, new_knotV, orderV);
    knotins_complete.add_row(row);
  }

  // transpose back
  knotins_complete.transpose();

  // create beziersurfaces and put on bezier stack
  std::set<value_type> unique_u(new_knotU.begin(), new_knotU.end());
  std::set<value_type> unique_v(new_knotV.begin(), new_knotV.end());

  typename std::set<value_type>::iterator para_min_u(unique_u.begin());
  typename std::set<value_type>::iterator para_max_u(unique_u.begin());
  typename std::set<value_type>::iterator para_min_v(unique_v.begin());
  typename std::set<value_type>::iterator para_max_v(unique_v.begin());
  ++para_max_u;
  ++para_max_v;

  std::size_t parts_u = unique_u.size() - 1;  // knotspans over u
  std::size_t parts_v = unique_v.size() - 1;  // knotspans over v

  std::size_t index_u = 0;
  std::size_t index_v = 0;

  for (std::size_t r = 0; r < parts_v; ++r) {
    for (std::size_t c = 0; c < parts_u; ++c) {
      beziersurface_from_nurbs<point_t> patch;
      pointmesh2d<point_t> subpatch = knotins_complete.subpatch(
          index_u, index_u + orderU - 1, index_v, index_v + orderV - 1);

      // create subpatch and apply parameter
      patch.surface.mesh(subpatch.begin(), subpatch.end());

      patch.surface.order_u(orderU);
      patch.surface.order_v(orderV);

      // mininum and maximum parameter on nurbs -> for trimming
      /*patch.set_patch_parameter(*para_min_u, *para_max_u, *para_min_v,
*para_max_v);
      patch.set_nurbs_parameter(*new_knotU.begin(), *new_knotU.rbegin(),
*new_knotV.begin(), *new_knotV.rbegin());

      // apply trimcurves here!
      for (std::vector<beziercurve<double3_t> >::iterator i =
impl_->curves_.begin(); i < impl_->curves_.end(); ++i)
      {
	      patch.add(*i);
      }

      // set trimming type
      if (nurbs.trimType() == nurbssurface<double4_t>::inner) {
	      patch.trimtype(true);
      } else {
	      patch.trimtype(false);
      }*/

      patch.umin = *para_min_u;
      patch.umax = *para_max_u;
      patch.vmin = *para_min_v;
      patch.vmax = *para_max_v;

      target = patch;

      // prepare for next patch
      index_u += orderU - 1;
      ++para_max_u;
      ++para_min_u;
    }
    // set indices for next line
    index_v += orderV - 1;
    index_u = 0;

    // minimum and maximum of parameter range on nurbs
    para_min_u = unique_u.begin();
    para_max_u = unique_u.begin();
    ++para_max_u;

    ++para_max_v;
    ++para_min_v;
  }
}

////////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename insert_iterator_t>
inline void converter<point_t>::convert(
    nurbsvolume_type const& volume,
    insert_iterator_t ins_iter,
    std::list<beziervolumeindex>& indices) const {
  typedef typename std::set<value_type>::const_iterator set_const_iterator;

  std::set<value_type> unique_knots_u(volume.knotvector_u().begin(),
                                      volume.knotvector_u().end());
  std::set<value_type> unique_knots_v(volume.knotvector_v().begin(),
                                      volume.knotvector_v().end());
  std::set<value_type> unique_knots_w(volume.knotvector_w().begin(),
                                      volume.knotvector_w().end());

  std::vector<value_type> unique_knots_u_as_vector(unique_knots_u.begin(),
                                                   unique_knots_u.end());
  std::vector<value_type> unique_knots_v_as_vector(unique_knots_v.begin(),
                                                   unique_knots_v.end());
  std::vector<value_type> unique_knots_w_as_vector(unique_knots_w.begin(),
                                                   unique_knots_w.end());

  // compute target size of mesh after knot insertion
  std::size_t knotspans_u = unique_knots_u.size() - 1;
  std::size_t knotspans_v = unique_knots_v.size() - 1;
  std::size_t knotspans_w = unique_knots_w.size() - 1;

  std::size_t targetsize_u = (knotspans_u) * (volume.order_u() - 1) + 1;
  std::size_t targetsize_v = (knotspans_v) * (volume.order_v() - 1) + 1;
  std::size_t targetsize_w = (knotspans_w) * (volume.order_w() - 1) + 1;

  pointmesh3d<point_t> mesh(volume.points().begin(),
                            volume.points().end(),
                            volume.knotvector_u().size() - volume.order_u(),
                            volume.knotvector_v().size() - volume.order_v(),
                            volume.knotvector_w().size() - volume.order_w());

  // knot insertion in u direction
  pointmesh3d<point_t> mesh_u(
      targetsize_u, volume.numberofpoints_v(), volume.numberofpoints_w());
  for (unsigned int v = 0; v != volume.numberofpoints_v(); ++v) {
    for (unsigned int w = 0; w != volume.numberofpoints_w(); ++w) {
      std::vector<point_t> curve = mesh.submesh(0, v, w);
      std::multiset<value_type> knots(volume.knotvector_u().begin(),
                                      volume.knotvector_u().end());
      knot_insertion(curve, knots, volume.order_u());
      mesh_u.submesh(curve.begin(), 0, v, w);
    }
  }

  // knot insertion in v direction
  pointmesh3d<point_t> mesh_uv(
      targetsize_u, targetsize_v, volume.numberofpoints_w());
  for (unsigned int u = 0; u != targetsize_u; ++u) {
    for (unsigned int w = 0; w != volume.numberofpoints_w(); ++w) {
      std::vector<point_t> curve = mesh_u.submesh(1, u, w);
      std::multiset<value_type> knots(volume.knotvector_v().begin(),
                                      volume.knotvector_v().end());
      knot_insertion(curve, knots, volume.order_v());
      mesh_uv.submesh(curve.begin(), 1, u, w);
    }
  }

  // knot insertion in w direction
  pointmesh3d<point_t> mesh_uvw(targetsize_u, targetsize_v, targetsize_w);
  for (unsigned int u = 0; u != targetsize_u; ++u) {
    for (unsigned int v = 0; v != targetsize_v; ++v) {
      std::vector<point_t> curve = mesh_uv.submesh(2, u, v);
      std::multiset<value_type> knots(volume.knotvector_w().begin(),
                                      volume.knotvector_w().end());
      knot_insertion(curve, knots, volume.order_w());
      mesh_uvw.submesh(curve.begin(), 2, u, v);
    }
  }

  typename beziervolume_type::parameter_type uvwglobalmin(
      unique_knots_u_as_vector[0],
      unique_knots_v_as_vector[0],
      unique_knots_w_as_vector[0]);
  typename beziervolume_type::parameter_type uvwglobalmax(
      unique_knots_u_as_vector[knotspans_u],
      unique_knots_v_as_vector[knotspans_v],
      unique_knots_w_as_vector[knotspans_w]);

  indices.clear();

  // read control points of submeshes
  for (int u = 0; u != int(knotspans_u); ++u) {
    for (int v = 0; v != int(knotspans_v); ++v) {
      for (int w = 0; w != int(knotspans_w); ++w) {
        typename beziervolume_type::parameter_type uvwmin(
            unique_knots_u_as_vector[u],
            unique_knots_v_as_vector[v],
            unique_knots_w_as_vector[w]);
        typename beziervolume_type::parameter_type uvwmax(
            unique_knots_u_as_vector[u + 1],
            unique_knots_v_as_vector[v + 1],
            unique_knots_w_as_vector[w + 1]);

        // create control point mesh of target size
        pointmesh3d<point_t> bmesh(
            volume.order_u(), volume.order_v(), volume.order_w());

        for (unsigned int du = 0; du != volume.order_u(); ++du) {
          for (unsigned int dv = 0; dv != volume.order_v(); ++dv) {
            for (unsigned int dw = 0; dw != volume.order_w(); ++dw) {
              bmesh(du, dv, dw) = mesh_uvw(u * (volume.order_u() - 1) + du,
                                           v * (volume.order_v() - 1) + dv,
                                           w * (volume.order_w() - 1) + dw);
            }
          }
        }

        // create and insert sub bezier volume
        beziervolume<point_t> subvolume(bmesh);

        subvolume.uvw_local(uvwmin, uvwmax);
        subvolume.uvw_global(uvwglobalmin, uvwglobalmax);

        *ins_iter = subvolume;
        beziervolumeindex idx = { u, v, w };
        indices.push_back(idx);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename point_container_t>
inline void converter<point_t>::knot_insertion(
    point_container_t& P,
    std::multiset<typename point_t::value_type>& knots,
    std::size_t order) const {
  //typedef typename point_t::value_type value_type;
  std::set<value_type> unique(knots.begin(), knots.end());

  for (typename std::set<value_type>::const_iterator i = unique.begin();
       i != unique.end();
       ++i) {
    if (knots.count(*i) < order - 1) {
      knot_insertion(P, knots, order, *i);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
template <typename point_t>
template <typename point_container_t>
inline void converter<point_t>::knot_insertion(point_container_t& P,
                                               std::multiset<value_type>& knots,
                                               std::size_t order,
                                               value_type t) const {
  typedef typename point_t::value_type value_type;

  // copy knotvector for subscript [] access
  std::vector<value_type> kv_cpy(knots.begin(), knots.end());
  // get parameter
  std::size_t p = order - 1;                        // degree
  std::size_t s = knots.count(t);                   // multiplicity
  std::size_t r = std::max(std::size_t(0), p - s);  // number of insertions

  // get knotspan
  std::size_t k = std::distance(knots.begin(), knots.upper_bound(t));
  std::size_t np = P.size();  // number of control points

  // start computation
  std::size_t nq = np + r;

  // helper arrays
  std::vector<point_t> Qw(nq);
  std::vector<point_t> Rw(p - s + 1);

  // copy unaffected points and transform into homogenous coords
  for (size_t i = 0; i <= k - p; ++i) {
    Qw[i] = P[i].as_homogenous();
  }
  for (size_t i = k - s - 1; i <= np - 1; ++i) {
    Qw[i + r] = P[i].as_homogenous();
  }

  // helper points
  for (size_t i = 0; i <= p - s; ++i) {
    Rw[i] = P[k - p + i - 1].as_homogenous();
  }

  // do knot insertion itself
  std::size_t L = 0;
  for (std::size_t j = 1; j <= r; ++j) {
    L = k - p + j;
    for (std::size_t i = 0; i <= p - j - s; ++i) {
      value_type alpha =
          (t - kv_cpy[L + i - 1]) / (kv_cpy[i + k] - kv_cpy[L + i - 1]);
      Rw[i] = alpha * Rw[i + 1] + value_type(1.0 - alpha) * Rw[i];
    }
    Qw[L - 1] = Rw[0];
    Qw[k + r - j - s - 1] = Rw[p - j - s];
  }

  // insert knots
  for (std::size_t i = 0; i < r; ++i) {
    knots.insert(t);
  }

  // copy new control points
  P.clear();

  // transform back to euclidian space
  for (typename std::vector<point_t>::iterator i = Qw.begin(); i != Qw.end();
       ++i) {
    P.push_back((*i).as_euclidian());
  }
}

}  // namespace tml
