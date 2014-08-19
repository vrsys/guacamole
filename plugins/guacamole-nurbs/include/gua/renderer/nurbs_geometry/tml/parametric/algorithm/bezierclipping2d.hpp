/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : bezierclipping2d.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_BEZIERCLIPPING2D_HPP
#define TML_BEZIERCLIPPING2D_HPP

// header, system
#include <set>      // std::set
#include <vector>   // std::vector
#include <utility>  // std::pair

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/beziercurve.hpp>
#include <gua/renderer/nurbs_geometry/tml/interval.hpp>

namespace tml {

template <typename point2_t> class bezierclipping2d {
 public:  // enums, typedefs

  enum classification {
    even,
    odd,
    unknown
  };

  typedef beziercurve<point2_t> curve_t;

  typedef typename point2_t::value_type value_type;

  typedef typename std::vector<curve_t>::iterator curve_iterator;
  typedef typename std::vector<curve_t>::const_iterator const_curve_iterator;
  typedef typename std::vector<point2_t>::iterator point_iterator;
  typedef typename std::vector<point2_t>::const_iterator const_point_iterator;

  typedef std::pair<curve_iterator, curve_iterator> curve_range;
  typedef std::pair<const_curve_iterator, const_curve_iterator>
      const_curve_range;

 public:

  inline bezierclipping2d(value_type const& min_contraction = 0.9)
      : _min_contraction(min_contraction) {}

  //////////////////////////////////////////////////////////////////////////////
  // split bezier curve at partial inflection points of coord
  // parameter :
  //   contraction : should be between 0.1 and 0.8
  //   epsilon : numerical error, should be > 1 * 10^-6
  //////////////////////////////////////////////////////////////////////////////
  void split(curve_t const& bc,
             unsigned def,
             unsigned val,
             value_type contraction,
             value_type epsilon,
             std::vector<curve_t>& result) {
    curve_t hodograph;

    if (bc.is_rational()) {
      hodograph = bc.scaled_hodograph().nishita(val);
    } else {
      hodograph = bc.hodograph().nishita(val);
    }

    std::set<value_type> roots;

    // check curve on linearity on value P[i][val] constant for all i
    if (!bc.is_constant(val, epsilon)) {
      intersect(hodograph,
                roots,
                interval<value_type>(0, 1),
                contraction,
                epsilon,
                def,
                val);
    }

    // insert limits
    roots.insert(0);
    roots.insert(1);

    typename std::set<value_type>::iterator i = roots.begin();
    typename std::set<value_type>::iterator j(i);
    std::advance(j, 1);

    // push parts back on stack
    for (; j != roots.end(); ++i, ++j) {
      curve_t tmp(bc);
      tmpclip_lr(*i, *j);
      result.push_back(tmp);
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // bezier clipping
  //   curve - original curve itself
  //   roots - found roots
  //   min, max - chull intersections found guaranteed to be non-intersecting
  //   contraction - contraction limit. if contraction is lower -> curve split
  //   epsilon - if abs(max-min) < epsilon, it is assumed to be an intersection
  //   def, val - bezier clipping done for a definition range and value range
  //////////////////////////////////////////////////////////////////////////////
  void intersect(curve_t const& curve,
                 std::set<value_type>& roots,
                 tml::interval<value_type> const& rng,
                 value_type contraction,
                 value_type epsilon,
                 unsigned def,
                 unsigned val) {
    value_type min = rng.minimum();
    value_type max = rng.maximum();

    // found intersection
    if (max - min <= epsilon) {
      roots.insert((max + min) / 2.0);
    } else {

      curve_t tmp(curve);
      tmpclip_lr(min, max);

      value_type t1, t2;
      tmp.intersect_chull(def, val, t1, t2);
      bool isects(t1 <= t2);

      // is intersecting
      if (isects) {

        value_type isect_rng(t2 - t1);
        bool contracts = (isect_rng / (max - min)) < contraction;

        // is still contracting
        if (contracts) {
          intersect(curve,
                    roots,
                    tml::interval<value_type>(t1, t2),
                    contraction,
                    epsilon,
                    def,
                    val);
        } else {
          value_type mid = (t1 + t2) / 2.0;

          // restart for left part
          intersect(curve,
                    roots,
                    tml::interval<value_type>(t1, mid),
                    contraction,
                    epsilon,
                    def,
                    val);

          // restart with right part
          intersect(curve,
                    roots,
                    tml::interval<value_type>(mid, t2),
                    contraction,
                    epsilon,
                    def,
                    val);
        }
      } else {
        // does not intersect, discard curve
      }
    }
  }

  classification classify(curve_t const& curve,
                          point2_t const& ray_origin,
                          std::size_t ray_direction) {
    // classification after nishita'90
    //  -----------------
    //  |___Q2___|__Q1__|
    //  |   Q3   |  Q4  |
    //  -----------------
    ///////////////////////////////////

    bool Q1(false), Q2(false), Q3(false), Q4(false);

    // issue points to quadrants
    for (const_point_iterator p = curve.begin(); p != curve.end(); ++p) {
      if ((*p)[point2_t::u] > ray_origin[point2_t::u] &&
          (*p)[point2_t::v] > ray_origin[point2_t::v]) {
        Q1 = true;
      }

      if ((*p)[point2_t::u] < ray_origin[point2_t::u] &&
          (*p)[point2_t::v] > ray_origin[point2_t::v]) {
        Q2 = true;
      }

      if ((*p)[point2_t::u] < ray_origin[point2_t::u] &&
          (*p)[point2_t::v] < ray_origin[point2_t::v]) {
        Q3 = true;
      }

      if ((*p)[point2_t::u] > ray_origin[point2_t::u] &&
          (*p)[point2_t::v] < ray_origin[point2_t::v]) {
        Q4 = true;
      }
    }

    // switch quadrants if in positive y-direction
    if (ray_direction == point2_t::v) {
      std::swap(Q1, Q3);
      std::swap(Q2, Q4);
    }

    // case A : no intersection
    if (Q1 && !Q2 && !Q3 && !Q4)
      return even;
    if (!Q1 && Q2 && !Q3 && !Q4)
      return even;
    if (!Q1 && !Q2 && Q3 && !Q4)
      return even;
    if (!Q1 && !Q2 && !Q3 && Q4)
      return even;

    if (!Q1 && !Q4)
      return even;
    if (!Q1 && !Q2)
      return even;
    if (!Q3 && !Q4)
      return even;

    // case B : quadrants 1 and 4
    if (Q1 && !Q2 && !Q3 && Q4) {
      // both on same side -> even number of intersections
      if ((curve.front()[point2_t::v] > ray_origin[point2_t::v] &&
           curve.back()[point2_t::v] > ray_origin[point2_t::v]) ||
          (curve.front()[point2_t::v] > ray_origin[point2_t::v] &&
           curve.back()[point2_t::v] > ray_origin[point2_t::v])) {
        return even;
      } else {
        return odd;
      }
    }

    // unknown combination -> clip
    return unknown;
  }

  //////////////////////////////////////////////////////////////////////////////
  inline bool chull2d_intersect(curve_t const& c,
                                unsigned const_axis,
                                value_type& t1,
                                value_type& t2) {
    assert(const_axis == point2_t::u || const_axis == point2_t::v);

    bool intersects = false;
    unsigned val = (const_axis + 1) % 2;

    t1 = std::numeric_limits<value_type>::max();
    t2 = -std::numeric_limits<value_type>::max();

    for (typename curve_t::const_point_iterator i = c.begin(); i != c.end();
         ++i) {
      for (typename curve_t::const_point_iterator j = i; j != c.end(); ++j) {
        if (((*i)[const_axis] > 0 && (*j)[const_axis] < 0) ||
            ((*i)[const_axis] < 0 && (*j)[const_axis] > 0)) {
          value_type dy = (*j)[const_axis] - (*i)[const_axis];
          value_type dx = (*j)[val] - (*i)[val];
          value_type y0 = (*i)[const_axis] - (dy / dx) * (*i)[val];
          value_type x0 = (-y0 * dx) / dy;

          t1 = std::max(0.0, std::min(t1, x0));
          t2 = std::min(1.0, std::max(t2, x0));

          intersects = true;
        }
      }
    }

    return intersects;
  }

  //////////////////////////////////////////////////////////////////////////////
  // determines whether there is an intersection between a ray and a curve or
  // not
  //////////////////////////////////////////////////////////////////////////////
  bool intersects_odd_times(curve_t const& curve,
                            point2_t const& ray_origin,
                            std::size_t ray_direction,
                            std::size_t max_iters,
                            std::size_t& iters) {
    iters = 0;
    std::size_t const_raycoord;
    switch (ray_direction) {
      case point2_t::u:
        const_raycoord = point2_t::v;
        break;
      case point2_t::v:
        const_raycoord = point2_t::u;
        break;
    }

    // classify points of curve
    bool result(false);

    std::vector<curve_t> stack;
    stack.push_back(curve);

    while (!stack.empty() && iters < max_iters) {
      std::vector<curve_t> new_stack;

      for (const_curve_iterator i = stack.begin(); i != stack.end(); ++i) {
        classification cls = classify(*i, ray_origin, ray_direction);

        // decide further proceeding
        switch (cls) {

          // case A
          case even:  // number of intersections or no intersection
            result ^= false;
            break;
          // case B
          case odd:  // number of intersections
            result ^= true;
            break;
          // case C
          case unknown:  // number of intersections
            ++iters;

            // find out clipping plane
            value_type dx =
                std::fabs(i->front()[point2_t::u] - ray_origin[point2_t::u]) +
                std::fabs(i->back()[point2_t::u] - ray_origin[point2_t::u]);
            value_type dy =
                std::fabs(i->front()[point2_t::v] - ray_origin[point2_t::v]) +
                std::fabs(i->back()[point2_t::v] - ray_origin[point2_t::v]);

            if (const_raycoord == point2_t::u) {
              std::swap(dx, dy);
            }

            value_type t1, t2;
            bool intersects = false;

            curve_t nsh(*i);
            nsh.translate(point2_t(
                -ray_origin[point2_t::u], -ray_origin[point2_t::v], 0.0));

            if (dx < dy) {  // case C1
              switch (const_raycoord) {
                case point2_t::v:
                  nsh.nishita(point2_t::u);
                  break;
                case point2_t::u:
                  nsh.nishita(point2_t::v);
                  break;
              }

              chull2d_intersect(nsh, unsigned(const_raycoord), t1, t2);

            } else {  // case C2
              switch (const_raycoord) {
                case point2_t::v:
                  nsh.nishita(point2_t::v);
                  break;
                case point2_t::u:
                  nsh.nishita(point2_t::u);
                  break;
              }

              chull2d_intersect(nsh, unsigned(ray_direction), t1, t2);
            }

            if (intersects && (t1 / t2) > _min_contraction) {
              curve_t left(*i);
              curve_t center(*i);
              curve_t right(*i);

              left.clip_right(t1);
              center.clip_lr(t1, t2);
              right.clip_left(t2);

              new_stack.push_back(center);
              new_stack.push_back(left);
              new_stack.push_back(right);
            } else {
              curve_t left;
              curve_t right;

              i->split(0.5, left, right);

              new_stack.push_back(left);
              new_stack.push_back(right);
            }
            break;
        }
      }
      std::swap(stack, new_stack);
    }

    if (iters > max_iters) {
      return false;
    } else {
      return result;
    }
  }

 private:  // members

  value_type _min_contraction;

};

}  // namespace tml

#endif  // TML_BEZIERCLIPPING_HPP
