/********************************************************************************
*
* Copyright (C) 2013 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : line.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_LINE_HPP
#define TML_LINE_HPP

#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

namespace tml {

template <typename value_t> class line {
 public:

  typedef point<value_t, 2> point_t;

 public:

  line(point_t const& a, point_t const& b) : start_(a), end_(b) {}

  ~line() {}

  bool intersect(line const& b, point_t& intersect) const {
    value_t ax = start_.x;
    value_t bx = b.start().x;

    value_t ay = start_.y;
    value_t by = b.start().y;

    value_t d_ax = end_.x - start_.x;
    value_t d_ay = end_.y - start_.y;

    value_t d_bx = b.end().x - b.start().x;
    value_t d_by = b.end().y - b.start().y;

    value_t s =
        ((ax - bx) * d_ay + (by - ay) * d_ax) / (d_bx * d_ay - d_by * d_ax);
    value_t t =
        ((ax - bx) * d_by + (by - ay) * d_bx) / (d_ay * d_bx - d_ax * d_by);

    if (s > 0 && s < 1 && t > 0 && t < 1) {
      intersect.x = bx + s * d_bx;
      intersect.y = by + s * d_by;

      return true;
    } else {
      return false;
    }
  }

  bool is_vertical() const { return start_.x == end_.x; }

  bool is_horizontal() const { return start_.y == end_.y; }

  point_t const& start() const { return start_; }

  point_t const& end() const { return end_; }

  void print(std::ostream& os) const {
    if (start_.x < end_.x || (start_.x == end_.x && start_.y < end_.y)) {
      os << "line from " << start_.x << ", " << start_.y << " to " << end_.x
         << ", " << end_.y << std::endl;
    } else {
      os << "line from " << end_.x << ", " << end_.y << " to " << start_.x
         << ", " << start_.y << std::endl;
    }
  }

 private:

  point_t start_;
  point_t end_;
};

template <typename value_t>
    std::ostream& operator<<(std::ostream& os, line<value_t> const& l) {
  l.print(os);
  return os;
}

}  // namespace tml

#endif  //TML_LINE_HPP
