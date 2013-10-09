/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : point.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_POINT_HPP
#define TML_POINT_HPP

// header, system
#include <iosfwd>
#include <cassert>

#include <boost/array.hpp>

namespace tml {

template <typename value_t, unsigned SIZE> class point {

 public:  // enums and typedefs

  typedef value_t value_type;
  typedef std::size_t coordinate_type;

  // euclidian space
  enum {
    x = 0
  };
  enum {
    y = 1
  };
  enum {
    z = 2
  };

  enum {
    u = 0
  };
  enum {
    v = 1
  };
  enum {
    w = 2
  };

  static const unsigned coordinates = SIZE;
  static const unsigned MEMSIZE = SIZE + 1;

 public:  // c'tor / d'tor

  point();
  explicit point(value_type const&);
  explicit point(value_type const&, value_type const&);
  explicit point(value_type const&, value_type const&, value_type const&);
  explicit point(value_type const&,
                 value_type const&,
                 value_type const&,
                 value_type const&);

  template <typename container_t> point(container_t const& container);

  point(point const& rhs);

  template <typename iterator_t>
  void set(iterator_t coord_begin, iterator_t coord_end);

  ~point();

 public:  // operators

  value_t const& operator[](coordinate_type const& axis) const;
  value_t& operator[](coordinate_type const& axis);

  point& operator=(point const& rhs);

  point& operator*=(value_type const& scalar);
  point& operator/=(value_type const& scalar);
  point& operator-=(point const& rhs);
  point& operator+=(point const& rhs);

  point operator-() const;

 public:  // methods

  void swap(point& rhs);

  value_type const& weight() const;
  void weight(value_type const& weight);

  void print(std::ostream& os) const;

  point as_homogenous() const;  // point in hyperspace P[x,y,z,...,w] ->
                                // P[wx,wy,wz,..,w]
  point as_euclidian() const;   // point in euclidian P[wx,wy,wz,...,w] ->
                                // P[x,y,z,...,w]

  void project_to_homogenous();
  void project_to_euclidian();

  // euclidian absolute value
  value_type const abs() const;
  void normalize();
  value_type distance(point const& other) const;

  static point maximum();
  static point minimum();

  static point set(value_type const& v);

 private:  // data members

  boost::array<value_t, MEMSIZE> _data;
};

template <typename value_t, unsigned SIZE>
    bool operator==(point<value_t, SIZE> const& lhs,
                    point<value_t, SIZE> const& rhs);

template <typename value_t, unsigned SIZE>
    bool operator!=(point<value_t, SIZE> const& lhs,
                    point<value_t, SIZE> const& rhs);

template <typename value_t, unsigned SIZE>
    std::ostream& operator<<(std::ostream& os, point<value_t, SIZE> const& rhs);

template <typename value_t, unsigned SIZE, typename scalar_t>
point<value_t, SIZE> operator*(point<value_t, SIZE> const& lhs,
                               scalar_t const& rhs);

template <typename value_t, unsigned SIZE>
point<value_t, SIZE> operator*(value_t const& lhs,
                               point<value_t, SIZE> const& rhs);

template <typename value_t, unsigned SIZE, typename scalar_t>
    point<value_t, SIZE> operator/(point<value_t, SIZE> const& lhs,
                                   scalar_t const& rhs);

template <typename value_t, unsigned SIZE>
    point<value_t, SIZE> operator/(point<value_t, SIZE> const& lhs,
                                   point<value_t, SIZE> const& rhs);

template <typename value_t, unsigned SIZE>
    point<value_t, SIZE> operator+(point<value_t, SIZE> const& lhs,
                                   point<value_t, SIZE> const& rhs);

template <typename value_t, unsigned SIZE>
    point<value_t, SIZE> operator-(point<value_t, SIZE> const& lhs,
                                   point<value_t, SIZE> const& rhs);

template <typename value_t, unsigned SIZE>
point<value_t, SIZE> elementwise_min(point<value_t, SIZE> const& a,
                                     point<value_t, SIZE> const& b);

template <typename value_t, unsigned SIZE>
point<value_t, SIZE> elementwise_max(point<value_t, SIZE> const& a,
                                     point<value_t, SIZE> const& b);

template <typename value_t, unsigned SIZE>
value_t dot(point<value_t, SIZE> const& a, point<value_t, SIZE> const& b);

template <typename value_t, unsigned SIZE>
point<value_t, SIZE> cross(point<value_t, SIZE> const& a,
                           point<value_t, SIZE> const& b);

template <typename point_t>
bool greater_by_length(point_t const& lhs, point_t const& rhs);

template <typename point_t>
bool less_by_length(point_t const& lhs, point_t const& rhs);

typedef point<int, 2> point2i;
typedef point<int, 3> point3i;
typedef point<float, 2> point2f;
typedef point<float, 3> point3f;
typedef point<double, 2> point2d;
typedef point<double, 3> point3d;

}  // namespace tml

#include "implementation/point_impl.hpp"

#endif  // TML_POINT_HPP
