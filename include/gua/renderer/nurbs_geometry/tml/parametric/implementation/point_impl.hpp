/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : point_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
// header, system
#include <cassert>  // assert
#include <vector>
#include <cmath>
#include <limits>

#include <boost/foreach.hpp>
#include <boost/numeric/conversion/bounds.hpp>

#if WIN32
#pragma warning(disable : 4996)
#endif

namespace tml {

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE>::point()
    : _data() {
  _data[SIZE] = 1;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE>::point(value_type const& x)
    : _data() {
  for (unsigned i = 0; i != MEMSIZE; ++i) {
    _data[i] = x;
  }
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE>::point(value_type const& x, value_type const& y)
    : _data() {
  switch (SIZE) {
    case 1:
      _data[0] = x;
      _data[1] = y;
      break;
    case 2:
      _data[0] = x;
      _data[1] = y;
      _data[2] = 1;
      break;
    default:
      throw std::runtime_error(
          "Wrong number of arguments in point<value_t, size>::point(value_type "
          "const& a, value_type const& b)");
  }
  ;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE>::point(value_type const& x,
                                   value_type const& y,
                                   value_type const& z)
    : _data() {
  switch (SIZE) {
    case 2:
      _data[0] = x;
      _data[1] = y;
      _data[2] = z;
      break;
    case 3:
      _data[0] = x;
      _data[1] = y;
      _data[2] = z;
      _data[3] = 1;
      break;
    default:
      throw std::runtime_error(
          "Wrong number of arguments in point<value_t, size>::point(value_type "
          "const& a, value_type const& b, value_type const& c)");
  }
  ;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE>::point(value_type const& a,
                                   value_type const& b,
                                   value_type const& c,
                                   value_type const& d)
    : _data() {
  switch (SIZE) {
    case 3:
      _data[0] = a;
      _data[1] = b;
      _data[2] = c;
      _data[3] = d;
      break;
    case 4:
      _data[0] = a;
      _data[1] = b;
      _data[2] = c;
      _data[3] = d;
      _data[4] = 1;
      break;
    default:
      throw std::runtime_error(
          "Wrong number of arguments in point<value_t, size>::point(value_type "
          "const& a, value_type const& b, value_type const& c, value_type "
          "const& "
          "d)");
  }
  ;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
template <typename container_t>
inline point<value_t, SIZE>::point(container_t const& c)
    : _data() {
  for (unsigned i = 0; i != SIZE; ++i) {
    _data[i] = value_type(c[i]);
  }
  _data[SIZE] = 1;  // weight = 1
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE>::point(point const& rhs)
    : _data(rhs._data) {}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
template <typename iterator_t>
inline void point<value_t, SIZE>::set(iterator_t begin, iterator_t end) {
  typedef typename std::iterator_traits<iterator_t>::value_type
      iterator_value_type;
  assert(sizeof(value_type) == sizeof(iterator_value_type));
  std::copy(begin, end, _data.begin());
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE> point<value_t, SIZE>::~point() {}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline value_t const& point<value_t, SIZE>::operator[](
    coordinate_type const& c) const {
  assert(c <= SIZE);
  return _data[c];
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline value_t& point<value_t, SIZE>::operator[](coordinate_type const& c) {
  assert(c <= SIZE);
  return _data[c];
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE>& point<value_t, SIZE>::operator=(point const& rhs) {
  point<value_t, SIZE> tmp(rhs);
  swap(tmp);
  return *this;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE>& point<value_t, SIZE>::operator*=(
    value_type const& scalar) {
  for (unsigned i = 0; i != MEMSIZE; ++i) {
    _data[i] *= scalar;
  }

  return *this;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE>& point<value_t, SIZE>::operator/=(
    value_type const& scalar) {
  for (unsigned i = 0; i != MEMSIZE; ++i) {
    _data[i] /= scalar;
  }

  return *this;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE>& point<value_t, SIZE>::operator-=(
    point const& rhs) {
  for (unsigned i = 0; i != MEMSIZE; ++i) {
    _data[i] -= rhs._data[i];
  }

  return *this;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE>& point<value_t, SIZE>::operator+=(
    point const& rhs) {
  for (unsigned i = 0; i != MEMSIZE; ++i) {
    _data[i] += rhs._data[i];
  }

  return *this;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
    point<value_t, SIZE> point<value_t, SIZE>::operator-() const {
  point neg;

  for (unsigned i = 0; i != SIZE; ++i) {
    neg._data[i] = -_data[i];
  }
  neg._data[SIZE] = _data[SIZE];

  return neg;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline void point<value_t, SIZE>::swap(point& rhs) {
  _data.swap(rhs._data);
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline value_t const& point<value_t, SIZE>::weight() const {
  return _data[SIZE];
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline void point<value_t, SIZE>::weight(value_t const& weight) {
  _data[SIZE] = weight;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline void point<value_t, SIZE>::print(std::ostream& os) const {
  os << "[ ";
  for (unsigned i = 0; i != MEMSIZE; ++i) {
    os << _data[i] << " ";
  }
  os << "]";
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE> point<value_t, SIZE>::as_homogenous() const {
  // make a copy of point
  point<value_t, SIZE> homogen(*this);

  // project to hyperspace
  homogen *= _data[SIZE];

  // reset weight to original
  homogen.weight(_data[SIZE]);

  return homogen;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE> point<value_t, SIZE>::as_euclidian() const {
  // make a copy of point
  point<value_t, SIZE> euclid(*this);

  // project back to euclidian space
  euclid /= _data[SIZE];

  // reset weight to original
  euclid.weight(_data[SIZE]);

  return euclid;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline void point<value_t, SIZE>::project_to_homogenous() {
  for (unsigned i = 0; i != SIZE; ++i) {
    _data[i] *= _data[SIZE];
  }
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline void point<value_t, SIZE>::project_to_euclidian() {
  for (unsigned i = 0; i != SIZE; ++i) {
    _data[i] /= _data[SIZE];
  }
  // _weight = _weight;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
value_t const point<value_t, SIZE>::abs() const {
  value_t squared = 0;

  for (std::size_t i = 0; i != SIZE; ++i) {
    squared += _data[i] * _data[i];
  }

  return std::sqrt(squared);
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
void point<value_t, SIZE>::normalize() {
  value_type absolut_value = abs();

  for (std::size_t i = 0; i != SIZE; ++i) {
    _data[i] /= absolut_value;
  }
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
value_t point<value_t, SIZE>::distance(point const& other) const {
  point tmp = (*this) - other;
  return tmp.abs();
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline /* static */ point<value_t, SIZE> point<value_t, SIZE>::maximum() {
  std::vector<value_t> data(MEMSIZE, std::numeric_limits<value_t>::max());
  return point<value_t, SIZE>(data);
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline /* static */ point<value_t, SIZE> point<value_t, SIZE>::minimum() {
  std::vector<value_t> data(MEMSIZE, -std::numeric_limits<value_t>::max());
  return point<value_t, SIZE>(data);
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline /* static */ point<value_t, SIZE> point<value_t, SIZE>::set(
    value_t const& v) {
  point<value_t, SIZE> p;

  for (std::size_t i = 0; i != point<value_t, SIZE>::MEMSIZE; ++i) {
    p[i] = v;
  }

  return p;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
    inline bool operator==(point<value_t, SIZE> const& lhs,
                           point<value_t, SIZE> const& rhs) {
  bool equal = true;

  for (std::size_t i = 0; i != point<value_t, SIZE>::MEMSIZE; ++i) {
    equal &= (lhs[i] == rhs[i]);
  }

  return equal;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
    inline bool operator!=(point<value_t, SIZE> const& lhs,
                           point<value_t, SIZE> const& rhs) {
  return !(lhs == rhs);
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
    inline point<value_t, SIZE> operator-(point<value_t, SIZE> const& lhs,
                                          point<value_t, SIZE> const& rhs) {
  point<value_t, SIZE> tmp(lhs);
  tmp -= rhs;
  return tmp;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
    inline point<value_t, SIZE> operator+(point<value_t, SIZE> const& lhs,
                                          point<value_t, SIZE> const& rhs) {
  point<value_t, SIZE> tmp(lhs);
  tmp += rhs;
  return tmp;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
    inline std::ostream& operator<<(std::ostream& os,
                                    point<value_t, SIZE> const& rhs) {
  rhs.print(os);
  return os;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE, typename scalar_t>
inline point<value_t, SIZE> operator*(point<value_t, SIZE> const& lhs,
                                      scalar_t const& rhs) {
  point<value_t, SIZE> tmp(lhs);
  tmp *= rhs;
  return tmp;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE> operator*(value_t const& lhs,
                                      point<value_t, SIZE> const& rhs) {
  point<value_t, SIZE> tmp(rhs);
  tmp *= lhs;
  return tmp;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE, typename scalar_t>
inline point<value_t, SIZE> operator/(point<value_t, SIZE> const& lhs,
                                      scalar_t const& rhs) {
  point<value_t, SIZE> tmp(lhs);
  tmp /= rhs;
  return tmp;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
point<value_t, SIZE> operator/(point<value_t, SIZE> const& lhs,
                               point<value_t, SIZE> const& rhs) {
  point<value_t, SIZE> tmp(lhs);

  for (std::size_t i = 0; i != point<value_t, SIZE>::MEMSIZE; ++i) {
    tmp[i] /= rhs[i];
  }

  return tmp;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE, typename scalar_t>
    inline point<value_t, SIZE> operator+(point<value_t, SIZE> const& lhs,
                                          point<value_t, SIZE> const& rhs) {
  point<value_t, SIZE> tmp(lhs);
  tmp += rhs;
  return tmp;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE, typename scalar_t>
    inline point<value_t, SIZE> operator-(point<value_t, SIZE> const& lhs,
                                          point<value_t, SIZE> const& rhs) {
  point<value_t, SIZE> tmp(lhs);
  tmp -= rhs;
  return tmp;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE> elementwise_min(point<value_t, SIZE> const& a,
                                            point<value_t, SIZE> const& b) {
  point<value_t, SIZE> pmin;

  for (std::size_t i = 0; i != point<value_t, SIZE>::MEMSIZE; ++i) {
    pmin[i] = std::min(a[i], b[i]);
  }

  return pmin;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline point<value_t, SIZE> elementwise_max(point<value_t, SIZE> const& a,
                                            point<value_t, SIZE> const& b) {
  point<value_t, SIZE> pmax;

  for (std::size_t i = 0; i != point<value_t, SIZE>::MEMSIZE; ++i) {
    pmax[i] = std::max(a[i], b[i]);
  }

  return pmax;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
inline value_t dot(point<value_t, SIZE> const& a,
                   point<value_t, SIZE> const& b) {
  value_t dotprod = 0;

  for (std::size_t i = 0; i != SIZE; ++i) {
    dotprod += a[i] * b[i];
  }

  return dotprod;
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned SIZE>
point<value_t, SIZE> cross(point<value_t, SIZE> const& a,
                           point<value_t, SIZE> const& b) {
  throw std::runtime_error("not implemented yet");
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t>
point<value_t, 3> cross(point<value_t, 2> const& a,
                        point<value_t, 2> const& b) {
  return point<value_t, 3>(0, 0, 1);
}

/////////////////////////////////////////////////////////////////////////////////
template <typename value_t>
point<value_t, 3> cross(point<value_t, 3> const& a,
                        point<value_t, 3> const& b) {
  return point<value_t, 3>(a[1] * b[2] - a[2] * b[1],
                           a[2] * b[0] - a[0] * b[2],
                           a[0] * b[1] - a[1] * b[0]);
}

/////////////////////////////////////////////////////////////////////////////////
template <typename point_t>
bool greater_by_length(point_t const& lhs, point_t const& rhs) {
  return lhs.length() > rhs.length();
}

/////////////////////////////////////////////////////////////////////////////////
template <typename point_t>
bool less_by_length(point_t const& lhs, point_t const& rhs) {
  return lhs.length() < rhs.length();
}

}  // namespace tml
