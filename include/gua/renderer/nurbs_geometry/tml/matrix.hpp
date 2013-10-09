/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : matrix.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_MATRIX_HPP
#define TML_MATRIX_HPP

// header, system
#include <boost/array.hpp>
#include <vector>
#include <set>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

namespace tml {

template <typename value_t, unsigned NROWS, unsigned NCOLS> class matrix {
 public:  // typedefs

  typedef value_t value_type;
  typedef point<value_t, NROWS> point_type;
  typedef boost::array<value_type, NCOLS> row_type;
  typedef boost::array<value_type, NROWS> col_type;
  typedef boost::array<row_type, NROWS> storage_type;

 public:  // ctor/dtor

  matrix();

  matrix(matrix const& bb);

  template <typename iterator_t> matrix(iterator_t begin, iterator_t end);

  ~matrix();

 public:  // methods

  row_type& operator[](std::size_t row);
  row_type const& operator[](std::size_t row) const;

  void operator/=(value_type const& s);

  value_type determinant() const;

  void validate(value_t const& eps);

  std::set<value_t> eigenvalues(value_t const& min,
                                value_t const& max,
                                value_t const& tolerance) const;

  std::vector<point_type> eigenvectors(value_t const& min,
                                       value_t const& max,
                                       value_t const& eps) const;

  matrix inverse() const;

  matrix<value_t, NCOLS, NROWS> transpose() const;

  row_type const& row(std::size_t r) const;
  col_type col(std::size_t c) const;

  virtual void print(std::ostream& os) const;
  virtual void write(std::ostream& os) const;
  virtual void read(std::istream& is);

 private:  // attributes

  storage_type _data;

};

template <typename value_t, unsigned N>
value_t compute_determinant(matrix<value_t, N, N> const&);

template <typename value_t>
value_t compute_determinant(matrix<value_t, 2, 2> const&);

template <typename value_t>
value_t compute_determinant(matrix<value_t, 3, 3> const&);

template <typename value_t, unsigned NROWS, unsigned NCOLS>
matrix<value_t, NROWS, NCOLS> compute_inverse(
    matrix<value_t, NROWS, NCOLS> const& rhs);

template <typename value_t, unsigned N>
matrix<value_t, N, N> compute_inverse(matrix<value_t, N, N> const& rhs);

template <typename value_t>
matrix<value_t, 3, 3> compute_inverse(matrix<value_t, 3, 3> const& rhs);

template <typename value_t>
matrix<value_t, 3, 3> compute_inverse(matrix<value_t, 3, 3> const& rhs);

template <typename value_t>
matrix<value_t, 3, 3> make_rotation_x(value_t const& arc);

template <typename value_t>
matrix<value_t, 3, 3> make_rotation_y(value_t const& arc);

template <typename value_t>
matrix<value_t, 3, 3> make_rotation_z(value_t const& arc);

template <typename value_t, unsigned N>
std::set<value_t> compute_eigenvalues(matrix<value_t, N, N> const& m,
                                      value_t const& min,
                                      value_t const& max,
                                      value_t const& tolerance);

template <typename value_t, unsigned N>
std::vector<point<value_t, N> > compute_eigenvectors(
    matrix<value_t, N, N> const&,
    value_t const& min,
    value_t const& max,
    value_t const& eps);

template <typename value_t, unsigned NROWS, unsigned NCOLS>
matrix<value_t, NROWS - 1, NCOLS - 1> generate_submatrix(
    matrix<value_t, NROWS, NCOLS> const& m,
    unsigned row,
    unsigned col);

template <typename value_t>
matrix<value_t, 1, 1> generate_submatrix(matrix<value_t, 1, 1> const& m,
                                         unsigned row,
                                         unsigned col);

template <typename value_t, unsigned NROWS, unsigned NCOLS>
    bool operator==(matrix<value_t, NROWS, NCOLS> const& lhs,
                    matrix<value_t, NROWS, NCOLS> const& rhs);

template <typename value_t, unsigned NROWS, unsigned NCOLS>
    bool operator!=(matrix<value_t, NROWS, NCOLS> const& lhs,
                    matrix<value_t, NROWS, NCOLS> const& rhs);

template <typename point_t, typename value_t, unsigned NROWS, unsigned NCOLS>
point_t operator*(matrix<value_t, NROWS, NCOLS> const& lhs, point_t const& rhs);

template <typename value_t, unsigned NROWS, unsigned NCOLS>
matrix<value_t, NROWS, NCOLS> operator*(
    matrix<value_t, NROWS, NCOLS> const& lhs,
    value_t const& rhs);

template <typename value_t, unsigned NROWS, unsigned NCOLS>
matrix<value_t, NROWS, NCOLS> operator*(
    value_t const& lhs,
    matrix<value_t, NROWS, NCOLS> const& rhs);

template <typename value_t, unsigned NROWS, unsigned NCOLS>
    matrix<value_t,
           NROWS,
           NCOLS> operator-(matrix<value_t, NROWS, NCOLS> const& lhs,
                            matrix<value_t, NROWS, NCOLS> const& rhs);

template <typename value_t,
          unsigned NROWS_A,
          unsigned NCOLS_A,
          unsigned NROWS_B,
          unsigned NCOLS_B>
matrix<value_t, NROWS_A, NCOLS_B> operator*(
    matrix<value_t, NROWS_A, NCOLS_A> const& lhs,
    matrix<value_t, NROWS_B, NCOLS_B> const& rhs);

template <typename value_t, unsigned NROWS, unsigned NCOLS>
    std::ostream& operator<<(std::ostream& os,
                             matrix<value_t, NROWS, NCOLS> const& a);

typedef matrix<float, 4, 4> matrix4f;
typedef matrix<float, 3, 3> matrix3f;

}  // namespace tml

#include <gua/renderer/nurbs_geometry/tml/matrix_impl.hpp>

#endif  // TML_MATRIX_HPP
