/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : matrix_impl.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_MATRIX_IMPL_HPP
#define TML_MATRIX_IMPL_HPP

// header, system
#include <boost/numeric/conversion/bounds.hpp>
#include <boost/foreach.hpp>
#include <set>
#include <map>
#include <iostream>

// header, project
#include <gua/renderer/nurbs_geometry/tml/polynomial.hpp>

namespace tml {

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
matrix<value_t, NROWS, NCOLS>::matrix()
    : _data() {
  if (NCOLS == NROWS) {
    for (std::size_t r = 0; r != NROWS; ++r) {
      for (std::size_t c = 0; c != NCOLS; ++c) {
        _data[r][c] = (r == c) ? value_t(1) : value_t(0);
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
matrix<value_t, NROWS, NCOLS>::matrix(matrix const& m)
    : _data(m._data) {}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
template <typename iterator_t>
matrix<value_t, NROWS, NCOLS>::matrix(iterator_t begin, iterator_t end) {
  for (std::size_t r = 0; r != NROWS; ++r) {
    for (std::size_t c = 0; c != NCOLS; ++c) {
      _data[r][c] = *begin++;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
matrix<value_t, NROWS, NCOLS>::~matrix() {}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
typename matrix<value_t, NROWS, NCOLS>::row_type&
matrix<value_t, NROWS, NCOLS>::operator[](std::size_t row) {
  assert(row <= NROWS);
  return _data[row];
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
typename matrix<value_t, NROWS, NCOLS>::row_type const&
matrix<value_t, NROWS, NCOLS>::operator[](std::size_t row) const {
  assert(row <= NROWS);
  return _data[row];
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
void matrix<value_t, NROWS, NCOLS>::operator/=(value_type const& s) {
  for (std::size_t r = 0; r != NROWS; ++r) {
    for (std::size_t c = 0; c != NCOLS; ++c) {
      _data[r][c] /= s;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
typename matrix<value_t, NROWS, NCOLS>::value_type
matrix<value_t, NROWS, NCOLS>::determinant() const {
  if (NCOLS == NROWS) {
    return compute_determinant(*this);
  } else {
    throw std::runtime_error("Not implemented yet");
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
void matrix<value_t, NROWS, NCOLS>::validate(value_t const& eps) {
  for (std::size_t r = 0; r != NROWS; ++r) {
    for (std::size_t c = 0; c != NCOLS; ++c) {
      _data[r][c] = fabs(_data[r][c]) < eps ? 0 : _data[r][c];
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
std::set<value_t> matrix<value_t, NROWS, NCOLS>::eigenvalues(
    value_t const& min,
    value_t const& max,
    value_t const& tolerance) const {
  if (NCOLS == NROWS) {
    return compute_eigenvalues(*this, min, max, tolerance);
  } else {
    throw std::runtime_error("Not implemented yet");
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
std::vector<typename matrix<value_t, NROWS, NCOLS>::point_type>
matrix<value_t, NROWS, NCOLS>::eigenvectors(value_t const& min,
                                            value_t const& max,
                                            value_t const& eps) const {
  if (NCOLS == NROWS) {
    return compute_eigenvectors(*this, min, max, eps);
  } else {
    throw std::runtime_error("Not implemented yet");
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
/* virtual */ void matrix<value_t, NROWS, NCOLS>::print(
    std::ostream& os) const {
  for (std::size_t r = 0; r != NROWS; ++r) {
    os << "[ ";

    for (std::size_t c = 0; c != NCOLS; ++c) {
      os << _data[r][c] << " ";
    }
    os << " ]" << std::endl;
  }
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
/* virtual */ void matrix<value_t, NROWS, NCOLS>::write(
    std::ostream& os) const {
  os.write(reinterpret_cast<char const*>(&_data.front()), sizeof(storage_type));
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
/* virtual */ void matrix<value_t, NROWS, NCOLS>::read(std::istream& is) {
  is.read(reinterpret_cast<char*>(&_data.front()), sizeof(storage_type));
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
matrix<value_t, NROWS, NCOLS> matrix<value_t, NROWS, NCOLS>::inverse() const {
  return compute_inverse(*this);
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
matrix<value_t, NCOLS, NROWS> matrix<value_t, NROWS, NCOLS>::transpose() const {
  matrix<value_t, NCOLS, NROWS> transposed;

  for (std::size_t r = 0; r != NROWS; ++r) {
    for (std::size_t c = 0; c != NCOLS; ++c) {
      transposed._data[c][r] = _data[r][c];
    }
  }

  return transposed;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
inline typename matrix<value_t, NROWS, NCOLS>::row_type const&
matrix<value_t, NROWS, NCOLS>::row(std::size_t r) const {
  assert(r < NROWS);
  return _data[r];
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
inline typename matrix<value_t, NROWS, NCOLS>::col_type
matrix<value_t, NROWS, NCOLS>::col(std::size_t c) const {
  assert(c < NCOLS);

  col_type column;

  for (std::size_t r = 0; r != NROWS; ++r) {
    column[r] = _data[r][c];
  }

  return column;
}

//////////////////////////////////////////////////////////////////////////////
// externals and partially specialized methods
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned N>
value_t compute_determinant(matrix<value_t, N, N> const& M) {
  // use generic recursive solution
  value_t det(0);
  for (unsigned r = 0; r != N; ++r) {
    for (unsigned c = 0; c != N; ++c) {
      value_t sign = ((r + c) % 2 == 1) ? value_t(-1) : value_t(1);
      det += sign * M[r][c] * compute_determinant(generate_submatrix(M, r, c));
    }
  }
  return det;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t>
value_t compute_determinant(matrix<value_t, 3, 3> const& M) {
  return value_t(M[0][0] * M[1][1] * M[2][2] + M[0][1] * M[1][2] * M[2][0] +
                 M[0][2] * M[1][0] * M[2][1] - M[0][2] * M[1][1] * M[2][0] -
                 M[0][1] * M[1][0] * M[2][2] - M[0][0] * M[1][2] * M[2][1]);
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t>
value_t compute_determinant(matrix<value_t, 2, 2> const& M) {
  return value_t(M[0][0] * M[1][1] - M[0][1] * M[1][0]);
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
matrix<value_t, NROWS, NCOLS> compute_inverse(
    matrix<value_t, NROWS, NCOLS> const& rhs) {
  throw std::runtime_error("Not implemented yet");
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned N>
matrix<value_t, N, N> compute_inverse(matrix<value_t, N, N> const& rhs) {
  throw std::runtime_error("Not implemented yet");
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t>
matrix<value_t, 3, 3> compute_inverse(matrix<value_t, 3, 3> const& rhs) {
  value_t det = rhs.determinant();

  if (det == value_t(0)) {
    //throw std::runtime_error("Matrix could not be inverted");
    std::cout << "matrix<T,N,M>::compute_inverse() : Warning, matrix could not "
                 "be inverted.\n";
  }

  matrix<value_t, 3, 3> m;

  m[0][0] = compute_determinant(generate_submatrix(rhs, 0, 0));
  m[0][1] = -compute_determinant(generate_submatrix(rhs, 1, 0));
  m[0][2] = compute_determinant(generate_submatrix(rhs, 2, 0));

  m[1][0] = -compute_determinant(generate_submatrix(rhs, 0, 1));
  m[1][1] = compute_determinant(generate_submatrix(rhs, 1, 1));
  m[1][2] = -compute_determinant(generate_submatrix(rhs, 2, 1));

  m[2][0] = compute_determinant(generate_submatrix(rhs, 0, 2));
  m[2][1] = -compute_determinant(generate_submatrix(rhs, 1, 2));
  m[2][2] = compute_determinant(generate_submatrix(rhs, 2, 2));

  return (value_t(1) / det) * m;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t>
matrix<value_t, 2, 2> compute_inverse(matrix<value_t, 2, 2> const& rhs) {
  value_t det = rhs.determinant();

  if (det == value_t(0)) {
    throw std::runtime_error("Matrix could not be inverted");
  }

  matrix<value_t, 2, 2> m;

  m[0][0] = rhs[1][1];
  m[0][1] = -rhs[0][1];
  m[1][0] = -rhs[1][0];
  m[1][1] = rhs[0][0];

  return (value_t(1) / det) * m;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t>
matrix<value_t, 3, 3> make_rotation_x(value_t const& arc) {
  matrix<value_t, 3, 3> m;

  //m[0][0] = 1;
  //m[0][1] = 0;
  //m[0][2] = 0;

  //m[1][0] = 0;
  m[1][1] = cos(arc);
  m[1][2] = -sin(arc);

  //m[2][0] = 0;
  m[2][1] = sin(arc);
  m[2][2] = cos(arc);

  return m;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t>
matrix<value_t, 3, 3> make_rotation_y(value_t const& arc) {
  matrix<value_t, 3, 3> m;

  m[0][0] = cos(arc);
  //m[0][1] = 0;
  m[0][2] = -sin(arc);

  //m[1][0] = 0;
  //m[1][1] = 1;
  //m[1][2] = 0;

  m[2][0] = sin(arc);
  //m[2][1] = 0;
  m[2][2] = cos(arc);

  return m;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t>
matrix<value_t, 3, 3> make_rotation_z(value_t const& arc) {
  matrix<value_t, 3, 3> m;

  m[0][0] = cos(arc);
  m[0][1] = sin(arc);
  //m[0][2] = 0;

  m[1][0] = -sin(arc);
  m[1][1] = cos(arc);
  //m[1][2] = 0;

  //m[2][0] = 0;
  //m[2][1] = 0;
  //m[2][2] = 1;

  return m;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned N>
std::set<value_t> compute_eigenvalues(matrix<value_t, N, N> const& M,
                                      value_t const& min,
                                      value_t const& max,
                                      value_t const& tolerance) {
  matrix<polynomial<value_t>, N, N> identity;
  matrix<polynomial<value_t>, N, N> A;

  // expand to polynomial matrix
  for (unsigned i = 0; i != N; ++i) {
    for (unsigned j = 0; j != N; ++j) {
      A[i][j] = polynomial<value_t>(M[i][j]);
    }
  }

  polynomial<value_t> lambda(value_t(0), value_t(1));
  matrix<polynomial<value_t>, N, N> lambda_identity = lambda * identity;
  matrix<polynomial<value_t>, N, N> char_equation = A - lambda_identity;
  polynomial<value_t> char_polynomial = char_equation.determinant();

  // should this polynomial be normalized?!?
  //char_polynomial.normalize();

  return char_polynomial.solve(min, max, tolerance);
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned N>
std::vector<point<value_t, N> > compute_eigenvectors(
    matrix<value_t, N, N> const& m,
    value_t const& min,
    value_t const& max,
    value_t const& eps) {
  matrix<value_t, N, N> identity;
  std::set<value_t> eigenvals = compute_eigenvalues(m, min, max, eps);
  std::vector<point<value_t, N> > eigenvecs;

  BOOST_FOREACH(value_t const & eigenval, eigenvals) {
    // target linear equation system for eigenvalue
    matrix<value_t, N, N> C = m - identity * eigenval;
    point<value_t, N> eigenvector;
    std::set<unsigned> free_parameter;
    std::map<unsigned, unsigned> nonfree_parameter;

    // elimination
    for (unsigned r = 0; r != N; ++r) {
      // eliminate round off errors by substitution with zero
      C.validate(eps);

      // first: find a pivot row
      std::pair<unsigned, unsigned> pivot = std::make_pair(N, N);
      for (unsigned i = r; i != N; ++i) {
        for (unsigned c = r; c != N; ++c) {
          if (c < pivot.first && fabs(C[i][c]) > eps) {  // found
            pivot = std::make_pair(c, i);
          }
        }
      }

      // eliminations have to be done
      if (pivot.second != N) {
        // if pivot is not the current row -> swap rows
        if (pivot.second != r) {
          std::swap(C[r], C[pivot.second]);
        }

        // do elimination of rows
        for (unsigned row = r + 1; row != N; ++row) {
          value_t factor = -C[row][pivot.first] / C[r][pivot.first];
          if (C[row][pivot.first] != 0) {
            for (unsigned col = 0; col != N; ++col) {
              C[row][col] =
                  (col <= pivot.first) ? 0 : C[row][col] + factor * C[r][col];
            }
          }
        }
      }

      if (pivot.second != N) {
        // set equation to solve later -> index changes to r
        nonfree_parameter.insert(std::make_pair(pivot.first, r));

        // overstepped parameter -> free parameter
        if (pivot.first != r) {
          free_parameter.insert(r);
        }
      } else {
        if (nonfree_parameter.find(r) ==
            nonfree_parameter
                .end()) {  // if not in nonfree_map -> add as free parameter
          free_parameter.insert(r);
        }
      }
    }

    int rank = N - int(free_parameter.size());

    if (rank == N) {
      // linear equation system is singular and first solution has to be set
      eigenvector[N - 1] = 1;
      // remove chosen parameter from nonfree-parameter list
      nonfree_parameter.erase(nonfree_parameter.find(N - 1));
    } else {
      // set all free parameter to 1
      BOOST_FOREACH(unsigned const & free, free_parameter) {
        eigenvector[free] = value_t(1);
      }
    }

    // solve all other equations
    for (std::map<unsigned, unsigned>::const_reverse_iterator pivot =
             nonfree_parameter.rbegin();
         pivot != nonfree_parameter.rend();
         ++pivot) {
      value_t rhs(0);
      for (unsigned c = 0; c != N; ++c) {
        if (c != pivot->first) {
          rhs -= C[pivot->second][c] * eigenvector[c];
        }
      }
      eigenvector[pivot->first] = rhs / C[pivot->second][pivot->first];
    }

    eigenvector /= eigenvector.abs();
    eigenvecs.push_back(eigenvector);
  }

  return eigenvecs;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
matrix<value_t, NROWS - 1, NCOLS - 1> generate_submatrix(
    matrix<value_t, NROWS, NCOLS> const& m,
    unsigned row,
    unsigned col) {
  assert(row < NROWS && col <= NCOLS);

  matrix<value_t, NROWS - 1, NCOLS - 1> sub;

  std::size_t rm = 0;
  for (std::size_t r = 0; r != NROWS - 1; ++r, ++rm) {
    if (r == row) {
      ++rm;
    }

    std::size_t cm = 0;
    for (std::size_t c = 0; c != NCOLS - 1; ++c, ++cm) {
      if (c == col) {
        ++cm;
      }
      sub[r][c] = m[rm][cm];
    }
  }
  return sub;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t>
matrix<value_t, 1, 1> generate_submatrix(matrix<value_t, 1, 1> const& m,
                                         unsigned /*row*/,
                                         unsigned /*col*/) {
  return m;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
    bool operator==(matrix<value_t, NROWS, NCOLS> const& lhs,
                    matrix<value_t, NROWS, NCOLS> const& rhs) {
  for (std::size_t r = 0; r != NROWS; ++r) {
    for (std::size_t c = 0; c != NCOLS; ++c) {
      if (lhs[r][c] != rhs[r][c])
        return false;
    }
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
    bool operator!=(matrix<value_t, NROWS, NCOLS> const& lhs,
                    matrix<value_t, NROWS, NCOLS> const& rhs) {
  return !(lhs == rhs);
}

//////////////////////////////////////////////////////////////////////////////
template <typename point_t, typename value_t, unsigned NROWS, unsigned NCOLS>
point_t operator*(matrix<value_t, NROWS, NCOLS> const& m, point_t const& p) {
  assert(point_t::coordinates == NROWS);
  point_t pm;

  for (std::size_t r = 0; r != NROWS; ++r) {
    for (std::size_t c = 0; c != NCOLS; ++c) {
      pm[r] += m[r][c] * p[c];
    }
  }

  pm.weight(p.weight());

  return pm;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t,
          unsigned NROWS_A,
          unsigned NCOLS_A,
          unsigned NROWS_B,
          unsigned NCOLS_B>
matrix<value_t, NROWS_A, NCOLS_B> operator*(
    matrix<value_t, NROWS_A, NCOLS_A> const& lhs,
    matrix<value_t, NROWS_B, NCOLS_B> const& rhs) {
  assert(NCOLS_A == NROWS_B);
  matrix<value_t, NROWS_A, NCOLS_B> m;

  for (std::size_t ra = 0; ra != NROWS_A; ++ra) {
    for (std::size_t cb = 0; cb != NCOLS_B; ++cb) {
      m[ra][cb] = 0;
      for (std::size_t i = 0; i != NROWS_B; ++i) {
        //for ( std::size_t ca = 0; ca != NCOLS_A; ++ca ) {
        m[ra][cb] += lhs[i][cb] * rhs[ra][i];
        //}
      }
    }
  }

  return m;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
matrix<value_t, NROWS, NCOLS> operator*(
    matrix<value_t, NROWS, NCOLS> const& lhs,
    value_t const& rhs) {
  matrix<value_t, NROWS, NCOLS> tmp(lhs);

  for (std::size_t r = 0; r != NROWS; ++r) {
    for (std::size_t c = 0; c != NCOLS; ++c) {
      tmp[r][c] *= rhs;
    }
  }

  return tmp;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
matrix<value_t, NROWS, NCOLS> operator*(
    value_t const& lhs,
    matrix<value_t, NROWS, NCOLS> const& rhs) {
  matrix<value_t, NROWS, NCOLS> tmp(rhs);

  for (std::size_t r = 0; r != NROWS; ++r) {
    for (std::size_t c = 0; c != NCOLS; ++c) {
      tmp[r][c] *= lhs;
    }
  }

  return tmp;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
    matrix<value_t,
           NROWS,
           NCOLS> operator-(matrix<value_t, NROWS, NCOLS> const& lhs,
                            matrix<value_t, NROWS, NCOLS> const& rhs) {
  matrix<value_t, NROWS, NCOLS> tmp(lhs);

  for (std::size_t r = 0; r != NROWS; ++r) {
    for (std::size_t c = 0; c != NCOLS; ++c) {
      tmp[r][c] -= rhs[r][c];
    }
  }

  return tmp;
}

//////////////////////////////////////////////////////////////////////////////
template <typename value_t, unsigned NROWS, unsigned NCOLS>
    std::ostream& operator<<(std::ostream& os,
                             matrix<value_t, NROWS, NCOLS> const& m) {
  m.print(os);
  return os;
}

}  // namespace tml

#endif  // TML_MATRIX_IMPL_HPP
