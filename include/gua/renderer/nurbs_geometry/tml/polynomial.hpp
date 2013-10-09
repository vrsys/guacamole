/********************************************************************************
*
* Copyright (C) 2011 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : polynomial.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_POLYNOM_HPP
#define TML_POLYNOM_HPP

#include <cassert>
#include <set>

#include <boost/array.hpp>
#include <boost/numeric/conversion/bounds.hpp>

namespace tml {
template <typename value_t> class polynomial {
 public:  // typedefs

  typedef value_t value_type;

 public:

  polynomial() : _coefficients(1) {}
  /*
  template <typename iterator_t>
  polynomial(iterator_t b, iterator_t e)
    : _coefficients(b, e)
  {}
  */
  polynomial(value_t const& x0) : _coefficients(1) { _coefficients[0] = x0; }

  polynomial(value_t const& x0, value_t const& x1) : _coefficients(2) {
    _coefficients[0] = x0;
    _coefficients[1] = x1;
  }

  polynomial(value_t const& x0, value_t const& x1, value_t const& x2)
      : _coefficients(3) {
    _coefficients[0] = x0;
    _coefficients[1] = x1;
    _coefficients[2] = x2;
  }

  std::size_t order() const { return _coefficients.size(); }

  void order(std::size_t const& order) {
    return _coefficients.resize(std::max(order, std::size_t(1)));
  }

  void simplify(value_type threshold =
                    boost::numeric::bounds<value_type>::smallest()) {
    for (std::size_t i = 0; i != order(); ++i) {
      if (fabs(_coefficients[i]) <= threshold) {
        _coefficients[i] = 0;
      }
    }

    collapse();
  }

  void collapse() {
    std::size_t last_non_zero_coefficient = 0;

    for (std::size_t i = 0; i != order(); ++i) {
      if (_coefficients[i] != 0) {
        last_non_zero_coefficient = i;
      }
    }

    order(last_non_zero_coefficient + 1);
  }

  std::set<value_type> solve(
      value_type xmin = boost::numeric::bounds<value_type>::lowest(),
      value_type xmax = boost::numeric::bounds<value_type>::highest(),
      value_type epsilon =
          boost::numeric::bounds<value_type>::smallest()) const {
    std::set<value_type> roots;

    if (order() == 1)
      return roots;

    // find solutions on lower degree
    polynomial<value_type> dp = *this;
    dp.derive();
    std::set<value_type> intervals = dp.solve(xmin, xmax, epsilon);

    // insert border limits
    intervals.insert(xmin);
    intervals.insert(xmax);

    // search each interval for root
    typename std::set<value_type>::const_iterator low = intervals.begin();
    typename std::set<value_type>::const_iterator high = intervals.begin();
    std::advance(high, 1);

    while (high != intervals.end()) {
      value_type root;
      if (bisect_to_root(root, *low, *high, epsilon)) {
        roots.insert(root);
      }
      ++low;
      ++high;
    }

    return roots;
  }

  value_type& operator[](std::size_t const& i) {
    assert(i < _coefficients.size());
    return _coefficients[i];
  }

  value_type const& operator[](std::size_t const& i) const {
    assert(i < _coefficients.size());
    return _coefficients[i];
  }

  polynomial operator-() {
    polynomial neg;

    for (std::size_t i = 0; i != order(); ++i) {
      neg._coefficients[i] = -_coefficients[i];
    }

    return neg;
  }

  polynomial const& operator+=(polynomial const& rhs) {
    order(std::max(rhs.order(), order()));

    for (std::size_t i = 0; i != order(); ++i) {
      if (i < rhs.order()) {
        _coefficients[i] += rhs._coefficients[i];
      } else {
        break;
      }
    }

    simplify();

    return *this;
  }

  polynomial const& operator-=(polynomial const& rhs) {
    order(std::max(rhs.order(), order()));

    for (std::size_t i = 0; i != order(); ++i) {
      if (i < rhs.order()) {
        _coefficients[i] -= rhs._coefficients[i];
      } else {
        break;
      }
    }

    simplify();

    return *this;
  }

  polynomial const& operator*=(value_type const& rhs) {
    for (std::size_t i = 0; i != order(); ++i) {
      _coefficients[i] *= rhs;
    }

    simplify();

    return *this;
  }

  polynomial const& operator*=(polynomial const& rhs) {
    polynomial tmp;
    tmp.order(order() + rhs.order());

    for (std::size_t i = 0; i != rhs.order(); ++i) {
      for (std::size_t j = 0; j != order(); ++j) {
        if (rhs[i] != 0 && _coefficients[j] != 0) {
          tmp[i + j] += rhs[i] * _coefficients[j];
        }
      }
    }

    *this = tmp;

    simplify();

    return *this;
  }

  value_type evaluate(value_type const& x) const {
    value_type fx = 0;

    for (std::size_t i = 0; i != order(); ++i) {
      fx += _coefficients[i] * std::pow(value_type(x), value_type(i));
    }

    return fx;
  }

  void derive() {
    polynomial<value_t> dp;

    if (order() > 1) {
      dp.order(order() - 1);

      for (std::size_t i = 0; i < order() - 1; ++i) {
        dp[i] = (i + 1) * _coefficients[i + 1];
      }
    }

    std::swap(dp, *this);
  }

  void normalize() {
    value_type xmin = boost::numeric::bounds<value_type>::highest();
    for (std::size_t i = 0; i != order(); ++i) {
      xmin = std::min(_coefficients[i], xmin);
    }
    for (std::size_t i = 0; i != order(); ++i) {
      _coefficients[i] /= xmin;
    }
  }

  bool bisect_to_root(value_type& root,
                      value_type xmin,
                      value_type xmax,
                      value_type epsilon) const {
    value_type ymin = evaluate(xmin);
    value_type ymax = evaluate(xmax);

    value_type dx = xmax - xmin;

    while (true) {
      // no root in interval
      if (ymin * ymax > 0) {
        return false;
      } else {
        // evaluate and bisect interval
        value_type x = (xmax + xmin) / 2;
        value_type y = evaluate(x);

        if (y * ymax > 0 &&
            y * ymin > 0) {  // both positive or negative -> no root
          return false;
        }

        if (xmax - xmin < epsilon && (ymin * ymax <= 0) &&
            fabs(ymin) < epsilon &&
            fabs(ymax) < epsilon) {  // necessary accuracy reached
          root = x;
          return true;
        }

        if (x >= xmax || x <= xmin) {  // numerical drop out
          root = x;
          return true;
        }

        if (y == 0) {  // precise solution found
          root = x;
          return true;
        }

        if (y * ymax < 0) {  // continue with x as new xmin
          xmin = x;
          ymin = y;
        }

        if (y * ymin < 0) {  // continue with x as new xmax
          xmax = x;
          ymax = y;
        }
      }

      if (xmax - xmin > dx)
        break;
    }

    return false;
  }

  void print(std::ostream& os) const {
    for (std::size_t i = 0; i != order(); ++i) {
      os << _coefficients[i] << "x^" << i;
      if (i != order() - 1) {
        os << " + ";
      }
    }
  }

 private:

  std::vector<value_type> _coefficients;

};

template <typename value_t>
polynomial<value_t> pow(polynomial<value_t> const& p, std::size_t n) {
  polynomial<value_t> tmp = (n == 0) ? polynomial<value_t>(1) : p;

  for (std::size_t i = 1; i < n; ++i) {
    tmp *= p;
  }

  return tmp;
}

template <typename value_t>
polynomial<value_t> derive(polynomial<value_t> const& p) {
  polynomial<value_t> dp(p);
  dp.derive();
  return dp;
}

template <typename value_t>
    polynomial<value_t> operator+(polynomial<value_t> const& lhs,
                                  polynomial<value_t> const& rhs) {
  polynomial<value_t> tmp(lhs);
  tmp += rhs;
  return tmp;
}

template <typename value_t>
    polynomial<value_t> operator-(polynomial<value_t> const& lhs,
                                  polynomial<value_t> const& rhs) {
  polynomial<value_t> tmp(lhs);
  tmp -= rhs;
  return tmp;
}

template <typename value_t>
polynomial<value_t> operator*(polynomial<value_t> const& lhs,
                              value_t const& rhs) {
  polynomial<value_t> tmp(lhs);
  tmp *= rhs;
  return tmp;
}

template <typename value_t>
polynomial<value_t> operator*(value_t const& lhs,
                              polynomial<value_t> const& rhs) {
  polynomial<value_t> tmp(rhs);
  tmp *= lhs;
  return tmp;
}

template <typename value_t>
polynomial<value_t> operator*(polynomial<value_t> const& lhs,
                              polynomial<value_t> const& rhs) {
  polynomial<value_t> tmp(lhs);
  tmp *= rhs;
  return tmp;
}

template <typename value_t>
    bool operator==(polynomial<value_t> const& lhs,
                    polynomial<value_t> const& rhs) {
  return !(lhs != rhs);
}

template <typename value_t>
    bool operator!=(polynomial<value_t> const& lhs,
                    polynomial<value_t> const& rhs) {
  std::size_t max_order = std::max(lhs.order(), rhs.order());

  for (std::size_t i = 0; i != max_order; ++i) {
    if (i < lhs.order() && i < rhs.order()) {
      if (lhs[i] != rhs[i]) {
        return true;
      }
    } else {
      if (i < lhs.order() && lhs[i] != 0) {
        return true;
      }
      if (i < rhs.order() && rhs[i] != 0) {
        return true;
      }
    }
  }
  return false;
}

template <typename value_t>
    std::ostream& operator<<(std::ostream& os, polynomial<value_t> const& p) {
  p.print(os);
  return os;
}

}

#endif
