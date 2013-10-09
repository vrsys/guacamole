/********************************************************************************
*
* Copyright (C) 2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : pointmesh2d.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_POINTMESH2D_HPP
#define TML_POINTMESH2D_HPP

// header, system
#include <cassert>   // std::assert
#include <vector>    // std::vector
#include <iterator>  // std::ostream_iterator

// header, project

namespace tml {

template <typename point_t> class pointmesh2d {

 public:  // typedefs

  typedef point_t point_type;
  typedef typename point_type::value_type value_type;
  typedef typename std::vector<point_type> container_type;
  typedef typename container_type::reference reference;
  typedef typename container_type::const_reference const_reference;
  typedef typename container_type::iterator iterator;
  typedef typename container_type::const_iterator const_iterator;

 public:  // c'tors and d'tor

  pointmesh2d();

  pointmesh2d(std::size_t width, std::size_t height);

  template <typename iterator_t>
  pointmesh2d(iterator_t begin,
              iterator_t end,
              std::size_t width,
              std::size_t height);

  pointmesh2d(pointmesh2d const& copy);

  ~pointmesh2d();

 public:  // methods

  pointmesh2d& operator=(pointmesh2d const& cp);

  void swap(pointmesh2d& swp);

  template <typename iterator_t> void add_row(iterator_t begin, iterator_t end);

  template <typename container_t> void add_row(container_t const& c);

  std::vector<point_t> row(std::size_t row) const;

  std::vector<point_t> column(std::size_t col) const;

  void push_back(point_t const& p);

  bool valid() const;

  void clear();

  std::size_t size() const;

  std::size_t width() const;
  void width(std::size_t w);
  std::size_t height() const;
  void height(std::size_t h);

  iterator begin();
  const_iterator begin() const;

  iterator end();
  const_iterator end() const;

  bool equals(pointmesh2d const& cp, value_type const& epsilon) const;

  void transpose();

  pointmesh2d subpatch(std::size_t left,
                       std::size_t right,
                       std::size_t top,
                       std::size_t bottom) const;

  void print(std::ostream& os) const;

 private:  // members

  std::size_t _width;
  std::size_t _height;
  std::vector<point_t> _points;
};

}  // namespace tml

#include "implementation/pointmesh2d_impl.hpp"

#endif  // TML_POINTMESH2D_HPP
