/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : discretizer2d.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_DISCRETIZER2D_HPP
#define TML_DISCRETIZER2D_HPP

namespace tml {
namespace util {

/////////////////////////////////////////////////////////////////////////////
template <typename vec2_t> class discretizer2d {
 public:

  typedef typename vec2_t::value_type value_type;

 public:

  discretizer2d(size_t samples_x,
                size_t samples_y,
                value_type minx,
                value_type maxx,
                value_type miny,
                value_type maxy)
      : _samplesx(samples_x),
        _samplesy(samples_y),
        _current(0),
        _minx(minx),
        _maxx(maxx),
        _miny(miny),
        _maxy(maxy),
        _offsetx((maxx - minx) / (samples_x - 1)),
        _offsety((maxy - miny) / (samples_y - 1)) {}

  vec2_t operator()() {
    size_t y = _current / _samplesx;
    size_t x = _current % _samplesx;

    ++_current;

    return vec2_t(_minx + x * _offsetx, _miny + y * _offsety);
  }

  void reset() { _current = 0; }

 private:

  size_t _samplesx;
  size_t _samplesy;
  size_t _current;

  value_type _minx;
  value_type _maxx;
  value_type _miny;
  value_type _maxy;

  value_type _offsetx;
  value_type _offsety;
};

}  // namespace util
}  // namespace tml

#endif  // TML_DISCRETIZER2D_HPP
