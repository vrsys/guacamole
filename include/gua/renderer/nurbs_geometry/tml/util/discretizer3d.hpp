/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : discretizer3d.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_DISCRETIZER3D_HPP
#define TML_DISCRETIZER3D_HPP

namespace tml {
namespace util {

/////////////////////////////////////////////////////////////////////////////
template <typename vec3_t> class discretizer3d {
 public:

  typedef typename vec3_t::value_type value_type;

 public:

  discretizer3d(size_t samples_x,
                size_t samples_y,
                size_t samples_z,
                value_type minx,
                value_type maxx,
                value_type miny,
                value_type maxy,
                value_type minz,
                value_type maxz)
      : _samplesx(samples_x),
        _samplesy(samples_y),
        _samplesz(samples_z),
        _current(0),
        _minx(minx),
        _maxx(maxx),
        _miny(miny),
        _maxy(maxy),
        _minz(minz),
        _maxz(maxz),
        _offsetx((maxx - minx) / (samples_x - 1)),
        _offsety((maxy - miny) / (samples_y - 1)),
        _offsetz((maxz - minz) / (samples_z - 1)) {}

  vec3_t operator()() {
    size_t x = _current % _samplesx;
    size_t y = (_current % (_samplesx * _samplesy)) / _samplesx;
    size_t z = _current / (_samplesx * _samplesy);

    ++_current;

    return vec3_t(
        _minx + x * _offsetx, _miny + y * _offsety, _minz + z * _offsetz);
  }

  void reset() { _current = 0; }

 private:

  size_t _samplesx;
  size_t _samplesy;
  size_t _samplesz;
  size_t _current;

  value_type _minx;
  value_type _maxx;
  value_type _miny;
  value_type _maxy;
  value_type _minz;
  value_type _maxz;

  value_type _offsetx;
  value_type _offsety;
  value_type _offsetz;
};

}  // namespace util
}  // namespace tml

#endif  // TML_DISCRETIZER2D_HPP
