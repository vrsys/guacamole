/********************************************************************************
*
* Copyright (C) 2010 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : prefilter3d.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_PREFILTER3D_HPP
#define TML_PREFILTER3D_HPP

#include <vector>
#include <numeric>
#include <algorithm>
#include <iterator>

#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

#include <gua/renderer/nurbs_geometry/tml/util/gauss2d.hpp>
#include <gua/renderer/nurbs_geometry/tml/util/copy_if.hpp>
#include <gua/renderer/nurbs_geometry/tml/util/discretizer2d.hpp>
#include <gua/renderer/nurbs_geometry/tml/util/signed_gradient.hpp>

namespace tml {
namespace util {

template <typename vec2_t> struct circular_filter {
  typedef typename vec2_t::value_type value_type;
  value_type radius;

  circular_filter(value_type const& r) : radius(r) {}

  bool operator()(vec2_t const& sample) {
    return radius > sqrt(sample[0] * sample[0] + sample[1] * sample[1]);
  }
};

///////////////////////////////////////////////////////////////////////////
// functor for prefiltering a circular pixel approximation with
// two intersecting edges
//
// - input :  vec3  ( angle,     /* between the two gradients of the edges */
//                    distance1  /* distance of first gradient to pixel center
// */
//                    distance2  /* distance of second gradient to pixel center
// */
//                  )
//
// - positive distance means pixel center is IN object -> coverage > 50%
//
// - output : vec3 ( percentage_coverage_first,
//                   percentage_coverage_second,
//                   percentage_coverage_both )
//
// - samples of circular pixel have 2d gaussian weights
///////////////////////////////////////////////////////////////////////////
template <typename vec3_t> class prefilter3d {
 public:

  typedef typename vec3_t::value_type value_type;
  typedef point<value_type, 2> vec2_t;

 public:

  ///////////////////////////////////////////////////////////////////////////
  prefilter3d(size_t samples = 64,
              value_type gauss_sigma = 0.5,
              value_type pixel_radius = 0.5)
      : _nsamples(samples),
        _sigma(gauss_sigma),
        _samples(),
        _weights(),
        _pixel_radius(pixel_radius),
        _maxintegral(0) {
    // create a discretizer that samples a rectangular domain
    discretizer2d<point<value_type, 2> > dtizer(_nsamples,
                                                _nsamples,
                                                -_pixel_radius,
                                                _pixel_radius,
                                                -_pixel_radius,
                                                _pixel_radius);

    // generate uniformly distributed samples in rectangular domain
    std::vector<vec2_t> tmp(_nsamples * _nsamples);
    std::generate(tmp.begin(), tmp.end(), dtizer);

    // filter samples that lie outside of a circular pixel approximation
    tml::util::copy_if(tmp.begin(),
                       tmp.end(),
                       std::back_inserter(_samples),
                       circular_filter<vec2_t>(_pixel_radius));
    _weights.resize(_samples.size());

    // compute inner-pixel weights
    std::transform(_samples.begin(),
                   _samples.end(),
                   _weights.begin(),
                   gauss2d<vec2_t>(_sigma));

    // integrate overall samples
    _maxintegral =
        std::accumulate(_weights.begin(), _weights.end(), value_type(0));
  }

  ///////////////////////////////////////////////////////////////////////////
  // argument = [angle, radius1, radius2]
  ///////////////////////////////////////////////////////////////////////////
  vec3_t operator()(vec3_t const& argument) const {
    vec3_t coverage(
        0, 0, 0);  // [first gradient, second gradient, both gradients]
    classify_sample_by_signed_gradient<point<value_type, 2> > insidetest;

    typename std::vector<value_type>::const_iterator w = _weights.begin();
    for (typename std::vector<vec2_t>::const_iterator s = _samples.begin();
         s != _samples.end();
         ++s, ++w) {
      // compute both gradients: since pixel is circularly approximated first
      // gradient has angle of 0, the second relatively
      vec2_t fst_gradient(0, argument[1]);
      vec2_t snd_gradient(argument[0], argument[2]);

      // sample covered by object?
      bool covered_fst = insidetest(fst_gradient, *s);
      bool covered_snd = insidetest(snd_gradient, *s);

      // sum up weighted samples
      coverage[0] += (*w) * value_type(covered_fst && !covered_snd);
      coverage[1] += (*w) * value_type(!covered_fst && covered_snd);
      coverage[2] += (*w) * value_type(covered_fst && covered_snd);
    }

    // normalize coverage
    coverage /= _maxintegral;

    // return coverage
    return coverage;
  }

 private:

  size_t _nsamples;
  value_type _sigma;
  std::vector<vec2_t> _samples;
  std::vector<value_type> _weights;
  value_type _pixel_radius;
  value_type _maxintegral;
};

}  // namespace util
}  // namespace tml

#endif  // TML_PREFILTER3D_HPP
