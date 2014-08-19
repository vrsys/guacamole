/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

// class header
#include <gua/renderer/nurbs_geometry/TrimmedSurfaceConverter.hpp>

// header, system
#include <list>
#include <cmath>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/pointmesh2d.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/algorithm/converter.hpp>

#include <gua/renderer/nurbs_geometry/TrimmedBezierSurfaceObject.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedNurbsSurfaceObject.hpp>

namespace gua {

class TrimmedSurfaceConverter::impl_t {
 public:
  impl_t()
      : target(),
        source(),
        next(),
        source_access(),
        target_access(),
        nthreads(16) {}

  std::shared_ptr<TrimmedBezierSurfaceObject> target;
  std::shared_ptr<TrimmedNurbsSurfaceObject> source;
  TrimmedNurbsSurfaceObject::const_iterator next;
  boost::mutex source_access;
  boost::mutex target_access;
  std::size_t nthreads;
};

////////////////////////////////////////////////////////////////////////////////
TrimmedSurfaceConverter::TrimmedSurfaceConverter() : _impl(new impl_t) {}

////////////////////////////////////////////////////////////////////////////////
TrimmedSurfaceConverter::~TrimmedSurfaceConverter() { delete _impl; }

////////////////////////////////////////////////////////////////////////////////
void TrimmedSurfaceConverter::convert(
    std::shared_ptr<TrimmedNurbsSurfaceObject> const& ns,
    std::shared_ptr<TrimmedBezierSurfaceObject> const& bs) {
  _impl->target = bs;
  _impl->source = ns;
  _impl->next = _impl->source->begin();

#if 1  // multithreaded conversion
  std::vector<boost::thread*> threadpool;

  for (std::size_t i = 0; i != _impl->nthreads; ++i) {
    auto task = boost::bind(&TrimmedSurfaceConverter::_fetch_task, this);
    threadpool.push_back(new boost::thread(task));
  }

  std::for_each(threadpool.begin(),
                threadpool.end(),
                boost::bind(&boost::thread::join, _1));

  BOOST_FOREACH(auto thread_ptr, threadpool) { delete thread_ptr; }

#else  // single-threaded conversion
  _fetch_task();
#endif
}

////////////////////////////////////////////////////////////////////////////////
void TrimmedSurfaceConverter::_convert(
    TrimmedNurbsSurfaceObject::surface_type const& nurbspatch) {
  tml::converter2d conv2d;
  tml::converter3d conv3d;

  // first convert trimmed nurbs surface into bezierpatches
  std::vector<tml::beziersurface_from_nurbs<tml::point3d> > bezierpatches;
  conv3d.convert(nurbspatch, std::back_inserter(bezierpatches));

  // then convert all trimming nurbs curves into bezier curves
  std::vector<TrimDomain::contour_type> loops;

  for (TrimmedNurbsSurface::const_trimloop_iterator l =
           nurbspatch.trimloops().begin();
       l != nurbspatch.trimloops().end();
       ++l) {
    // first split nurbs loop into bezier loop
    TrimmedBezierSurface::curve_container loop;
    for (auto c = l->begin(); c != l->end(); ++c) {
      conv2d.convert(*c, std::back_inserter(loop));
    }

    // convert to pointer
    std::vector<TrimDomain::curve_ptr> loop_as_ptr;
    for (auto c = loop.begin(); c != loop.end(); ++c) {
      loop_as_ptr.push_back(
          TrimDomain::curve_ptr(new TrimDomain::curve_type(*c)));
    }

    loops.push_back(
        TrimDomain::contour_type(loop_as_ptr.begin(), loop_as_ptr.end()));
  }

  TrimmedBezierSurface::trimdomain_ptr domain(new TrimDomain);

  // add curves, set trimtype and which part of the nurbs patch the patch is
  std::for_each(
      loops.begin(), loops.end(), boost::bind(&TrimDomain::add, domain, _1));
  domain->type(nurbspatch.trimtype());
  domain->nurbsdomain(TrimDomain::bbox_type(
      TrimDomain::point_type(nurbspatch.umin(), nurbspatch.vmin()),
      TrimDomain::point_type(nurbspatch.umax(), nurbspatch.vmax())));

  // generate trimmed bezier patches from subpatches and trimming curves
  for (std::vector<tml::beziersurface_from_nurbs<tml::point3d> >::const_iterator
           b = bezierpatches.begin();
       b != bezierpatches.end();
       ++b) {
    boost::shared_ptr<TrimmedBezierSurface> tbs(
        new TrimmedBezierSurface(b->surface));

    tbs->domain(domain);

    tbs->bezierdomain(
        TrimDomain::bbox_type(TrimDomain::point_type(b->umin, b->vmin),
                              TrimDomain::point_type(b->umax, b->vmax)));

    // lock for write access into TrimmedBezierSurfaceObject
    boost::mutex::scoped_lock lock(_impl->target_access);
    _impl->target->add(tbs);
  }
}

////////////////////////////////////////////////////////////////////////////////
void TrimmedSurfaceConverter::_fetch_task() {
  bool something_to_do = true;

  while (something_to_do) {
    // try to get a task
    boost::mutex::scoped_lock lck(_impl->source_access);

    if (_impl->next != _impl->source->end()) {
      TrimmedNurbsSurface const& task = *(_impl->next);
      ++(_impl->next);
      _convert(task);

    } else {
      something_to_do = false;
    }
  }

}

}  // namespace gua
