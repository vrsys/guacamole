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

#ifndef GUA_RENDER_WARP_GRID_PASS_HPP
#define GUA_RENDER_WARP_GRID_PASS_HPP

#include <gua/renderer/WarpPass.hpp>

#include <memory>

namespace gua {

class Pipeline;

class GUA_DLL RenderWarpGridPassDescription : public PipelinePassDescription {
 public:

  RenderWarpGridPassDescription();

  RenderWarpGridPassDescription& show_warp_grid(bool val);
  bool show_warp_grid() const;

  RenderWarpGridPassDescription& mode(WarpPassDescription::GBufferWarpMode mode);
  WarpPassDescription::GBufferWarpMode mode() const;

  std::shared_ptr<PipelinePassDescription> make_copy() const override;
  friend class Pipeline;

 protected:
  PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;

  bool show_warp_grid_;
  WarpPassDescription::GBufferWarpMode mode_;
};

}

#endif  // GUA_RENDER_WARP_GRID_PASS_HPP
