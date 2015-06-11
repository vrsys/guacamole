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

#ifndef GUA_WARP_PASS_HPP
#define GUA_WARP_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>

#include <memory>

namespace gua {

class Pipeline;

class GUA_DLL WarpPassDescription : public PipelinePassDescription {
 public:

  enum GBufferWarpMode {
    GBUFFER_NONE,
    GBUFFER_POINTS,
    GBUFFER_SCALED_POINTS,
    GBUFFER_QUADS_SCREEN_ALIGNED,
    GBUFFER_QUADS_NORMAL_ALIGNED,
    GBUFFER_QUADS_DEPTH_ALIGNED,
    GBUFFER_GRID_DEPTH_THRESHOLD,
    GBUFFER_GRID_SURFACE_ESTIMATION,
    GBUFFER_GRID_ADVANCED_SURFACE_ESTIMATION
  };

  enum ABufferWarpMode {
    ABUFFER_NONE,
    ABUFFER_POINTS,
    ABUFFER_QUADS,
    ABUFFER_SCALED_POINTS
  };

  WarpPassDescription();

  WarpPassDescription& use_abuffer_from_window(std::string const& name);
  std::string const& use_abuffer_from_window() const;

  WarpPassDescription& max_layers(int val);
  int max_layers() const;

  WarpPassDescription& depth_test(bool val);
  bool depth_test() const;

  WarpPassDescription& debug_cell_colors(bool val);
  bool debug_cell_colors() const;

  WarpPassDescription& debug_cell_gap(bool val);
  bool debug_cell_gap() const;

  WarpPassDescription& gbuffer_warp_mode(GBufferWarpMode gbuffer_warp_mode);
  GBufferWarpMode gbuffer_warp_mode() const;

  WarpPassDescription& abuffer_warp_mode(ABufferWarpMode abuffer_warp_mode);
  ABufferWarpMode abuffer_warp_mode() const;

  WarpPassDescription& warp_matrix(math::mat4f const& mat);
  math::mat4f const& warp_matrix() const;

  std::shared_ptr<PipelinePassDescription> make_copy() const override;
  friend class Pipeline;

 protected:
  PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;

  std::string shared_window_name_;

  bool depth_test_;
  bool debug_cell_colors_;
  bool debug_cell_gap_;
  int max_layers_;
  GBufferWarpMode gbuffer_warp_mode_;
  ABufferWarpMode abuffer_warp_mode_;
};

}

#endif  // GUA_WARP_PASS_HPP
