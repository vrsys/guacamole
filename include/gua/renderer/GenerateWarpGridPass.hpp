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

#ifndef GUA_GENERATE_WARP_GRID_PASS_HPP
#define GUA_GENERATE_WARP_GRID_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>

#include <memory>

namespace gua {

class Pipeline;

class GUA_DLL GenerateWarpGridPassDescription : public PipelinePassDescription {
 public:

  enum Mode {
    SURFACE_ESTIMATION,
    ADAPTIVE_SURFACE_ESTIMATION,
    DEPTH_THRESHOLD
  };

  GenerateWarpGridPassDescription();

  GenerateWarpGridPassDescription& cell_size(int val);
  int cell_size() const;

  GenerateWarpGridPassDescription& split_threshold(float val);
  float split_threshold() const;

  GenerateWarpGridPassDescription& mode(Mode mode);
  Mode mode() const;

  std::shared_ptr<PipelinePassDescription> make_copy() const override;
  friend class Pipeline;

 protected:
  PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;

  int cell_size_;
  float split_threshold_;
  Mode mode_;
};

}

#endif  // GUA_GENERATE_WARP_GRID_PASS_HPP
