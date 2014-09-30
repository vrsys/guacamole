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

#ifndef GUA_BACKGROUND_PASS_HPP
#define GUA_BACKGROUND_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/GeometryRessource.hpp>

#include <memory>

namespace gua {

class Pipeline;

class BackgroundPass : public PipelinePass {
 public:

  virtual bool needs_color_buffer_as_input() const { return false; }
  virtual bool writes_only_color_buffer()    const { return true;  }

  virtual void process(Pipeline* pipe);

  friend class Pipeline;

 protected:
  BackgroundPass();
  ~BackgroundPass() {}

 private:
  std::shared_ptr<ShaderProgram>   shader_;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_;
};

}

#endif  // GUA_BACKGROUND_PASS_HPP
