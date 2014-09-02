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

#ifndef GUA_PIPELINE_PASS_HPP
#define GUA_PIPELINE_PASS_HPP

#include <string>

namespace gua {

class PipelinePass {
 public:

  PipelinePass& set_source(std::string const& source) {

    std::string header(R"(
      #version 420

      uniform ...

    )");

    source_ = header + source;

    return *this;
  }

  void process() {}

  friend class Pipeline;

 protected:
  PipelinePass() {}
  ~PipelinePass() {}

 private:
  std::string source_;
};






class LightingPass : public PipelinePass {
 public:

  friend class Pipeline;

 protected:
  LightingPass() {
    set_source(R"(
      ...
    )");
  }
  ~LightingPass() {}
};





class SSAOPass : public PipelinePass {
 public:

  SSAOPass& set_radius(float val) {
    return *this;
  }

  SSAOPass& set_intensity(float val) {
    return *this;
  }

  friend class Pipeline;

 protected:
  SSAOPass() {
    set_source(R"(
      ...
    )");
  }
  ~SSAOPass() {}
};




}

#endif  // GUA_PIPELINE_PASS_HPP
