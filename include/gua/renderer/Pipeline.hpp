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

#ifndef GUA_PIPELINE_HPP
#define GUA_PIPELINE_HPP

#include <gua/renderer/Renderer.hpp>
#include <gua/renderer/PipelinePass.hpp>
#include <gua/renderer/SerializedScene.hpp>
#include <gua/renderer/Texture2D.hpp>
#include <gua/renderer/FrameBufferObject.hpp>
#include <gua/renderer/RessourceRenderer.hpp>
#include <gua/renderer/CameraUniformBlock.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/configuration_macro.hpp>
#include <gua/math.hpp>

#include <memory>

namespace gua {

class GBuffer;
class WindowBase;
class RenderContext;

class PipelineDescription {
 public:

  PipelineDescription() {}

  PipelineDescription(PipelineDescription const& other) {
    for (auto pass: other.passes_) {
      passes_.push_back(pass->make_copy());
    }
  }

  virtual ~PipelineDescription() {
    for (auto pass: passes_) {
      delete pass;
    } 
  }

  template<class T>
  T& add_pass() {
    T* t = new T();
    passes_.push_back(t);
    return *t;
  }

  std::vector<PipelinePassDescription*> const& get_passes() const {
    return passes_;
  }


  bool operator==(PipelineDescription const& other) const {
    if (passes_.size() != other.passes_.size()) {
      return false;
    }

    for (int i(0); i<passes_.size(); ++i) {
      if (typeid(passes_[i]) != typeid(other.passes_[i])) {
        return false;
      }
    } 

    return true;
  }

  bool operator!=(PipelineDescription const& other) const {
    return !(*this == other);
  }

  PipelineDescription& operator=(PipelineDescription const& other) {
    for (auto pass: passes_) {
      delete pass;
    } 

    passes_.clear();

    for (auto pass: other.passes_) {
      passes_.push_back(pass->make_copy());
    }

    return *this;
  }
 
 private:
  std::vector<PipelinePassDescription*> passes_;
};




class Pipeline {
 public:
  
  static PipelineDescription make_default();

  Pipeline();
  ~Pipeline();

  void process(node::SerializedCameraNode const& camera,
               std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs,
               float application_fps, float rendering_fps);

  std::vector<PipelinePass*>  const& get_passes()  const;
  GBuffer                          & get_gbuffer() const;
  SerializedScene             const& get_scene()   const;
  RenderContext               const& get_context() const;
  RenderContext                    & get_context();
  
  void bind_gbuffer_input(std::shared_ptr<ShaderProgram> const& shader) const;
  void bind_camera_uniform_block(unsigned location) const;
  void draw_fullscreen_quad();

  std::shared_ptr<RessourceRenderer> get_renderer(GeometryResource const& type);

 private:
  GBuffer*                           gbuffer_;
  std::shared_ptr<WindowBase>        window_;
  SerializedScene                    current_scene_;
  CameraUniformBlock*                camera_block_;

  math::vec2ui                       last_resolution_;
  PipelineDescription                last_description_;
  
  std::vector<PipelinePass*>         passes_;
  std::unordered_map<std::type_index, std::shared_ptr<RessourceRenderer>> renderers_; 

  scm::gl::quad_geometry_ptr         fullscreen_quad_;

};

}

#endif  // GUA_PIPELINE_HPP
