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

#ifndef GUA_UBER_SHADER_HPP
#define GUA_UBER_SHADER_HPP

// guacamole headers
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/ShadingModel.hpp>
#include <gua/renderer/UniformMapping.hpp>
#include <gua/renderer/LayerMapping.hpp>
#include <gua/renderer/UberShaderFactory.hpp>
#include <gua/renderer/Frustum.hpp>
#include <gua/renderer/enums.hpp>

#define CASES_PER_UBERSHADER_SWITCH 30

namespace gua {

class UberShaderFactory;

/**
 * This class represents a (multipass-) stage for rendering geometry into a layered fbo
 */
class UberShader {

 public: // typedefs, enums etc

   /**
   * logical combination of used render_stages provides valid rendermask, e.g:
   *
   * The rendermask ( PRE_FRAME_STAGE | PRE_DRAW_STAGE | DRAW_STAGE ) defines that
   * the ubershader provides an implementation of the defined stages
   */
   enum render_stage {
     NO_STAGE         = 0x00,
     PRE_FRAME_STAGE  = 0x01,
     PRE_DRAW_STAGE   = 0x02,
     DRAW_STAGE       = 0x04,
     POST_DRAW_STAGE  = 0x08,
     POST_FRAME_STAGE = 0x10
   };

   typedef unsigned stage_mask;

 public:

  /**
  * c'tor
  */
  UberShader();

  /**
  * d'tor
  */
  virtual ~UberShader();

  /**
  * initialize material dependent layer mappings for ubershaders
  */
  virtual void create(std::set<std::string> const& material_names);

  /**
   * set material dependent uniforms for all programs of ubershader
   */
  void set_material_uniforms(std::set<std::string> const& materials,
                             ShadingModel::StageID stage,
                             RenderContext const& context);

  /**
  * set a uniform for all programs of this uebershader
  */
  template <typename T>
  void set_uniform(RenderContext const& context,
    T const& value,
    std::string const& name,
    unsigned position = 0) const 
  {
    UniformValue<T> tmp(value);
    for (auto const& program : programs_) {  
      program->apply_uniform(context, &tmp, name, position);
    }
  }

  /**
   * get gbuffer layers of ubershader
   */
  virtual LayerMapping const* get_gbuffer_mapping() const;

  /**
   * get uniform mapping of ubershader
   */
  virtual UniformMapping const* get_uniform_mapping() const;

  /**
  * add a program to ubershader
  */
  virtual void add_program(std::shared_ptr<ShaderProgram> const&);

  /**
  * returns a container with all involved programs of this ubershader
  */ 
  std::vector<std::shared_ptr<ShaderProgram>> const& programs() const;

  /**
  * returns a program in enumerated order 
  */
  virtual std::shared_ptr<ShaderProgram> const& get_program(unsigned index = 0) const;

  /**
  * uploads ressources to the GPU
  */
  virtual bool upload_to(RenderContext const& context) const;

  /**
  *
  */
  virtual stage_mask const get_stage_mask() const { return NO_STAGE; }

  /**
  * This callback is called ONCE per frame BEFORE rendering all drawables of this type 
  *
  * default: no operations performed
  */
  virtual void pre_frame ( RenderContext const& context ) const {};

  /**
  * This method is called for ONCE per drawable to perform predraw operations
  *
  * default: no operations performed
  */
  virtual void predraw (  RenderContext const& context,
                          std::string const& name,
                          std::string const& material,
                          scm::math::mat4 const& model_matrix,
                          scm::math::mat4 const& normal_matrix,
                          Frustum const& frustum) const {};
   
  /**
  * This method is called for ONCE per drawable to perform draw operations
  *
  * default: no implementation provided
  */
  virtual void draw(RenderContext const& context,
                    std::string const& name,
                    std::string const& material,
                    scm::math::mat4 const& model_matrix,
                    scm::math::mat4 const& normal_matrix,
                    Frustum const& frustum) const {};

  /**
  * This method is called for ONCE per drawable to perform postdraw operations
  *
  * default: no operations performed
  */
  virtual void postdraw ( RenderContext const& context,
                          std::string const& name,
                          std::string const& material,
                          scm::math::mat4 const& model_matrix,
                          scm::math::mat4 const& normal_matrix,
                          Frustum const& frustum) const {};

  /**
  * This callback is called ONCE per frame AFTER rendering all drawables of this type
  *
  * default: no operations performed
  */
  virtual void post_frame ( RenderContext const& context) const {};

 protected: // methods

  void set_uniform_mapping(UniformMapping const& mapping);
  void set_output_mapping(LayerMapping const& mapping);

  std::string const print_material_switch(UberShaderFactory const& factory) const;
  std::string const print_material_methods(UberShaderFactory const& factory) const;

  protected: // attributes

  UniformMapping uniform_mapping_;
  LayerMapping output_mapping_;

  std::unique_ptr<UberShaderFactory> vshader_factory_;
  std::unique_ptr<UberShaderFactory> fshader_factory_;

  private: // attributes

  std::vector<std::shared_ptr<ShaderProgram>> programs_;

};

}

#endif  // GUA_UBER_SHADER_HPP
