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
#include <gua/platform.hpp>
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
 * This class represents a (multipass-) stage with material-dependent meta-shader components
 */
class GUA_DLL UberShader {

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

  virtual void cleanup (RenderContext const& context);

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

  virtual void save_shaders_to_file(std::string const& directory,
                                    std::string const& name) const;

  /**
  * returns a program in enumerated order
  */
  virtual std::shared_ptr<ShaderProgram> const& get_program(unsigned index = 0) const;

  /**
  * returns a container with all involved programs of this ubershader
  */
  std::vector<std::shared_ptr<ShaderProgram>> const& programs() const;

  /**
  * uploads ressources to the GPU
  */
  virtual bool upload_to(RenderContext const& context) const;

  virtual void set_left_resolution(math::vec2ui const& resolution);
  virtual void set_right_resolution(math::vec2ui const& resolution);

 protected: // methods

  void set_uniform_mapping(UniformMapping const& mapping);
  void set_output_mapping(LayerMapping const& mapping);

  std::string print_material_switch(UberShaderFactory const& factory) const;
  std::string print_material_methods(UberShaderFactory const& factory) const;

  protected: // attributes

  UniformMapping uniform_mapping_;
  LayerMapping output_mapping_;

  std::unique_ptr<UberShaderFactory> vshader_factory_;
  std::unique_ptr<UberShaderFactory> fshader_factory_;

  math::vec2ui                                left_resolution_;
  math::vec2ui                                right_resolution_;

  private: // attributes

  std::vector<std::shared_ptr<ShaderProgram>> programs_;

};

}

#endif  // GUA_UBER_SHADER_HPP
