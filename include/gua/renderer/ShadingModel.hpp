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

#ifndef GUA_SHADING_MODEL_HPP
#define GUA_SHADING_MODEL_HPP

// guacamole headers
#include <gua/renderer/ShaderStage.hpp>
#include <gua/utils/TextFile.hpp>

namespace gua {
/**
 * Stores information on a ShadingModel.
 *
 *
 */
class ShadingModel {
 public:

  enum StageID {
    GBUFFER_VERTEX_STAGE,
    GBUFFER_FRAGMENT_STAGE,
    LIGHTING_STAGE,
    FINAL_STAGE
  };
  /**
   * Default constructor.
   *
   */
  ShadingModel();

  /**
   * Constructor from a ShadingModel description.
   *
   */
  ShadingModel(std::string const& name);

  /**
   * Constructor from a ShadingModel description.
   *
   */
  ShadingModel(std::string const& name, std::string const& file_name);

  /**
   * Constructor from a buffer.
   *
   */
  ShadingModel(std::string const& name, const char* buffer, unsigned buffer_size);

  void reload();

  inline std::string const& get_name() const { return name_; }
  inline std::vector<ShaderStage>& get_stages() { return shader_stages_; }

  ShaderStage& get_gbuffer_vertex_stage();
  ShaderStage& get_gbuffer_fragment_stage();
  ShaderStage& get_lbuffer_stage();
  ShaderStage& get_final_shading_stage();

  void save_to_file(std::string const& file_name) const;

  static unsigned current_revision;

 private:
  void construct_from_file(TextFile const& file);
  void construct_from_buffer(const char* buffer, unsigned buffer_size);


  std::string file_name_;

  std::vector<ShaderStage> shader_stages_;
  std::string name_;
};

}

#endif  // GUA_SHADING_MODEL_HPP
