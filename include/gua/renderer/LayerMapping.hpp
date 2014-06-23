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

#ifndef GUA_LAYER_MAPPING_HPP
#define GUA_LAYER_MAPPING_HPP

// guacamole headers
#include <gua/renderer/ShadingModel.hpp>

// external headers
#include <scm/gl_core/state_objects/sampler_state.h>
#include <memory>

namespace gua {

/**
 * Maps stuff like
 *     There are some default layers:
 *         PRE_GBUFFER:
 *             1. layer depth: depth --- also used for gbuffer_position
 *             1. layer float: normal
 *             1. layer unsigned: material_id
 *         POST_GBUFFER:
 *             1. layer float: normal_mapped
 *         FINAL_BUFFER
 *             1. layer float: color
 *
 *
 */
class LayerMapping {
 public:

  /**
   *
   */
  LayerMapping();

  LayerMapping(std::set<std::string> const& materials,
               ShadingModel::StageID stage);

  /**
   *
   */
  void add(std::string const& shading_model_name,
           std::string const& layer_name,
           BufferComponent layer_type);

  /**
   * eg: SimplePhong/normal -> gua_float_gbuffer_out_0.xyz
   */
  std::string get_output_string(std::string const& shading_model_name,
                                      std::string const& output_name) const;

  /**
   * eg: SimplePhong/normal ->
   * texture2D(gua_get_float_sampler(gua_float_gbuffer_in[1]),
   * gua_get_quad_coords()).xyz
   */
  std::string get_input_string(std::string const& shading_model_name,
                                     std::string const& input_name,
                                     ShadingModel::StageID from_stage) const;

  /**
   *
   */
  std::string get_gbuffer_input_definition(
      ShadingModel::StageID from_stage) const;
  std::string get_gbuffer_output_definition(bool as_input,
                                                  bool as_varying) const;

  std::vector<std::pair<BufferComponent,
                        scm::gl::sampler_state_desc> > get_layers(
      BufferComponentType type) const;
  std::vector<std::pair<BufferComponent,
                        scm::gl::sampler_state_desc> > get_layers() const;

  ShadingModel::StageID get_stage() const;

 private:
  BufferComponent combine_outputs(
      std::vector<BufferComponent> const& outputs) const;
  BufferComponent get_largest(
      std::vector<BufferComponent> const& outputs) const;
  void store_defaults(std::string const& shading_model_name);

  ShadingModel::StageID stage_;

  // stores for each output layer a map which stores for each shading
  // model the outputs with their types which are rendered to this layer
  //                   model_name                         output_name
  // output_type
  std::vector<std::unordered_map<
      std::string,
      std::vector<std::pair<std::string, BufferComponent> > > >
      integer_mapping_;
  std::vector<std::unordered_map<
      std::string,
      std::vector<std::pair<std::string, BufferComponent> > > >
      unsigned_mapping_;
  std::vector<std::unordered_map<
      std::string,
      std::vector<std::pair<std::string, BufferComponent> > > > half_mapping_;
  std::vector<std::unordered_map<
      std::string,
      std::vector<std::pair<std::string, BufferComponent> > > > float_mapping_;
};

}

#endif  // GUA_LAYER_MAPPING_HPP
