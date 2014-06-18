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

#ifndef GUA_UNIFORM_MAPPING_HPP
#define GUA_UNIFORM_MAPPING_HPP

// guacamole headers
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ShadingModel.hpp>

// external headers
#include <memory>

namespace gua {

/**
 * Maps stuff like
 *        Red/shinyness -> gua_floats[7], passed as std::pair("gua_floats", 7)
 */
class UniformMapping {
 public:

  /**
   *
   */
  UniformMapping();

  /**
   *
   */
  void add(std::string const& material, std::string const& uniform);

  /**
   *
   */
  std::pair<std::string, int> const& get_mapping(
      std::string const& material,
      std::string const& uniform) const;

  std::unordered_map<
      std::string,
      std::unordered_map<std::string, std::pair<std::string, int> > > const& get_mapping() const {

    return mapping_;
  }

  std::string const get_uniform_definition(Pipeline::PipelineStage stage) const;

  /**
   *
   */
  int get_uniform_count(UniformType type) const;

  /**
   *
   */
  std::map<UniformType, int> const& get_uniform_counts() const;

 private:

  //       material_name, uniform_name, mapped_type, mapped_position
  std::unordered_map<
      std::string,
      std::unordered_map<std::string, std::pair<std::string, int> > > mapping_;

  //       UniformType, count
  std::map<UniformType, int> uniform_counts_;

  // for error returns
  std::pair<std::string, int> error_;
};

}

#endif  // GUA_UNIFORM_MAPPING_HPP
