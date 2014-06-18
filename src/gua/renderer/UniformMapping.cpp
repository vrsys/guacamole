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

// class header
#include <gua/renderer/UniformMapping.hpp>

// guacamole headers
#include <gua/databases.hpp>
#include <gua/utils.hpp>

namespace gua {

UniformMapping::UniformMapping() : error_("", -1) {}

void UniformMapping::add(std::string const& material,
                         std::string const& uniform) {
  auto mat_ptr(MaterialDatabase::instance()->lookup(material));

  if (mat_ptr) {
    auto model_ptr(ShadingModelDatabase::instance()->lookup(
        mat_ptr->get_description().get_shading_model()));

    if (model_ptr) {

      for (int i(ShadingModel::GBUFFER_VERTEX_STAGE);
           i <= ShadingModel::FINAL_STAGE;
           ++i) {

        auto it(model_ptr->get_stages()[i].get_uniforms().find(uniform));
        if (it != model_ptr->get_stages()[i].get_uniforms().end()) {

          UniformType type(it->second);
          mapping_[material][uniform] =
              std::make_pair("gua_" + enums::uniform_type_to_string(type) + "s",
                             uniform_counts_[type]);
          ++uniform_counts_[type];
          break;
        }
      }

    }
  }
}

std::pair<std::string, int> const& UniformMapping::get_mapping(
    std::string const& material,
    std::string const& uniform) const {
  auto mat_it(mapping_.find(material));

  if (mat_it != mapping_.end()) {

    auto result(mat_it->second.find(uniform));

    if (result != mat_it->second.end()) {
      return result->second;
    }
  }

  Logger::LOG_WARNING << "The uniform " << uniform << " for material " << material << " is not mapped!" << std::endl;

  return error_;
}

std::string const UniformMapping::get_uniform_definition(Pipeline::PipelineStage stage) const {

  std::stringstream result;

  result << "layout(binding = " << static_cast<int>(stage)+1 << ") buffer GuaMaterialUniformStorage" << std::endl;
  result << "{" << std::endl;

  for (unsigned t(0); t < static_cast<unsigned>(UniformType::NONE); ++t) {
    int count(get_uniform_count(UniformType(t)));

    if (count > 0) {
      if (UniformType(t) == UniformType::SAMPLER2D) {
        std::string type(enums::uniform_type_to_string(UniformType(t)));
        result << "uvec2 gua_" << type << "s[" << count << "];" << std::endl;
      } else {
        std::string type(enums::uniform_type_to_string(UniformType(t)));
        result << type << " gua_" << type << "s[" << count << "];" << std::endl;
      }
    }
  }

  result << "} gua_material_uniforms;" << std::endl;

  return result.str();
}

int UniformMapping::get_uniform_count(UniformType type) const {
  auto count(uniform_counts_.find(type));

  if (count != uniform_counts_.end())
    return count->second;

  return 0;
}

std::map<UniformType, int> const& UniformMapping::get_uniform_counts() const {
  return uniform_counts_;
}

}
