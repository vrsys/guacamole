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
#include <gua/renderer/LayerMapping.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

LayerMapping::LayerMapping() {}

////////////////////////////////////////////////////////////////////////////////

LayerMapping::LayerMapping(std::set<std::string> const& materials,
                           ShadingModel::StageID stage)
    : stage_(stage) {

    // list of all used shading models
    std::set<std::string> shading_models;
    for (auto const& material : materials) {
        auto const& material_ptr(MaterialDatabase::instance()->lookup(material));
        auto const& shading_model(material_ptr->get_description().get_shading_model());
        shading_models.insert(shading_model);
    }

    for (auto const& model : shading_models)
        store_defaults(model);
}

////////////////////////////////////////////////////////////////////////////////

void LayerMapping::add(std::string const & shading_model_name,
                       std::string const & layer_name,
                       BufferComponent layer_type) {

    // those outputs are stored by default, as they've got reserved layers
    if (layer_name == "gua_position" || layer_name == "gua_normal" ||
        layer_name == "gua_color" || layer_name == "material_id") {

        return;
    }

    auto* mapping = &integer_mapping_;
    switch (enums::get_type(layer_type)) {
        case TYPE_UNSIGNED:
          mapping = &unsigned_mapping_;
          break;
        case TYPE_HALF:
          mapping = &half_mapping_;
          break;
        case TYPE_FLOAT:
          mapping = &float_mapping_;
          break;
    }

    bool found_slot(false);

    // search through existing layers for some empty slot
    // layer := std::unordered_map<std::string, std::unordered_map<std::string,
    // BufferComponent>>

    for (auto& layer : *mapping) {
        std::vector<BufferComponent> existing_outputs;

        // outputs := std::pair<std::string, std::unordered_map<std::string,
        // BufferComponent>>
        auto outputs(layer.find(shading_model_name));

        if (outputs != layer.end()) {
            // output := std::pair<std::string, BufferComponent>
            for (auto const& output : outputs->second) {
                existing_outputs.push_back(output.second);
            }
        }

        existing_outputs.push_back(layer_type);

        if (combine_outputs(existing_outputs) != BufferComponent::NONE) {
            layer[shading_model_name]
                .push_back(std::make_pair(layer_name, layer_type));
            found_slot = true;
            break;
        }
    }

    if (!found_slot) {
        mapping->push_back(std::unordered_map<
            std::string,
            std::vector<std::pair<std::string, BufferComponent> > >());
        (*mapping)[mapping->size() - 1][shading_model_name]
            .push_back(std::make_pair(layer_name, layer_type));
    }
}

////////////////////////////////////////////////////////////////////////////////

std::string const LayerMapping::get_output_string(
    std::string const & shading_model_name,
    std::string const & output_name) const {

    if (output_name == "gua_position") {
        return "gua_position_varying";
    }

    auto model_ptr(ShadingModelDatabase::instance()->lookup(shading_model_name));

    if (!model_ptr) {
      Logger::LOG_WARNING << "Failed to retrieve shadel model from database " << output_name.c_str() <<
        "defined in shading model " << shading_model_name.c_str() << std::endl;
      return "";
    }

    auto output_type(BufferComponent::NONE);

    auto output(model_ptr->get_stages()[stage_]
                    .get_outputs().find(output_name));
    if (output != model_ptr->get_stages()[stage_].get_outputs().end())
        output_type = output->second;

    if (output_type == BufferComponent::NONE) {
        Logger::LOG_WARNING << "Failed to generate gbuffer output string: There is no output "
                "\"" << output_name << "\" defined in shading model \"" << shading_model_name << "\"!" << std::endl;
        return "";
    }

    std::string type("int");
    auto* mapping = &integer_mapping_;

    switch (enums::get_type(output_type)) {
        case TYPE_UNSIGNED:
          mapping = &unsigned_mapping_;
          type = "uint";
          break;
        case TYPE_HALF:
          mapping = &half_mapping_;
          type = "half";
          break;
        case TYPE_FLOAT:
          mapping = &float_mapping_;
          type = "float";
          break;
    }

    for (unsigned i(0); i < mapping->size(); ++i) {

        auto model((*mapping)[i].find(shading_model_name));

        if (model != (*mapping)[i].end()) {

            std::vector<BufferComponent> preceeding_outputs;

            for (auto const& output : model->second) {
                if (output.first == output_name) {

                    unsigned length(
                        enums::get_number_of_components(output.second));
                    unsigned offset(enums::get_number_of_components(
                        combine_outputs(preceeding_outputs)));

                    std::stringstream result;

                    if (stage_ == ShadingModel::GBUFFER_VERTEX_STAGE)
                        result << "gua_" << type << "_gbuffer_varying_" << i
                               << ".";
                    else
                        result << "gua_" << type << "_gbuffer_out_" << i << ".";

                    std::vector<char> swizzle({
                      'x', 'y', 'z', 'w'
                    });

                    for (unsigned s(offset); s < offset + length; ++s) {
                        result << swizzle[s];
                    }

                    return result.str();

                } else {
                    preceeding_outputs.push_back(output.second);
                }
            }
        }
    }

    Logger::LOG_WARNING << "Failed to generate gbuffer output string: The output \"" << output_name << "\" for "
            "shading model \"" << shading_model_name << "\" is not mapped!" << std::endl;

    return "";
}

////////////////////////////////////////////////////////////////////////////////

std::string const LayerMapping::get_input_string(
    std::string const & shading_model_name,
    std::string const & input_name,
    ShadingModel::StageID from_stage) const {

    if (input_name == "gua_position") {
        return "gua_get_position()";
    }

    auto model_ptr(ShadingModelDatabase::instance()->lookup(shading_model_name));
    auto input_type(BufferComponent::NONE);

    auto input(model_ptr->get_stages()[stage_].get_outputs().find(input_name));
    if (input != model_ptr->get_stages()[stage_].get_outputs().end())
        input_type = input->second;

    if (input_type == BufferComponent::NONE) {
        Logger::LOG_WARNING << "Failed to generate gbuffer input string: There is no input "
                "\"" << input_name << "\" defined in shading model \"" << shading_model_name << "\" in stage " << stage_ << "!" << std::endl;
        return "";
    }

    std::string type("int");
    auto* mapping = &integer_mapping_;

    switch (enums::get_type(input_type)) {
        case TYPE_UNSIGNED:
          mapping = &unsigned_mapping_;
          type = "uint";
          break;
        case TYPE_HALF:
          mapping = &half_mapping_;
          type = "half";
          break;
        case TYPE_FLOAT:
          mapping = &float_mapping_;
          type = "float";
          break;
    }

    for (unsigned i(0); i < mapping->size(); ++i) {

        auto model((*mapping)[i].find(shading_model_name));

        if (model != (*mapping)[i].end()) {

            std::vector<BufferComponent> preceeding_outputs;

            for (auto const& output : model->second) {
                if (output.first == input_name) {

                    unsigned length(
                        enums::get_number_of_components(output.second));
                    unsigned offset(enums::get_number_of_components(
                        combine_outputs(preceeding_outputs)));

                    std::stringstream result;

                    if (from_stage == ShadingModel::GBUFFER_VERTEX_STAGE)
                        result << "gua_" << type << "_gbuffer_varying_" << i
                               << ".";
                    else
                        result << "texture2D(gua_get_" << type
                               << "_sampler(gua_" << type << "_gbuffer_in_"
                               << from_stage << "[" << i
                               << "]), gua_get_quad_coords()).";


                    std::vector<char> swizzle({
                      'x', 'y', 'z', 'w'
                    });

                    for (unsigned s(offset); s < offset + length; ++s) {
                        result << swizzle[s];
                    }

                    return result.str();

                } else {
                    preceeding_outputs.push_back(output.second);
                }
            }
        }
    }

    Logger::LOG_WARNING << "Failed to generate gbuffer input string: The input \"" << input_name << "\" for "
            "shading model \"" << shading_model_name << "\" is not mapped!" << std::endl;

    return "";
}

////////////////////////////////////////////////////////////////////////////////

std::string const LayerMapping::get_gbuffer_input_definition(
    ShadingModel::StageID from_stage) const {

    std::stringstream result;

    auto print = [&](BufferComponentType type, std::string const & type_name) {
        auto layers(get_layers(type));
        if (layers.size() > 0) {
            result << "uniform uvec2 gua_" << type_name << "_gbuffer_in_"
                   << from_stage << "[" << layers.size() << "];" << std::endl;
        }
    }
    ;

    print(TYPE_INTEGER, "int");
    print(TYPE_UNSIGNED, "uint");
    print(TYPE_HALF, "half");
    print(TYPE_FLOAT, "float");

    return result.str();
}

////////////////////////////////////////////////////////////////////////////////

std::string const LayerMapping::get_gbuffer_output_definition(
    bool as_input, bool as_varying) const {

    unsigned layout_location(0);
    std::string in_out(as_input ? "in" : "out");

    std::stringstream result;

    auto print_layout_string = [&]()->std::string {
        if (as_varying)
          return "";

        std::stringstream tmp;
        tmp << "layout(location=" << layout_location++ << ") ";
        return tmp.str();
    }
    ;

    auto print = [&](BufferComponentType type,
                     std::string const & name,
                     std::string const & flat) {
        auto layers(get_layers(type));
        if (layers.size() > 0) {
            for (unsigned l(0); l < layers.size(); ++l) {

                std::string infix(as_varying ? "varying" : "out");
                result << print_layout_string() << flat << in_out << " "
                       << enums::output_type_to_string(layers[l].first)
                       << " gua_" << name << "_gbuffer_" << infix << "_" << l
                       << ";" << std::endl;
            }
        }
    }
    ;

    print(TYPE_INTEGER, "int", as_varying ? "flat " : "");
    print(TYPE_UNSIGNED, "uint", as_varying ? "flat " : "");
    print(TYPE_HALF, "half", "");
    print(TYPE_FLOAT, "float", "");

    return result.str();
}

////////////////////////////////////////////////////////////////////////////////

std::vector<
    std::pair<BufferComponent,
              scm::gl::sampler_state_desc> > const LayerMapping::get_layers(
    BufferComponentType type) const {

    std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc> >
        result;
    auto* mapping = &integer_mapping_;

    switch (type) {
        case TYPE_UNSIGNED:
          mapping = &unsigned_mapping_;
          break;
        case TYPE_HALF:
          mapping = &half_mapping_;
          break;
        case TYPE_FLOAT:
          mapping = &float_mapping_;
          break;
    }

    // layer := std::unordered_map<std::string, std::unordered_map<std::string,
    // BufferComponent>>
    for (auto const& layer : *mapping) {
        std::vector<BufferComponent> output_sums;

        // model := std::pair<std::string, std::unordered_map<std::string,
        // BufferComponent>>
        for (auto const& model : layer) {
            std::vector<BufferComponent> outputs_per_model;

            // output := std::pair<std::string, BufferComponent>
            for (auto const& output : model.second) {
                outputs_per_model.push_back(output.second);
            }

            output_sums.push_back(combine_outputs(outputs_per_model));
        }

        scm::gl::sampler_state_desc state(scm::gl::FILTER_ANISOTROPIC,
                                          scm::gl::WRAP_MIRRORED_REPEAT,
                                          scm::gl::WRAP_MIRRORED_REPEAT);

        result.push_back(std::make_pair(get_largest(output_sums), state));
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<std::pair<
    BufferComponent,
    scm::gl::sampler_state_desc> > const LayerMapping::get_layers() const {

    std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc> >
        result;

    auto add_type = [&](BufferComponentType type) {
        auto tmp(get_layers(type));
        result.insert(result.end(), tmp.begin(), tmp.end());
    }
    ;

    add_type(TYPE_INTEGER);
    add_type(TYPE_UNSIGNED);
    add_type(TYPE_HALF);
    add_type(TYPE_FLOAT);

    return result;
}

////////////////////////////////////////////////////////////////////////////////

ShadingModel::StageID LayerMapping::get_stage() const { return stage_; }

////////////////////////////////////////////////////////////////////////////////

BufferComponent LayerMapping::combine_outputs(
    std::vector<BufferComponent> const & outputs) const {

    if (outputs.size() == 0)
        return BufferComponent::NONE;

    BufferComponentType type(enums::get_type(outputs[0]));
    unsigned count(0);
    for (auto output : outputs) {
        if (enums::get_type(output) != type)
            return BufferComponent::NONE;

        count += enums::get_number_of_components(output);
    }

    if (count > 4)
        return BufferComponent::NONE;

    return enums::get_component(type, count);
}

////////////////////////////////////////////////////////////////////////////////

BufferComponent LayerMapping::get_largest(std::vector<BufferComponent> const &
                                          outputs) const {

    BufferComponent result(BufferComponent::NONE);

    int number_of_components(0);

    for (auto output : outputs) {
        if (enums::get_number_of_components(output) > number_of_components) {
            result = output;
            number_of_components = enums::get_number_of_components(output);
        }
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////

void LayerMapping::store_defaults(std::string const & shading_model_name) {

    if (stage_ == ShadingModel::GBUFFER_VERTEX_STAGE) {

        if (unsigned_mapping_.size() < 1)
            unsigned_mapping_.resize(1);

        unsigned_mapping_[0][shading_model_name]
            .push_back(std::make_pair("material_id", BufferComponent::U1));

    } else if (stage_ == ShadingModel::GBUFFER_FRAGMENT_STAGE) {

        if (float_mapping_.size() < 1)
            float_mapping_.resize(1);

        float_mapping_[0][shading_model_name]
            .push_back(std::make_pair("gua_normal", BufferComponent::F3));

        if (unsigned_mapping_.size() < 1)
            unsigned_mapping_.resize(1);

        unsigned_mapping_[0][shading_model_name]
            .push_back(std::make_pair("material_id", BufferComponent::U1));

    } else if (stage_ == ShadingModel::FINAL_STAGE) {

        if (float_mapping_.size() < 1)
            float_mapping_.resize(1);

        float_mapping_[0][shading_model_name]
            .push_back(std::make_pair("gua_color", BufferComponent::F3));
    }
}

}
