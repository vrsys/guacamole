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
#include <gua/renderer/UberShaderFactory.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

UberShaderFactory::UberShaderFactory(
    ShadingModel::StageID stage,
    std::set<std::string> const& material_names,
    UniformMapping const& uniform_mapping)
    : stage_(stage),
      uniform_mapping_(uniform_mapping),
      output_mapping_(material_names, stage) {

    // list of all materials
    std::set<std::shared_ptr<Material> > materials;

    // list of all used shading models
    std::set<std::shared_ptr<ShadingModel> > shading_models;

    // fill material list
    for (auto const& mat : material_names) {
        materials.insert(MaterialDatabase::instance()->lookup(mat));
    }

    // fill shading model list and create uniform mappings
    for (auto const& mat : materials) {
        auto shading_model(ShadingModelDatabase::instance()->lookup(
            mat->get_description().get_shading_model()));
        shading_models.insert(shading_model);

        // add an uniform mapping for each uniform of the current material and
        // shader stage
        for (auto const& uniform : shading_model->get_stages()[stage].get_uniforms()) {
            uniform_mapping_.add(mat->get_name(), uniform.first);
        }

        // store material specific main method calls
        std::stringstream call;
        call << "/* " << mat->get_name() << " */ ";

        auto name(shading_model->get_name());
        std::replace(name.begin(), name.end(), '/',  '_');
        std::replace(name.begin(), name.end(), '\\', '_');
        std::replace(name.begin(), name.end(), '.',  '_');
        std::replace(name.begin(), name.end(), '(',  '_');
        std::replace(name.begin(), name.end(), ')',  '_');
        std::replace(name.begin(), name.end(), ' ',  '_');

        call << name << "_main(";

        auto uniform(shading_model->get_stages()[stage].get_uniforms().begin());

        while (uniform !=
               shading_model->get_stages()[stage].get_uniforms().end()) {

            auto mapped(
                uniform_mapping_.get_mapping(mat->get_name(), uniform->first));

            if (uniform->second == UniformType::SAMPLER2D)
                call << "gua_get_float_sampler(gua_material_uniforms." << mapped.first << "["
                     << mapped.second << "])";
            else
                call << "gua_material_uniforms." << mapped.first << "[" << mapped.second << "]";


            if (++uniform !=
                shading_model->get_stages()[stage].get_uniforms().end())
                call << ", ";
        }

        call << ");";

        main_calls_[mat->get_id()] = call.str();
    }

    // fill output_mapping
    for (auto const& model : shading_models) {
      for (auto const& output : model->get_stages()[stage].get_outputs()) {
            output_mapping_.add(model->get_name(), output.first, output.second);
        }
    }

    // store functions
    for (auto const& model : shading_models) {
        load_main_functions(model);
        load_custom_functions(model);
    }
}

////////////////////////////////////////////////////////////////////////////////

void UberShaderFactory::add_inputs_to_main_functions(
    std::vector<LayerMapping const*> const & inputs,
    ShadingModel::StageID first_input_stage) {

    // add gua_position to all functions
    if (first_input_stage != ShadingModel::GBUFFER_VERTEX_STAGE) {
        for (auto& entry : main_functions_) {
            auto model(ShadingModelDatabase::instance()->lookup(entry.first));
            auto bracket_pos = entry.second.find("{");
            std::stringstream stream;
            stream << std::endl;
            stream << "    vec3 gua_position = gua_get_position();"
                   << std::endl;

            entry.second.insert(bracket_pos + 1, stream.str());
        }
    }

    for (unsigned i(0); i < inputs.size(); ++i) {
        ShadingModel::StageID from_stage(
            (ShadingModel::StageID)(i + first_input_stage));

        for (auto& entry : main_functions_) {

            auto model(ShadingModelDatabase::instance()->lookup(entry.first));

            auto bracket_pos = entry.second.find("{");
            std::stringstream stream;
            stream << std::endl;

            for (auto const& input : model->get_stages()[from_stage].get_outputs()) {
                stream << "    " << enums::output_type_to_string(input.second)
                       << " " << input.first << " = "
                       << inputs[i]->get_input_string(
                              model->get_name(), input.first, from_stage) << ";"
                       << std::endl;
            }

            entry.second.insert(bracket_pos + 1, stream.str());
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

UniformMapping const& UberShaderFactory::get_uniform_mapping() const {
    return uniform_mapping_;
}

////////////////////////////////////////////////////////////////////////////////

LayerMapping const& UberShaderFactory::get_output_mapping() const {
    return output_mapping_;
}

////////////////////////////////////////////////////////////////////////////////

std::unordered_map<std::string, std::string> const&
UberShaderFactory::get_main_functions() const {
    return main_functions_;
}

////////////////////////////////////////////////////////////////////////////////

std::unordered_map<int, std::string> const&
UberShaderFactory::get_main_calls() const {
    return main_calls_;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<std::string> const&
UberShaderFactory::get_custom_functions() const {
    return custom_functions_;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<std::string> const&
UberShaderFactory::get_custom_function_declares() const {
    return custom_function_declares_;
}

////////////////////////////////////////////////////////////////////////////////

void UberShaderFactory::load_main_functions(std::shared_ptr<ShadingModel> const& model) {


    // write main() method <model>_main(<uniforms>) ----------------------------
    std::stringstream stream;

    auto name(model->get_name());
    std::replace(name.begin(), name.end(), '/',  '_');
    std::replace(name.begin(), name.end(), '\\', '_');
    std::replace(name.begin(), name.end(), '.',  '_');
    std::replace(name.begin(), name.end(), '(',  '_');
    std::replace(name.begin(), name.end(), ')',  '_');
    std::replace(name.begin(), name.end(), ' ',  '_');

    stream << "void " << name << "_main(";

    auto uniform(model->get_stages()[stage_].get_uniforms().begin());

    while (uniform != model->get_stages()[stage_].get_uniforms().end()) {
        stream << enums::uniform_type_to_string(uniform->second) << " "
               << uniform->first;

        if (++uniform != model->get_stages()[stage_].get_uniforms().end())
            stream << ", ";
    }

    std::string body(model->get_stages()[stage_].get_body());

    stream << ") {" << std::endl;

    // add output variables to the beginning of the main method ----------------
    for (auto const& output : model->get_stages()[stage_].get_outputs()) {
        stream << "    " << enums::output_type_to_string(output.second) << " "
               << output.first << " = " << enums::output_type_to_string(output.second) << "(0);" << std::endl;
    }

    // write body --------------------------------------------------------------
    stream << body << std::endl;

    // write output variables to gua_out_layer_* at end of the main method -----
    for (auto const& output : model->get_stages()[stage_].get_outputs()) {
        stream << "    " << output_mapping_.get_output_string(
                                model->get_name(), output.first) << " = "
               << output.first << ";" << std::endl;
    }

    stream << "}" << std::endl;

    main_functions_[model->get_name()] = stream.str();
}

////////////////////////////////////////////////////////////////////////////////

void UberShaderFactory::load_custom_functions(std::shared_ptr<ShadingModel> const& model) {

    std::string functions(model->get_stages()[stage_].get_functions());

    // get signature of each custom function
    // ...

    custom_functions_.push_back(functions);

}

}
