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
#include <gua/renderer/LodLoader.hpp>

#include <gua/utils/vis_settings.hpp>

// guacamole headers
#include <gua/utils.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/node/PLodNode.hpp>
#include <gua/node/MLodNode.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/TimeSeriesDataSetDatabase.hpp>
#include <gua/renderer/LodResource.hpp>

// external headers
#include <lamure/ren/dataset.h>
#include <lamure/ren/model_database.h>
#include <lamure/ren/policy.h>
#include <lamure/ren/ray.h>

#include <boost/algorithm/string.hpp>

namespace gua
{
/////////////////////////////////////////////////////////////////////////////
LodLoader::LodLoader() : _supported_file_extensions_model_file(), _supported_file_extensions_vis_file()
{
    _supported_file_extensions_model_file.insert("bvh");
    _supported_file_extensions_model_file.insert("BVH");

    _supported_file_extensions_vis_file.insert("vis");
    _supported_file_extensions_vis_file.insert("VIS");
}


/////////////////////////////////////////////////////////////////////////////
std::vector<std::shared_ptr<node::PLodNode>> LodLoader::load_lod_pointclouds_from_vis_file(std::string const& vis_file_name, unsigned flags)
{
    auto desc = std::make_shared<gua::MaterialShaderDescription>();
    auto material_shader(std::make_shared<gua::MaterialShader>("Lod_unshaded_material", desc));
    gua::MaterialShaderDatabase::instance()->add(material_shader);

    auto cached_nodes(load_lod_pointclouds_from_vis_file(vis_file_name, material_shader->make_new_material(), flags));

    if(!cached_nodes.empty())
    {
        return cached_nodes;
    }

    Logger::LOG_WARNING << "LodLoader::load_lod_pointclouds_from_vis_file() : unable to create Lod Nodes" << std::endl;
    return std::vector<std::shared_ptr<node::PLodNode>>();
}

/////////////////////////////////////////////////////////////////////////////
void parse_simulation_positions(std::string const& simulation_positions_file_path, std::shared_ptr<TimeSeriesDataSet> const& current_time_series_dataset) {
    std::cout << "simulation_position_file: " <<  simulation_positions_file_path << std::endl;


    std::ifstream in_simulation_positions_file(simulation_positions_file_path);

    int line_index = 0;
    std::string line_buffer = "";
    int num_simulations_positions_per_line = 0;


    while(std::getline(in_simulation_positions_file, line_buffer)) {
        std::istringstream in_sstream(line_buffer);

        if(0 == line_index) {
            in_sstream >> num_simulations_positions_per_line;
            std::cout << num_simulations_positions_per_line << std::endl;
            current_time_series_dataset->simulation_positions.resize(num_simulations_positions_per_line);
        } else {
            for(int position_index = 0; position_index < num_simulations_positions_per_line; ++position_index) {
                scm::math::vec3f current_simulation_position;

                in_sstream >> current_simulation_position[0];
                in_sstream >> current_simulation_position[1];
                in_sstream >> current_simulation_position[2];
                current_time_series_dataset->simulation_positions[position_index].push_back(current_simulation_position);
            }
        }

        ++line_index;
    }
    in_simulation_positions_file.close();

    current_time_series_dataset->num_simulation_positions_per_timestep = num_simulations_positions_per_line;
}


/////////////////////////////////////////////////////////////////////////////
std::vector<std::shared_ptr<node::PLodNode>> LodLoader::load_lod_pointclouds_from_vis_file(std::string const& vis_file_name, std::shared_ptr<Material> const& fallback_material, unsigned flags) {
    
    std::vector<std::string> model_files_to_load;

    float initial_max_surfel_radius = -1.0f;

    scm::math::mat4f fem_transform_matrix;

    std::vector<std::string> parsed_time_series_data_description;

    std::string simulation_positions_filename = "";

    try {
        if(!is_supported_vis_file(vis_file_name)) {
            throw std::runtime_error(std::string("Unsupported filetype: ") + vis_file_name);
        } else {
            std::ifstream in_vis_filestream(vis_file_name, std::ios::in);
            std::string line_buffer;

            //int model_count = 0;
            std::vector<std::shared_ptr<node::PLodNode>> loaded_point_cloud_models;
            while(std::getline(in_vis_filestream, line_buffer) ) {
                boost::trim(line_buffer);

                if(is_supported_model_file(line_buffer)) {
                    model_files_to_load.push_back(line_buffer);
                } else {
                    std::istringstream attribute_parser_stringstream(line_buffer);

                    std::string attribute_string_dummy_buffer("");
                    if(0 == line_buffer.rfind("max_radius:", 0)) {
                        attribute_parser_stringstream >> attribute_string_dummy_buffer;
                        attribute_parser_stringstream >> initial_max_surfel_radius;
                    }

                    if(0 == line_buffer.rfind("fem_to_pcl_transform:", 0)) {
                        attribute_parser_stringstream >> attribute_string_dummy_buffer;
                        for(int transform_matrix_element = 0; transform_matrix_element < 16; ++transform_matrix_element) {
                            attribute_parser_stringstream >> fem_transform_matrix[transform_matrix_element];
                        }
                    }

                    if(0 == line_buffer.rfind("simulation_positions_filename:", 0)) {
                        attribute_parser_stringstream >> attribute_string_dummy_buffer;
                        attribute_parser_stringstream >> simulation_positions_filename;
                    }

                }
            }

            

            in_vis_filestream.close();

            vis_settings settings;
            load_vis_settings(vis_file_name, settings);

            //assertions
            if (settings.provenance_ != 0) {
                if (settings.json_.size() > 0) {
                    if (settings.json_.substr(settings.json_.size()-5) != ".json") {
                        std::cout << "unsupported json file: " << settings.json_ << std::endl;
                        exit(-1);
                    } else {
                        std::cout << "Loading Provenance-Layout-Description from JSON: " << settings.json_ << std::endl;
                        lamure::ren::data_provenance::get_instance()->parse_json(settings.json_);
                    
                        std::cout << "Loading mapping file: " << std::endl;
                        std::cout << settings.fem_value_mapping_file_ << std::endl;




                        std::string line_buffer;
                        std::ifstream in_mapping_file_stream(settings.fem_value_mapping_file_);



                        while( std::getline(in_mapping_file_stream, line_buffer) ) {
                            boost::trim(line_buffer);
                            if (! (line_buffer.rfind("#", 0) == 0 ) ) {
                                std::cout << line_buffer << std::endl;

                                std::istringstream data_item_description_strstream(line_buffer);


                                std::shared_ptr<TimeSeriesDataSet> shared_time_series_dataset = std::make_shared<TimeSeriesDataSet>();
                

                                data_item_description_strstream >> shared_time_series_dataset->name; 
                                data_item_description_strstream >> shared_time_series_dataset->num_attributes;
                                data_item_description_strstream >> shared_time_series_dataset->num_timesteps; 
                                data_item_description_strstream >> shared_time_series_dataset->sequence_length; 

                                std::ifstream in_attribute_file(shared_time_series_dataset->name, std::ios::in | std::ios::binary | std::ios::ate);

                                size_t total_num_byte_in_file = in_attribute_file.tellg();

                                in_attribute_file.clear();
                                in_attribute_file.seekg(0, std::ios::beg);


                                std::size_t num_vertices_per_file = (total_num_byte_in_file / sizeof(float) ) / (shared_time_series_dataset->num_attributes * shared_time_series_dataset->num_timesteps );
                                std::size_t num_elements_to_read = total_num_byte_in_file / sizeof(float);
                                std::size_t num_elements_per_attribute_over_time = num_vertices_per_file * shared_time_series_dataset->num_timesteps;

                                shared_time_series_dataset->data.resize(num_elements_to_read);

                                in_attribute_file.read( (char*) shared_time_series_dataset->data.data(), total_num_byte_in_file);

                                //memcpy((char*) shared_time_series_dataset->data.data(), (char*) dummy_data.data(),dummy_data.size() * sizeof(float));

                                uint64_t total_num_attributes = shared_time_series_dataset->data.size() / num_elements_per_attribute_over_time;

                                shared_time_series_dataset->extreme_values.resize(total_num_attributes);

                                for(uint64_t attribute_idx = 0; attribute_idx < total_num_attributes; ++attribute_idx) {
                                    shared_time_series_dataset->extreme_values[attribute_idx].first = std::numeric_limits<float>::max();
                                    shared_time_series_dataset->extreme_values[attribute_idx].second = std::numeric_limits<float>::min();                        
                                }

                                for(uint32_t attribute_idx = 0; attribute_idx < total_num_attributes; ++attribute_idx) {
                                    uint32_t base_offset = num_elements_per_attribute_over_time * attribute_idx;
                                    
                                    for(uint32_t element_idx = 0; element_idx < num_elements_per_attribute_over_time; ++element_idx) {

                                        shared_time_series_dataset->extreme_values[attribute_idx].first = std::min(shared_time_series_dataset->extreme_values[attribute_idx].first,
                                                                                                       shared_time_series_dataset->data[base_offset + element_idx]);
                                        shared_time_series_dataset->extreme_values[attribute_idx].second = std::max(shared_time_series_dataset->extreme_values[attribute_idx].second,
                                                                                                       shared_time_series_dataset->data[base_offset + element_idx]);
                                    }
                                }

                                in_attribute_file.close();


                                if(!simulation_positions_filename.empty()) {

                                    std::string directory;
                                    std::size_t const last_slash_idx = shared_time_series_dataset->name.find_last_of("\\/");
                                    if (std::string::npos != last_slash_idx)
                                    {
                                        directory = shared_time_series_dataset->name.substr(0, last_slash_idx);
                                    }

                                    std::string simulation_positions_source = (directory) + "/" + simulation_positions_filename;
                                    parse_simulation_positions(simulation_positions_source,  shared_time_series_dataset);
                                }


                                TimeSeriesDataSetDatabase::instance()->add(shared_time_series_dataset->name, shared_time_series_dataset);

                                shared_time_series_dataset->simulation_positions_filename = simulation_positions_filename;
                                shared_time_series_dataset->time_series_transform_matrix = fem_transform_matrix;

                                parsed_time_series_data_description.push_back(shared_time_series_dataset->name);

                            }
                        }

                        in_mapping_file_stream.close();
                    }
                }

            } 

            

            // after the vis file was parsed, load all models and set the common properties

            int32_t model_index = 0;

            for(auto const& model_path : model_files_to_load) {

                std::size_t found = model_path.find_last_of("/\\");
                auto const automatic_node_name = model_path.substr(found+1);

                auto const& current_point_cloud_shared_ptr = load_lod_pointcloud(automatic_node_name, model_path, fallback_material, flags);
                    
                current_point_cloud_shared_ptr->set_time_series_data_descriptions(parsed_time_series_data_description);

                loaded_point_cloud_models.push_back(current_point_cloud_shared_ptr);

                if(initial_max_surfel_radius > 0.0f) {
                    current_point_cloud_shared_ptr->set_max_surfel_radius(initial_max_surfel_radius);
                }

                if(model_index < settings.num_models_with_provenance_) {
                    current_point_cloud_shared_ptr->set_has_provenance_attributes(true);
                }
    
                ++model_index;
            }

            return loaded_point_cloud_models;
        }
    } catch(std::exception& e) {
        Logger::LOG_WARNING << "Failed to parse vis file object \"" << vis_file_name << "\": " << e.what() << std::endl;
        return {};
    }
}


/////////////////////////////////////////////////////////////////////////////
std::shared_ptr<node::PLodNode> LodLoader::load_lod_pointcloud(std::string const& nodename, std::string const& filename, std::shared_ptr<Material> const& material, unsigned flags)
{
    try
    {
        if(!is_supported_model_file(filename))
        {
            throw std::runtime_error(std::string("Unsupported filetype: ") + filename);
        }
        else
        {
            lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
            Logger::LOG_WARNING << "Trying to load " << filename << std::endl;
            GeometryDescription desc("PLod", filename, 0, flags);

            lamure::model_t model_id = database->add_model(filename, desc.unique_key());

            if(database->get_model(model_id)->get_bvh()->get_primitive() != lamure::ren::bvh::primitive_type::POINTCLOUD)
            {
                Logger::LOG_WARNING << "Failed to load lod file \"" << filename << "\":"
                                    << ". Please use load_lod_trimesh for meshes" << std::endl;
                return nullptr;
            }

            math::mat4 local_transform = scm::math::make_translation(math::vec3(database->get_model(model_id)->get_bvh()->get_translation()));

            auto resource = std::make_shared<LodResource>(model_id, flags & LodLoader::MAKE_PICKABLE, local_transform);
            GeometryDatabase::instance()->add(desc.unique_key(), resource);

            auto node = std::make_shared<node::PLodNode>(nodename, desc.unique_key(), filename, material);

            node->update_cache();

            auto bbox = resource->get_bounding_box();

            // normalize position?
            auto normalize_position = flags & LodLoader::NORMALIZE_POSITION;
            if(normalize_position)
            {
                auto bbox_center_object_space = local_transform * math::vec4(bbox.center().x, bbox.center().y, bbox.center().z, 1.0);
                node->translate(-bbox_center_object_space.x, -bbox_center_object_space.y, -bbox_center_object_space.z);
            }

            // normalize scale?
            auto normalize_scale = flags & LodLoader::NORMALIZE_SCALE;
            if(normalize_scale)
            {
                auto scale = 1.0f / scm::math::length(bbox.max - bbox.min);
                node->scale(scale, scale, scale);
            }

            return node;
        }
    }
    catch(std::exception& e)
    {
        Logger::LOG_WARNING << "Failed to load Lod object \"" << filename << "\": " << e.what() << std::endl;
        return nullptr;
    }
}

/////////////////////////////////////////////////////////////////////////////
std::shared_ptr<node::MLodNode> LodLoader::load_lod_trimesh(std::string const& nodename, std::string const& filename, std::shared_ptr<Material> const& material, unsigned flags)
{
    try
    {
        if(!is_supported_model_file(filename))
        {
            throw std::runtime_error(std::string("Unsupported filetype: ") + filename);
        }
        else
        {
            lamure::ren::model_database* database = lamure::ren::model_database::get_instance();

            GeometryDescription desc("MLod", filename, 0, flags);

            lamure::model_t model_id = database->add_model(filename, desc.unique_key());
            if(database->get_model(model_id)->get_bvh()->get_primitive() != lamure::ren::bvh::primitive_type::TRIMESH)
            {
                Logger::LOG_WARNING << "Failed to load lod file \"" << filename << "\":"
                                    << ". Please use load_lod_pointcloud for pointclouds" << std::endl;
                return nullptr;
            }

            math::mat4 local_transform = scm::math::make_translation(math::vec3(database->get_model(model_id)->get_bvh()->get_translation()));

            auto resource = std::make_shared<LodResource>(model_id, flags & LodLoader::MAKE_PICKABLE, local_transform);
            GeometryDatabase::instance()->add(desc.unique_key(), resource);

            auto node = std::make_shared<node::MLodNode>(nodename, desc.unique_key(), filename, material);

            node->update_cache();

            auto bbox = resource->get_bounding_box();

            // normalize position?
            auto normalize_position = flags & LodLoader::NORMALIZE_POSITION;
            if(normalize_position)
            {
                auto bbox_center_object_space = local_transform * math::vec4(bbox.center().x, bbox.center().y, bbox.center().z, 1.0);
                node->translate(-bbox_center_object_space.x, -bbox_center_object_space.y, -bbox_center_object_space.z);
            }

            // normalize scale?
            auto normalize_scale = flags & LodLoader::NORMALIZE_SCALE;
            if(normalize_scale)
            {
                auto scale = 1.0f / scm::math::length(bbox.max - bbox.min);
                node->scale(scale, scale, scale);
            }

            return node;
        }
    }
    catch(std::exception& e)
    {
        Logger::LOG_WARNING << "Failed to load Lod object \"" << filename << "\": " << e.what() << std::endl;
        return nullptr;
    }
}

/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::PLodNode> LodLoader::load_lod_pointcloud(std::string const& filename, unsigned flags)
{
    auto desc = std::make_shared<gua::MaterialShaderDescription>();
    auto material_shader(std::make_shared<gua::MaterialShader>("Lod_unshaded_material", desc));
    gua::MaterialShaderDatabase::instance()->add(material_shader);

    auto cached_node(load_lod_pointcloud(filename, filename, material_shader->make_new_material(), flags));

    if(cached_node)
    {
        return cached_node;
    }

    Logger::LOG_WARNING << "LodLoader::load_lod_pointcloud() : unable to create Lod Node" << std::endl;
    return std::shared_ptr<node::PLodNode>(new node::PLodNode(filename));
}

/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::MLodNode> LodLoader::load_lod_trimesh(std::string const& filename, unsigned flags)
{
    auto desc = std::make_shared<gua::MaterialShaderDescription>();
    auto material_shader(std::make_shared<gua::MaterialShader>("MLod_unshaded_material", desc));
    gua::MaterialShaderDatabase::instance()->add(material_shader);

    auto cached_node(load_lod_trimesh(filename, filename, material_shader->make_new_material(), flags));

    if(cached_node)
    {
        return cached_node;
    }

    Logger::LOG_WARNING << "LodLoader::load_lod_trimesh() : unable to create Lod Node" << std::endl;
    return std::shared_ptr<node::MLodNode>(new node::MLodNode(filename));
}

////////////////////////////////////////////////////////////////////////////////

bool LodLoader::is_supported_model_file(std::string const& file_name) const
{
    std::vector<std::string> filename_decomposition = gua::string_utils::split(file_name, '.');
    return filename_decomposition.empty() ? false : _supported_file_extensions_model_file.count(filename_decomposition.back()) > 0;
}

bool LodLoader::is_supported_vis_file(std::string const& file_name) const
{
    std::vector<std::string> filename_decomposition = gua::string_utils::split(file_name, '.');
    return filename_decomposition.empty() ? false : _supported_file_extensions_vis_file.count(filename_decomposition.back()) > 0;
}

////////////////////////////////////////////////////////////////////////////////
void LodLoader::apply_fallback_material(std::shared_ptr<node::Node> const& root, std::shared_ptr<Material> const& fallback_material) const
{
    std::cout << "Trying to apply fallback material\n";
    // auto g_node(std::dynamic_pointer_cast<node::PLodNode>(root));

    /*
      if(g_node) {
        if (!g_node->get_material()) {
          g_node->set_material(fallback_material);
          g_node->update_cache();
        }
        else {
          if (g_node->get_material()->get_shader_name() == "")
          {
            g_node->set_material(fallback_material);
            g_node->update_cache();
          }
        }
      }

      for(auto& child : root->get_children()) {
        apply_fallback_material(child, fallback_material);
      }*/

    auto g_node(std::dynamic_pointer_cast<node::MLodNode>(root));

    if(g_node && !g_node->get_material())
    {
        g_node->set_material(fallback_material);
        g_node->update_cache();
    }
    else if(g_node /*&& no_shared_materials*/)
    {
        g_node->set_material(std::make_shared<Material>(*g_node->get_material()));
    }

    for(auto& child : root->get_children())
    {
        apply_fallback_material(child, fallback_material /*, no_shared_materials*/);
    }
}
/////////////////////////////////////////////////////////////////////////////////

std::pair<std::string, math::vec3>
LodLoader::pick_lod_bvh(math::vec3 const& ray_origin, math::vec3 const& ray_forward, float max_distance, std::set<std::string> const& model_filenames, float aabb_scale) const
{
    scm::math::vec3f ray_pos = scm::math::vec3f(ray_origin.x, ray_origin.y, ray_origin.z);
    scm::math::vec3f ray_fwd = scm::math::vec3f(ray_forward.x, ray_forward.y, ray_forward.z);

    std::pair<std::string, math::vec3> result = std::make_pair("", math::vec3::zero());

    lamure::ren::ray ray(ray_pos, ray_fwd, max_distance);
    lamure::ren::ray::intersection_bvh intersection;

    if(ray.intersect_bvh(model_filenames, aabb_scale, intersection))
    {
        result = std::make_pair(intersection.bvh_filename_, intersection.position_);
    }

    return result;
}

///////////////////////////////////////////////////////////////////////////////

std::set<PickResult> LodLoader::pick_lod_interpolate(math::vec3 const& bundle_origin,
                                                     math::vec3 const& bundle_forward,
                                                     math::vec3 const& bundle_up,
                                                     float bundle_radius,
                                                     float max_distance,
                                                     unsigned int max_depth,
                                                     unsigned int surfel_skip,
                                                     float aabb_scale) const
{
    std::set<PickResult> results;

    scm::math::vec3f ray_pos = scm::math::vec3f(bundle_origin.x, bundle_origin.y, bundle_origin.z);
    scm::math::vec3f ray_fwd = scm::math::vec3f(bundle_forward.x, bundle_forward.y, bundle_forward.z);
    scm::math::vec3f ray_up = scm::math::vec3f(bundle_up.x, bundle_up.y, bundle_up.z);

    lamure::ren::ray ray(ray_pos, ray_fwd, max_distance);
    lamure::ren::ray::intersection intersection;

    if(ray.intersect(aabb_scale, ray_up, bundle_radius, max_depth, surfel_skip, intersection))
    {
        PickResult result(intersection.distance_, nullptr, math::vec3(intersection.error_), math::vec3(intersection.position_), math::vec3(), math::vec3(intersection.normal_), math::vec2());
        results.insert(result);
    }

    return results;
}

////////////////////////////////////////////////////////////////////////////////

void LodLoader::set_upload_budget_in_mb(const size_t upload_budget)
{
    lamure::ren::policy* policy = lamure::ren::policy::get_instance();
    policy->set_max_upload_budget_in_mb(upload_budget);
}

////////////////////////////////////////////////////////////////////////////////

void LodLoader::set_render_budget_in_mb(const size_t render_budget)
{
    lamure::ren::policy* policy = lamure::ren::policy::get_instance();
    policy->set_render_budget_in_mb(render_budget);
}

////////////////////////////////////////////////////////////////////////////////

void LodLoader::set_out_of_core_budget_in_mb(const size_t out_of_core_budget)
{
    lamure::ren::policy* policy = lamure::ren::policy::get_instance();
    policy->set_out_of_core_budget_in_mb(out_of_core_budget);
}

////////////////////////////////////////////////////////////////////////////////

size_t LodLoader::get_upload_budget_in_mb() const
{
    lamure::ren::policy* policy = lamure::ren::policy::get_instance();
    return policy->max_upload_budget_in_mb();
}

////////////////////////////////////////////////////////////////////////////////

size_t LodLoader::get_render_budget_in_mb() const
{
    lamure::ren::policy* policy = lamure::ren::policy::get_instance();

    return policy->render_budget_in_mb();
}

////////////////////////////////////////////////////////////////////////////////
size_t LodLoader::get_out_of_core_budget_in_mb() const
{
    lamure::ren::policy* policy = lamure::ren::policy::get_instance();
    return policy->out_of_core_budget_in_mb();
}

////////////////////////////////////////////////////////////////////////////////
math::mat4 LodLoader::_load_local_transform(std::string const& filename) const
{
    auto tf_filename = boost::filesystem::change_extension(filename, "tf");

    if(boost::filesystem::exists(tf_filename))
    {
        // try to load matrix
        std::array<math::float_t, 16> input_mat;
        bool load_matrix_success = false;

        try
        {
            std::ifstream fstr(tf_filename.c_str(), std::ios::in);
            for(unsigned i = 0; i != input_mat.size(); ++i)
            {
                fstr >> input_mat[i];
            }
            fstr.close();
            load_matrix_success = true;
        }
        catch(...)
        {
        }

        if(load_matrix_success)
        {
            return math::mat4(input_mat[0],
                              input_mat[4],
                              input_mat[8],
                              input_mat[12],
                              input_mat[1],
                              input_mat[5],
                              input_mat[9],
                              input_mat[13],
                              input_mat[2],
                              input_mat[6],
                              input_mat[10],
                              input_mat[14],
                              input_mat[3],
                              input_mat[7],
                              input_mat[11],
                              input_mat[15]);
        }
    }

    // loading failed -> return identity
    return math::mat4::identity();
}

} // namespace gua
