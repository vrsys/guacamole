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
#include <gua/renderer/TemporalSVOResource.hpp>

#include <gua/utils/Singleton.hpp>
#include <gua/node/TemporalSVONode.hpp>

#include <scm/gl_core/render_device.h>
#include <scm/gl_core/buffer_objects.h>
#include <scm/gl_core/shader_objects.h>
#include <scm/gl_core/texture_objects.h>
#include <scm/gl_core/render_device/opengl/util/assert.h>
#include <scm/gl_core/data_formats.h>
#include <scm/gl_core/constants.h>

#include <scm/gl_util/data/volume/volume_loader.h>

#include <boost/assign/list_of.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/node/Node.hpp>

//#include <pbr/ren/ray.h>
//#include <pbr/ren/controller.h>

// external headers
#include <stack>
#include <algorithm>
#include <regex>
#include <fstream>

namespace gua
{
std::mutex TemporalSVOResource::cpu_volume_loading_mutex_;
std::map<std::size_t, bool> TemporalSVOResource::are_cpu_time_steps_loaded_;
std::map<std::size_t, std::map<std::string, uint64_t>> TemporalSVOResource::volume_descriptor_tokens_;
std::map<std::size_t, std::vector<std::ifstream>> TemporalSVOResource::per_resource_file_streams_;
std::map<std::size_t, std::vector<std::vector<uint8_t>>> TemporalSVOResource::per_resource_cpu_cache_;

////////////////////////////////////////////////////////////////////////////////

void TemporalSVOResource::tokenize_volume_name(std::string const& string_to_split, std::map<std::string, uint64_t>& tokens)
{
    size_t file_starting_pos = string_to_split.find_last_of("/") + 1;
    std::string file_name = string_to_split.substr(file_starting_pos);
    std::string extension_removed_string = file_name.substr(0, file_name.find(".raw"));
    extension_removed_string = extension_removed_string.substr(0, extension_removed_string.find(".bin"));
    extension_removed_string = extension_removed_string.substr(0, extension_removed_string.find(".dp"));

    std::stringstream test(extension_removed_string);
    std::string segment;

    std::regex non_negative_number_regex("[[:digit:]]+");
    std::vector<std::string> registered_tokens;

    registered_tokens.push_back("w");
    registered_tokens.push_back("ow");
    registered_tokens.push_back("h");
    registered_tokens.push_back("oh");
    registered_tokens.push_back("d");
    registered_tokens.push_back("od");
    registered_tokens.push_back("cn");
    registered_tokens.push_back("c");
    registered_tokens.push_back("b");
    registered_tokens.push_back("t");
    registered_tokens.push_back("bs");
    registered_tokens.push_back("i");
    registered_tokens.push_back("p");
    registered_tokens.push_back("sy");
    registered_tokens.push_back("a");
    registered_tokens.push_back("si");
    registered_tokens.push_back("norm");
    registered_tokens.push_back("min");
    registered_tokens.push_back("max");
    registered_tokens.push_back("fb");

    while(std::getline(test, segment, '_'))
    {
        for(auto const& potentially_matching_token : registered_tokens)
        {
            if(segment.find(potentially_matching_token) == 0)
            {
                uint64_t length_of_token = potentially_matching_token.size();
                std::string remaining_string = segment.substr(length_of_token);

                bool regex_match_result = std::regex_match(remaining_string, non_negative_number_regex);
                if(regex_match_result)
                {
                    tokens[potentially_matching_token] = std::atoi(remaining_string.c_str());
                    break;
                }
            }
        }
    }

    tokens["num_voxels"] = tokens["w"] * tokens["h"] * tokens["d"];
    tokens["num_bytes_per_voxel"] = tokens["b"] / 8;
    tokens["total_num_bytes"] = tokens["num_voxels"] * tokens["num_bytes_per_voxel"];
}

////////////////////////////////////////////////////////////////////////////////

TemporalSVOResource::TemporalSVOResource(std::string const& resource_file_string, bool is_pickable)
    : resource_file_name_(resource_file_string), is_pickable_(is_pickable),
      local_transform_(gua::math::mat4(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0))
{
    std::cout << "Created Uncompressed Volume Resource\n";
    std::cout << "Loading once\n";
    bounding_box_.min = scm::math::vec3(0.0f, 0.0f, 0.0f);
    bounding_box_.max = scm::math::vec3(1.0f, 1.0f, 1.0f);

    cpu_volume_loading_mutex_.lock();

    if(!are_cpu_time_steps_loaded_[uuid_])
    {
        std::vector<std::string> volumes_to_load;
        if(resource_file_name_.find(".v_rsc") != std::string::npos)
        {
            std::string line_buffer;
            std::ifstream volume_resource_file(resource_file_name_, std::ios::in);
            while(std::getline(volume_resource_file, line_buffer))
            {
                if(line_buffer.find(".raw") != std::string::npos)
                {
                    volumes_to_load.push_back(line_buffer);
                }
            }
        }
        else
        {
            volumes_to_load.push_back(resource_file_name_);
        }

        uint8_t num_parsed_volumes = 0;
        for(auto const& vol_path : volumes_to_load)
        {
            if(0 == num_parsed_volumes++)
            {
                tokenize_volume_name(vol_path, volume_descriptor_tokens_[uuid_]);
            }

            per_resource_file_streams_[uuid_].push_back(std::ifstream(vol_path.c_str(), std::ios::in | std::ios::binary));
        }
        are_cpu_time_steps_loaded_[uuid_] = true;
    }
    cpu_volume_loading_mutex_.unlock();
}

////////////////////////////////////////////////////////////////////////////////

TemporalSVOResource::~TemporalSVOResource() {}

////////////////////////////////////////////////////////////////////////////////

void TemporalSVOResource::draw(RenderContext const& ctx, scm::gl::vertex_array_ptr const& vertex_array) const
{
    // dummy
}

void TemporalSVOResource::apply_resource_dependent_uniforms(RenderContext const& ctx, std::shared_ptr<ShaderProgram> const& current_program) const {

};

void TemporalSVOResource::upload_to(RenderContext const& ctx) const
{
    if(are_cpu_time_steps_loaded_[uuid_])
    {
        int32_t loaded_volumes_count = 0;

        auto& current_tokens = volume_descriptor_tokens_[uuid_];

        for(auto& vol_path : per_resource_file_streams_[uuid_])
        {
            scm::math::vec3ui vol_dims = scm::math::vec3ui(current_tokens["w"], current_tokens["h"], current_tokens["d"]);

            int64_t num_bytes_per_voxel = current_tokens["num_bytes_per_voxel"];

            int64_t num_voxels = current_tokens["total_num_bytes"];

            // std::vector<unsigned char> read_buffer(num_voxels);

            per_resource_cpu_cache_[uuid_].push_back(std::vector<uint8_t>(num_voxels, 0));
            vol_path.read((char*)&per_resource_cpu_cache_[uuid_][loaded_volumes_count][0], num_voxels);

            scm::gl::data_format read_format = scm::gl::data_format::FORMAT_NULL;

            /*volume_textures_.push_back(ctx.render_device->create_texture_3d(scm::gl::texture_3d_desc(vol_dims, read_format), read_format,
                                                                            {(void*) &per_resource_cpu_cache_[uuid_][loaded_volumes_count][0]} ) );
            */
            ctx.texture_3d_arrays[uuid()].push_back(
                ctx.render_device->create_texture_3d(scm::gl::texture_3d_desc(vol_dims, read_format), read_format, {(void*)&per_resource_cpu_cache_[uuid_][loaded_volumes_count][0]}));
            ++loaded_volumes_count;
        }

        num_time_steps_ = loaded_volumes_count;
        /*

        std::make_shared<scm::gl::texture_3d>(
                                                                new scm::gl::texture_3d(*ctx.render_device,
                                                                    scm::gl::texture_3d_desc(scm::math::vec3ui(256,256,225),
                                                                                        scm::gl::data_format::FORMAT_R_8)  );
      */
    }
}

void TemporalSVOResource::bind_volume_texture(RenderContext const& ctx, scm::gl::sampler_state_ptr const& sampler_state) const
{
    auto iter = ctx.texture_3d_arrays.find(uuid());
    if(iter == ctx.texture_3d_arrays.end())
    {
        upload_to(ctx);
        iter = ctx.texture_3d_arrays.find(uuid());
    }
    using namespace std::chrono;

    high_resolution_clock::time_point current_time_point = high_resolution_clock::now();

    double elapsed_seconds = duration_cast<duration<double>>(current_time_point - last_time_point_).count();

    int playback_multiplier = 1;

    if(PlaybackMode::BACKWARD == playback_mode_)
    {
        playback_multiplier = -1;
    }

    if(PlaybackMode::NONE != playback_mode_)
    {
        time_cursor_pos_ += elapsed_seconds * playback_fps_ * playback_multiplier;
    }

    last_time_point_ = current_time_point;

    while(time_cursor_pos_ < 0.0)
    {
        time_cursor_pos_ += num_time_steps_;
    }

    int32_t volume_id = int32_t(time_cursor_pos_) % num_time_steps_;

    // ctx.render_context->bind_texture(volume_textures_[ ((frame_counter_++) / 10) % volume_textures_.size()], sampler_state, 0);
    ctx.render_context->bind_texture((iter->second)[volume_id], sampler_state, 0);

    /*
      std::cout << "In drawing branch\n";
      scm::gl::context_vertex_input_guard vig(ctx.render_context);

      ctx.render_context->apply();
      std::cout << "Before drawing\n";
      volume_proxy_.draw(ctx.render_context);
    */
}

////////////////////////////////////////////////////////////////////////////////
math::mat4 const& TemporalSVOResource::local_transform() const
{
    // return scm::math::mat4d(1.0);
    return local_transform_;
}

////////////////////////////////////////////////////////////////////////////////

void TemporalSVOResource::ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits)
{
    /*
      if (!is_pickable_)
        return;

      bool has_hit = false;
      PickResult pick = PickResult(
        0.f,
        owner,
        ray.origin_,
        math::vec3(0.f, 0.f, 0.f),
        math::vec3(0.f, 1.f, 0.f),
        math::vec3(0.f, 1.f, 0.f),
        math::vec2(0.f, 0.f));

      const auto model_transform = owner->get_cached_world_transform();
      const auto world_origin = ray.origin_;
      const auto world_direction = scm::math::normalize(ray.direction_);

      pbr::ren::Ray plod_ray(math::vec3f(world_origin), math::vec3f(world_direction), scm::math::length(ray.direction_));
      pbr::ren::Ray::Intersection intersection;

      auto plod_node = reinterpret_cast<node::PLODNode*>(owner);

      float aabb_scale = 9.0f;
      unsigned int max_depth = 255;
      unsigned int surfel_skip = 1;
      bool wysiwyg = true;

      pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();
      pbr::model_t model_id = controller->DeduceModelId(plod_node->get_geometry_description());

      if (plod_ray.IntersectModel(model_id, math::mat4f(model_transform), aabb_scale, max_depth, surfel_skip, wysiwyg, intersection)) {
        has_hit = true;
        pick.distance = intersection.distance_;
        pick.world_position = intersection.position_;
        auto object_position = scm::math::inverse(model_transform) * gua::math::vec4(intersection.position_.x, intersection.position_.y, intersection.position_.z, 1.0);
        pick.position = math::vec3(object_position.x, object_position.y, object_position.z);
        pick.normal = intersection.normal_;
      }

      if (has_hit && (hits.empty() || pick.distance < hits.begin()->distance)) {
        hits.insert(pick);
      }
    */
}

/////////////////////////////////////////////////////////////////////////////////

} // namespace gua
