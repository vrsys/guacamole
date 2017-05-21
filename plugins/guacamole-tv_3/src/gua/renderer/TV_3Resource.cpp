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
#include <gua/renderer/TV_3Resource.hpp>

#include <gua/utils/Singleton.hpp>
#include <gua/node/TV_3Node.hpp>

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

namespace gua {

////////////////////////////////////////////////////////////////////////////////

TV_3Resource::TV_3Resource(std::string const& resource_file_string, bool is_pickable)
  : resource_file_name_(resource_file_string),
    is_pickable_(is_pickable),
    local_transform_(),
    volume_texture_(nullptr) {
  bounding_box_.min = scm::math::vec3(0.0f, 0.0f, 0.0f);
  bounding_box_.max = scm::math::vec3(1.0f, 1.0f, 1.0f);

  //volume_proxy_.reset();
}

////////////////////////////////////////////////////////////////////////////////

TV_3Resource::~TV_3Resource() {
}

////////////////////////////////////////////////////////////////////////////////

void TV_3Resource::draw(
  RenderContext const& ctx,
  scm::gl::vertex_array_ptr const& vertex_array) const {
  //dummy
}

void TV_3Resource::upload_to(RenderContext const& ctx) const {
  
  std::string raw_vol_path = resource_file_name_;


  if(resource_file_name_.find(".v_rsc") != std::string::npos) {
    std::string line_buffer;
    std::ifstream volume_resource_file(resource_file_name_, std::ios::in);
    while(std::getline(volume_resource_file, line_buffer)) {
      raw_vol_path = line_buffer;
    }
  }

  scm::gl::volume_loader vl;

  scm::math::vec3ui vol_dims = vl.read_dimensions(raw_vol_path);


  //volume_texture_ = ctx.render_device->create_texture_3d(scm::math::vec3ui(256,256,225), scm::gl::data_format::FORMAT_R_8);
  /*
  ctx.render_context->update_sub_texture(volume_texture_, scm::gl::texture_region(scm::math::vec3ui(0,0,0), scm::math::vec3ui(256,256,225)),
                                         scm::gl::data_format::FORMAT_R_8, (void* const) &cpu_data[0]);
  
*/

  
  //volume_texture_ = vl.load_texture_3d(*ctx.render_device, raw_vol_path, false, false);
  //volume_texture_ = ctx.render_device->create_texture_3d(vol_dims, scm::gl::data_format::FORMAT_R_16);

  std::ifstream in_vol_file(raw_vol_path.c_str(), std::ios::in | std::ios::binary);
  std::vector<void*> mip_data;

  uint64_t num_voxels = vol_dims[0] * vol_dims[1] * vol_dims[2] * 2;
  std::vector<unsigned char> read_buffer(num_voxels);

  in_vol_file.read( (char*) &read_buffer[0], num_voxels);
  in_vol_file.close();
  mip_data.push_back( (void*) &read_buffer[0] );

  volume_texture_ = ctx.render_device->create_texture_3d(scm::gl::texture_3d_desc(vol_dims, scm::gl::data_format::FORMAT_R_8), scm::gl::data_format::FORMAT_R_8, mip_data);


  /*

  std::make_shared<scm::gl::texture_3d>(
                                                          new scm::gl::texture_3d(*ctx.render_device, 
                                                              scm::gl::texture_3d_desc(scm::math::vec3ui(256,256,225),
                                                                                  scm::gl::data_format::FORMAT_R_8)  );
*/
}

void TV_3Resource::bind_volume_texture(
  RenderContext const& ctx, scm::gl::sampler_state_ptr const& sampler_state) const {

  if( nullptr == volume_texture_ ) {
    upload_to(ctx);
  }

  ctx.render_context->bind_texture(volume_texture_, sampler_state, 0);

/*
  std::cout << "In drawing branch\n";
  scm::gl::context_vertex_input_guard vig(ctx.render_context);

  ctx.render_context->apply();
  std::cout << "Before drawing\n";
  volume_proxy_.draw(ctx.render_context);
*/
}


////////////////////////////////////////////////////////////////////////////////
math::mat4 const& TV_3Resource::local_transform() const
{
  //return scm::math::mat4d(1.0);
  return local_transform_;
}

////////////////////////////////////////////////////////////////////////////////

void TV_3Resource::ray_test(Ray const& ray,
                             int options,
                             node::Node* owner,
                             std::set<PickResult>& hits) {
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

}
