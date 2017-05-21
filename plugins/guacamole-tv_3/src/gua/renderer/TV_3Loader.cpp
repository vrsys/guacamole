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
#include <gua/renderer/TV_3Loader.hpp>

// guacamole headers
#include <gua/utils.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/node/TV_3Node.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/renderer/TV_3Resource.hpp>


namespace gua {

/////////////////////////////////////////////////////////////////////////////
TV_3Loader::TV_3Loader() : _supported_file_extensions() {
  _supported_file_extensions.insert("v_rsc");
  _supported_file_extensions.insert("raw");
}


////////////////////////////////////////////////////////////////////////////////

/*
std::shared_ptr<node::Node> TriMeshLoader::load_geometry(
    std::string const& file_name,
    unsigned flags) {
  std::shared_ptr<node::Node> cached_node;
  std::string key(file_name + "_" + string_utils::to_string(flags));
  auto searched(loaded_files_.find(key));

  if (searched != loaded_files_.end()) {

    cached_node = searched->second;

  } else {

    bool fileload_succeed = false;

    if (is_supported(file_name)) {
      cached_node = load(file_name, flags);
      cached_node->update_cache();

      loaded_files_.insert(std::make_pair(key, cached_node));

      // normalize mesh position and rotation
      if (flags & TriMeshLoader::NORMALIZE_POSITION ||
          flags & TriMeshLoader::NORMALIZE_SCALE) {
        auto bbox = cached_node->get_bounding_box();

        if (flags & TriMeshLoader::NORMALIZE_POSITION) {
          auto center((bbox.min + bbox.max) * 0.5f);
          cached_node->translate(-center);
        }

        if (flags & TriMeshLoader::NORMALIZE_SCALE) {
          auto size(bbox.max - bbox.min);
          auto max_size(std::max(std::max(size.x, size.y), size.z));
          cached_node->scale(1.f / max_size);
        }

      }

      fileload_succeed = true;
    }

    if (!fileload_succeed) {

      Logger::LOG_WARNING << "Unable to load " << file_name
                          << ": Type is not supported!" << std::endl;
    }
  }

  return cached_node;
}
*/


/////////////////////////////////////////////////////////////////////////////
std::shared_ptr<node::Node> TV_3Loader::load_geometry(std::string const& nodename,
                                                      std::string const& filename,
                                                      unsigned flags,
                                                      int64_t const cpu_budget,
                                                      int64_t const gpu_budget)
{

  
  try {
    if(!is_supported(filename)){
      throw std::runtime_error(std::string("Unsupported filetype: ") + filename);
    }
    else {
      
      GeometryDescription desc("TV_3", filename, 0, flags);

      auto resource = std::make_shared<TV_3Resource>(filename, flags & TV_3Loader::MAKE_PICKABLE);
      GeometryDatabase::instance()->add(desc.unique_key(), resource);

      auto node = std::make_shared<node::TV_3Node>(nodename, desc.unique_key(), filename);
      
      node->update_cache();
     
      auto bbox = resource->get_bounding_box();

      //normalize position?
      auto normalize_position = flags & TV_3Loader::NORMALIZE_POSITION;
      if (normalize_position) {
        auto bbox_center_object_space = math::vec4(bbox.center().x, bbox.center().y, bbox.center().z, 1.0);
        node->translate(-bbox_center_object_space.x, -bbox_center_object_space.y, -bbox_center_object_space.z);
      }

      //normalize scale?
      auto normalize_scale = flags & TV_3Loader::NORMALIZE_SCALE;
      if (normalize_scale) {
        auto scale = 1.0f / scm::math::length(bbox.max - bbox.min);
        node->scale(scale, scale, scale);
      }

      return node;
    }
  } catch (std::exception & e) {
    Logger::LOG_WARNING << "Failed to load (time-varying) volume object \"" << filename << "\": " << e.what() << std::endl;
    return nullptr;
  }
}





/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> TV_3Loader::load_geometry(std::string const& filename, unsigned flags,
                                                      int64_t const cpu_budget,
                                                      int64_t const gpu_budget) {

  auto desc = std::make_shared<gua::MaterialShaderDescription>();
  auto material_shader(std::make_shared<gua::MaterialShader>("Lod_unshaded_material", desc));
  gua::MaterialShaderDatabase::instance()->add(material_shader);

  auto cached_node(load_geometry(filename, filename, flags));

  if (cached_node) {
    return cached_node;
  }

  Logger::LOG_WARNING << "TV_3Loader::load_tv_3() : unable to create Lod Node" << std::endl;
  return std::shared_ptr<node::Node>(new node::TV_3Node(filename));

}


////////////////////////////////////////////////////////////////////////////////

bool TV_3Loader::is_supported(std::string const& file_name) const {

  std::vector<std::string> filename_decomposition = gua::string_utils::split(file_name, '.');
  return filename_decomposition.empty()
             ? false
             : _supported_file_extensions.count(filename_decomposition.back()) >
                   0;
}


/////////////////////////////////////////////////////////////////////////////////

/*
std::pair<std::string, math::vec3> TV_3Loader::pick_lod_bvh(math::vec3 const& ray_origin,
                                      math::vec3 const& ray_forward,
                                      float max_distance,
                                      std::set<std::string> const& model_filenames,
                                      float aabb_scale) const {

  scm::math::vec3f ray_pos = scm::math::vec3f(ray_origin.x, ray_origin.y, ray_origin.z);
  scm::math::vec3f ray_fwd = scm::math::vec3f(ray_forward.x, ray_forward.y, ray_forward.z);

  std::pair<std::string, math::vec3> result = std::make_pair("", math::vec3::zero());

  lamure::ren::ray ray(ray_pos, ray_fwd, max_distance);
  lamure::ren::ray::intersection_bvh intersection;

  if (ray.intersect_bvh(model_filenames, aabb_scale, intersection)) {
     result = std::make_pair(intersection.bvh_filename_, intersection.position_);

  }

  return result;

}

///////////////////////////////////////////////////////////////////////////////

std::set<PickResult> TV_3Loader::pick_lod_interpolate(math::vec3 const& bundle_origin,
                                           math::vec3 const& bundle_forward,
                                           math::vec3 const& bundle_up,
                                           float bundle_radius,
                                           float max_distance,
                                           unsigned int max_depth,
                                           unsigned int surfel_skip,
                                           float aabb_scale) const {

  std::set<PickResult> results;

  scm::math::vec3f ray_pos = scm::math::vec3f(bundle_origin.x, bundle_origin.y, bundle_origin.z);
  scm::math::vec3f ray_fwd = scm::math::vec3f(bundle_forward.x, bundle_forward.y, bundle_forward.z);
  scm::math::vec3f ray_up = scm::math::vec3f(bundle_up.x, bundle_up.y, bundle_up.z);

  lamure::ren::ray ray(ray_pos, ray_fwd, max_distance);
  lamure::ren::ray::intersection intersection;

  if (ray.intersect(aabb_scale, ray_up, bundle_radius, max_depth, surfel_skip, intersection)) {
    PickResult result(intersection.distance_,  
                      nullptr, 
                      math::vec3(intersection.error_),
                      math::vec3(intersection.position_), 
                      math::vec3(), 
                      math::vec3(intersection.normal_),
                      math::vec2());
    results.insert(result);
  }

  return results;
  
}
*/
}

