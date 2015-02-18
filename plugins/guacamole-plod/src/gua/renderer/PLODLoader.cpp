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
#include <gua/renderer/PLODLoader.hpp>

// guacamole headers
#include <gua/utils.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/node/PLODNode.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/renderer/PLODResource.hpp>

// external headers
#include <pbr/ren/lod_point_cloud.h>
#include <pbr/ren/model_database.h>
#include <pbr/ren/policy.h>
#include <pbr/ren/ray.h>

namespace gua {

/////////////////////////////////////////////////////////////////////////////
PLODLoader::PLODLoader() : _supported_file_extensions() {
  _supported_file_extensions.insert("kdn");
  _supported_file_extensions.insert("KDN");
}

/////////////////////////////////////////////////////////////////////////////
std::shared_ptr<node::PLODNode> PLODLoader::load_geometry(std::string const& nodename,
                                                          std::string const& filename,
                                                          std::shared_ptr<Material> const& material,
                                                          unsigned flags)
{
  try {
    if(!is_supported(filename)){
      throw std::runtime_error(std::string("Unsupported filetype: ") + filename);
    }
    else {
      pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();

      GeometryDescription desc("PLOD", filename, 0, flags);

      pbr::model_t model_id = database->AddModel(filename, desc.unique_key());

      auto resource = std::make_shared<PLODResource>(model_id, flags & PLODLoader::MAKE_PICKABLE);
      GeometryDatabase::instance()->add(desc.unique_key(), resource);

      std::shared_ptr<node::PLODNode> node(new node::PLODNode(filename, desc.unique_key(), filename, material));
      
      node->update_cache();
     

      auto bbox = resource->get_bounding_box();

      //normalize position?
      auto normalize_position = flags & PLODLoader::NORMALIZE_POSITION;
      if (normalize_position) {
        node->translate(-bbox.center());
      }

      //normalize scale?
      auto normalize_scale = flags & PLODLoader::NORMALIZE_SCALE;
      if (normalize_scale) {
        node->scale(1.0f / scm::math::length(bbox.max - bbox.min));
      }

      return node;
    }
  } catch (std::exception & e) {
    Logger::LOG_WARNING << "Failed to load PLOD object \"" << filename << "\": " << e.what() << std::endl;
    return nullptr;
  }
}



/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::PLODNode> PLODLoader::load_geometry(std::string const& filename, unsigned flags)
{
  auto shader(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material"));
  auto cached_node(load_geometry(filename, filename, shader->make_new_material(), flags));

  if (cached_node) {
#if 0
    auto copy = std::dynamic_pointer_cast<node::PLODNode>(cached_node->deep_copy());
    if (copy) {
      return copy;
    }
#endif
    return cached_node;
  }

  Logger::LOG_WARNING << "PLODLoader::load_geometry() : unable to create PLOD Node" << std::endl;
  return std::shared_ptr<node::PLODNode>(new node::PLODNode(filename));

}

////////////////////////////////////////////////////////////////////////////////

bool PLODLoader::is_supported(std::string const& file_name) const {

  std::vector<std::string> filename_decomposition = gua::string_utils::split(file_name, '.');
  return filename_decomposition.empty()
             ? false
             : _supported_file_extensions.count(filename_decomposition.back()) >
                   0;
}

////////////////////////////////////////////////////////////////////////////////
void PLODLoader::apply_fallback_material(std::shared_ptr<node::Node> const& root, std::shared_ptr<Material> const& fallback_material) const {
  auto g_node(std::dynamic_pointer_cast<node::PLODNode>(root));

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
  }
}
////////////////////////////////////////////////////////////////////////////////

std::set<PickResult> PLODLoader::pick_plod_interpolate(math::vec3 const& bundle_origin,
                                           math::vec3 const& bundle_forward,
                                           math::vec3 const& bundle_up,
                                           float bundle_radius,
                                           float max_distance,
                                           unsigned int max_depth,
                                           unsigned int surfel_skip) const {

  std::set<PickResult> results;


  scm::math::vec3f ray_pos = scm::math::vec3f(bundle_origin.x, bundle_origin.y, bundle_origin.z);
  scm::math::vec3f ray_fwd = scm::math::vec3f(bundle_forward.x, bundle_forward.y, bundle_forward.z);
  scm::math::vec3f ray_up = scm::math::vec3f(bundle_up.x, bundle_up.y, bundle_up.z);

  pbr::ren::Ray ray(ray_pos, ray_fwd, max_distance);
  pbr::ren::Ray::Intersection intersection;

  if (ray.Intersect(max_distance, ray_up, bundle_radius, max_depth, surfel_skip, intersection)) {
    PickResult result(intersection.distance_, 
                      nullptr, 
                      math::vec3(), 
                      intersection.position_, 
                      math::vec3(), 
                      intersection.normal_, 
                      math::vec2());
    results.insert(result);
  }

  return results;
  
}

////////////////////////////////////////////////////////////////////////////////

void PLODLoader::set_upload_budget_in_mb(const size_t upload_budget) {

  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
  policy->set_max_upload_budget_in_mb(upload_budget);
}

////////////////////////////////////////////////////////////////////////////////

void PLODLoader::set_render_budget_in_mb(const size_t render_budget) {

  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
  policy->set_render_budget_in_mb(render_budget);
}


////////////////////////////////////////////////////////////////////////////////

void PLODLoader::set_out_of_core_budget_in_mb(const size_t out_of_core_budget) {
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
  policy->set_out_of_core_budget_in_mb(out_of_core_budget);
}

////////////////////////////////////////////////////////////////////////////////

size_t PLODLoader::get_upload_budget_in_mb() const {
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
  return policy->max_upload_budget_in_mb();
}

////////////////////////////////////////////////////////////////////////////////

size_t PLODLoader::get_render_budget_in_mb() const {
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();

  return policy->render_budget_in_mb();
}

size_t PLODLoader::get_out_of_core_budget_in_mb() const {
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
  return policy->out_of_core_budget_in_mb();
}

float PLODLoader::get_error_treshold() const {
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
  return policy->error_threshold();
}

////////////////////////////////////////////////////////////////////////////////

void PLODLoader::set_error_threshold(const float error_threshold) {
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
  policy->set_error_threshold(error_threshold);
}


}
