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
#include <gua/renderer/PLODResource.hpp>

// external headers
#include <pbr/ren/lod_point_cloud.h>
#include <pbr/ren/model_database.h>
#include <pbr/ren/policy.h>

namespace gua {

/////////////////////////////////////////////////////////////////////////////
PLODLoader::PLODLoader() : _supported_file_extensions() {
  _supported_file_extensions.insert("kdn");
  _supported_file_extensions.insert("KDN");
}

/////////////////////////////////////////////////////////////////////////////
std::shared_ptr<node::PLODNode> PLODLoader::load_geometry(std::string const& nodename,
                                                          std::string const& filename,
                                                          std::shared_ptr<Material> const& fallback_material,
                                                          unsigned flags)
{
  auto cached_node(load_geometry(filename, flags));

  if (cached_node) {
    auto copy = std::dynamic_pointer_cast<node::PLODNode>(cached_node->deep_copy());

    if (copy) {
      apply_fallback_material(copy, fallback_material);
      copy->set_name(nodename);
      return copy;
    }
  }

  Logger::LOG_WARNING << "PLODLoader::load_geometry() : unable to create PLOD Node" << std::endl;
  return std::shared_ptr<node::PLODNode>(new node::PLODNode(nodename));
}

/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::PLODNode> PLODLoader::load_geometry(std::string const& filename, unsigned flags)
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

      std::shared_ptr<node::PLODNode> node(new node::PLODNode(filename, desc.unique_key(), filename));
      node->update_cache();
     

      auto bbox = resource->get_bounding_box();

      //normalize position?
      auto normalize_position = flags & PLODLoader::NORMALIZE_POSITION;
      if (normalize_position) {
        node->translate(-bbox.center());
      }

      //normalize scale?
      auto normalize_node = flags & PLODLoader::NORMALIZE_SCALE;
      if (normalize_node) {
        node->scale(1.0f / scm::math::length(bbox.max - bbox.min));
      }

      return node;
    }
  } catch (std::exception & e) {
    Logger::LOG_WARNING << "Failed to load PLOD object \"" << filename << "\": " << e.what() << std::endl;
    return nullptr;
  }
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

  if(g_node && g_node->get_material()->get_shader_name() == "") {
    g_node->set_material(fallback_material);
    g_node->update_cache();
  }

  for(auto& child : root->get_children()) {
    apply_fallback_material(child, fallback_material);
  }
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

size_t PLODLoader::get_upload_budget_in_mb() const {
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
  return policy->max_upload_budget_in_mb();
}

size_t PLODLoader::get_render_budget_in_mb() const {
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();

  return policy->render_budget_in_mb();
}

size_t PLODLoader::get_out_of_core_budget_in_mb() const {
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
  return policy->out_of_core_budget_in_mb();
}

////////////////////////////////////////////////////////////////////////////////

void PLODLoader::set_importance(std::string const& file_name, const float importance) {
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();
  policy->SetImportance(file_name, importance);
}

}
