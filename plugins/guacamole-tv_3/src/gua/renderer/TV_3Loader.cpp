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
#include <gua/renderer/TV_3ResourceVQCompressed.hpp>

namespace gua
{
/////////////////////////////////////////////////////////////////////////////
// static variables
/////////////////////////////////////////////////////////////////////////////
std::unordered_map<std::string, std::shared_ptr<::gua::node::Node>> TV_3Loader::loaded_files_ = std::unordered_map<std::string, std::shared_ptr<::gua::node::Node>>();

/////////////////////////////////////////////////////////////////////////////
TV_3Loader::TV_3Loader() : _supported_file_extensions()
{
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
std::shared_ptr<node::Node> TV_3Loader::load_geometry(std::string const& file_name, unsigned flags, int const cpu_budget, int const gpu_budget)
{
    std::shared_ptr<node::Node> cached_node = nullptr;
    std::string key(file_name + "_" + string_utils::to_string(flags));
    auto searched(loaded_files_.find(key));

    if(searched != loaded_files_.end())
    {
        cached_node = searched->second;
    }
    else
    {
        bool fileload_succeed = false;

        if(is_supported(file_name))
        {
            // cached_node = load(file_name, flags);
            GeometryDescription desc("TV_3", file_name, 0, flags);

            std::shared_ptr<TV_3Resource> resource = nullptr;
            if(file_name.find("SW_VQ") != std::string::npos)
            {
                resource = std::make_shared<TV_3ResourceVQCompressed>(file_name, flags & TV_3Loader::MAKE_PICKABLE);
            }
            else
            {
                resource = std::make_shared<TV_3Resource>(file_name, flags & TV_3Loader::MAKE_PICKABLE);
            }

            GeometryDatabase::instance()->add(desc.unique_key(), resource);

            cached_node = std::make_shared<node::TV_3Node>("", desc.unique_key(), file_name);
            cached_node->update_cache();

            loaded_files_.insert(std::make_pair(key, cached_node));

            // normalize mesh position and rotation
            if(flags & TV_3Loader::NORMALIZE_POSITION || flags & TV_3Loader::NORMALIZE_SCALE)
            {
                auto bbox = cached_node->get_bounding_box();

                if(flags & TV_3Loader::NORMALIZE_POSITION)
                {
                    auto center((bbox.min + bbox.max) * 0.5f);
                    cached_node->translate(-center);
                }

                if(flags & TV_3Loader::NORMALIZE_SCALE)
                {
                    auto size(bbox.max - bbox.min);
                    auto max_size(std::max(std::max(size.x, size.y), size.z));
                    cached_node->scale(1.f / max_size);
                }
            }

            auto tv_3_cached_node = std::dynamic_pointer_cast<gua::node::TV_3Node>(cached_node);
            if(flags & TV_3Loader::USE_SURFACE_MODE)
            {
                tv_3_cached_node->set_render_mode(node::TV_3Node::RenderMode::SUR_PBR);
            }
            else
            {
                tv_3_cached_node->set_render_mode(node::TV_3Node::RenderMode::VOL_COMPOSITING);
            }

            fileload_succeed = true;
        }

        if(!fileload_succeed)
        {
            Logger::LOG_WARNING << "Unable to load " << file_name << ": Type is not supported!" << std::endl;
        }
    }
    return cached_node;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> TV_3Loader::create_geometry_from_file(
    std::string const& node_name, std::string const& file_name, std::shared_ptr<Material> const& fallback_material, unsigned flags, int const cpu_budget, int const gpu_budget)
{
    auto cached_node(load_geometry(file_name, flags));

    if(cached_node)
    {
        auto copy(cached_node->deep_copy(false));

        apply_fallback_material(copy, fallback_material, flags); //& NO_SHARED_MATERIALS

        copy->set_name(node_name);
        return copy;
    }

    return cached_node;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> TV_3Loader::create_geometry_from_file(std::string const& node_name, std::string const& file_name, unsigned flags, int const cpu_budget, int const gpu_budget)
{
    auto cached_node(load_geometry(file_name, flags));

    if(cached_node)
    {
        auto copy(std::dynamic_pointer_cast<node::Node>(cached_node->deep_copy(false)));

        auto shader(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material"));
        apply_fallback_material(copy, shader->make_new_material(), flags /*& NO_SHARED_MATERIALS*/);

        copy->set_name(node_name);
        return copy;
    }

    return cached_node;
}

////////////////////////////////////////////////////////////////////////////////

void TV_3Loader::apply_fallback_material(std::shared_ptr<node::Node> const& root, std::shared_ptr<Material> const& fallback_material, bool no_shared_materials)
{
    auto g_node(std::dynamic_pointer_cast<node::TV_3Node>(root));

    if(g_node && !g_node->get_material())
    {
        g_node->set_material(fallback_material);
        g_node->update_cache();
    }
    else if(g_node && no_shared_materials)
    {
        g_node->set_material(std::make_shared<Material>(*g_node->get_material()));
    }

    for(auto& child : root->get_children())
    {
        apply_fallback_material(child, fallback_material, no_shared_materials);
    }
}
/////////////////////////////////////////////////////////////////////////////

/*
std::shared_ptr<node::TV_3Node> TV_3Loader::load_geometry(std::string const& filename, unsigned flags,
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
  return std::shared_ptr<node::TV_3Node>(new node::TV_3Node(filename));

}
*/

////////////////////////////////////////////////////////////////////////////////

bool TV_3Loader::is_supported(std::string const& file_name) const
{
    std::vector<std::string> filename_decomposition = gua::string_utils::split(file_name, '.');
    return filename_decomposition.empty() ? false : _supported_file_extensions.count(filename_decomposition.back()) > 0;
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
} // namespace gua
