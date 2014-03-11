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

// header
#include <gua/renderer/GeometryLoader.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/scenegraph/TransformNode.hpp>
#include <gua/renderer/MeshLoader.hpp>
#include <gua/renderer/NURBSLoader.hpp>
#include <gua/renderer/Video3DLoader.hpp>
#include <gua/renderer/VolumeLoader.hpp>
#include <gua/scenegraph/GeometryNode.hpp>
#include <gua/scenegraph/Video3DNode.hpp>
#include <gua/scenegraph/VolumeNode.hpp>
#include <gua/utils/logger.hpp>

// external headers
#include <iostream>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

std::unordered_map<std::string, std::shared_ptr<Node>>
    GeometryLoader::loaded_files_ =
        std::unordered_map<std::string, std::shared_ptr<Node>>();

////////////////////////////////////////////////////////////////////////////////
GeometryLoader::GeometryLoader() : fileloaders_() {  
  fileloaders_.push_back(new MeshLoader);
  fileloaders_.push_back(new NURBSLoader);
  fileloaders_.push_back(new Video3DLoader);
  fileloaders_.push_back(new VolumeLoader);
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */
GeometryLoader::~GeometryLoader() {
  for (auto f : fileloaders_) {
    if (f) {
      delete f;
    }
  }
  fileloaders_.clear();
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> GeometryLoader::load_geometry(std::string const& file_name, unsigned flags) {
  std::shared_ptr<Node> cached_node;
  std::string key(file_name + "_" + string_utils::to_string(flags));

  auto searched(loaded_files_.find(key));

  if (searched != loaded_files_.end()) {

      cached_node = searched->second;

  } else {

    bool fileload_succeed = false;
    for (auto f : fileloaders_) {
      if (f->is_supported(file_name)) {
        cached_node = f->load(file_name, flags);
        cached_node->update_cache();

        loaded_files_.insert(std::make_pair(key, cached_node));

        // normalize mesh position and rotation
        if (flags & GeometryLoader::NORMALIZE_POSITION || flags & GeometryLoader::NORMALIZE_SCALE) {
          auto bbox = cached_node->get_bounding_box();

          if (flags & GeometryLoader::NORMALIZE_POSITION) {
            auto center((bbox.min + bbox.max)*0.5);
            cached_node->translate(-center);
          }

          if (flags & GeometryLoader::NORMALIZE_SCALE) {
            auto size(bbox.max - bbox.min);
            auto max_size(std::max(std::max(size.x, size.y), size.z));
            cached_node->scale(1.f/max_size);
          }

        }

        fileload_succeed = true;
        break;
      }
    }

    if (!fileload_succeed) {

      WARNING("Unable to load %s: Type is not supported!", file_name.c_str());
    }
  }

  return cached_node;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> GeometryLoader::create_geometry_from_file
                                            (std::string const& node_name,
                                             std::string const& file_name,
                                             std::string const& fallback_material,
                                             unsigned flags) {

  auto cached_node(load_geometry(file_name, flags));

  if (cached_node) {
    auto copy(cached_node->deep_copy());

    apply_fallback_material(copy, fallback_material);

    copy->set_name(node_name);
    return copy;
  }

  return std::make_shared<TransformNode>(node_name);
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> GeometryLoader::create_volume_from_file(std::string const& node_name,
                                                            	std::string const& file_name,
                                                            	unsigned flags)
{
    std::shared_ptr<Node> cached_node;
    std::string key(file_name + "_" + string_utils::to_string(flags));

    auto searched(loaded_files_.find(key));
    if (searched != loaded_files_.end()) {
        cached_node = searched->second;
    } else {

        bool fileload_succeed = false;
        for (auto f : fileloaders_) {
            if (f->is_supported(file_name)) {
                cached_node = f->load(file_name, flags);
                cached_node->update_cache();
                loaded_files_.insert(std::make_pair(key, cached_node));

                // normalize volume position and rotation
                if ( flags & VolumeLoader::NORMALIZE_POSITION
                  || flags & VolumeLoader::NORMALIZE_SCALE) {
                      auto bbox = cached_node->get_bounding_box();

                      if (flags & VolumeLoader::NORMALIZE_POSITION) {
                            auto center((bbox.min + bbox.max)*0.5);
                            cached_node->translate(-center);
                      }

                      if (flags & VolumeLoader::NORMALIZE_SCALE) {
                            auto size(bbox.max - bbox.min);
                            auto max_size(std::max(std::max(size.x, size.y),
                                                  size.z));
                            cached_node->scale(1.f / max_size);
                      }

                }
            }
            fileload_succeed = true;
            break;
        }

        if (!cached_node) {
            WARNING("Unable to load %s: Volume Type is not supported!",
                    file_name.c_str());
        }
    }

    if (cached_node) {
        auto copy(cached_node->deep_copy());

        copy->set_name(node_name);
        return copy;
    }

    return std::make_shared<TransformNode>(node_name);
}

////////////////////////////////////////////////////////////////////////////////

void GeometryLoader::apply_fallback_material(std::shared_ptr<Node> const& root,
                                   std::string const& fallback_material) const {

  auto g_node(std::dynamic_pointer_cast<GeometryNode>(root));
  
  if (g_node) {
    if (g_node->get_material().empty()) {
      g_node->set_material(fallback_material);
    }
  }
  else{
      auto v_node = std::dynamic_pointer_cast<Video3DNode>(root);
      if (v_node) {
          if (v_node->data.get_material().empty()) {
              v_node->data.set_material(fallback_material);
          }
      }
  }
  
  for(auto& child: root->get_children()) {
    apply_fallback_material(child, fallback_material);
  }

}

////////////////////////////////////////////////////////////////////////////////

}  // namespace gua
