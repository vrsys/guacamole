#/******************************************************************************
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
#if 0
// class header
#include <gua/node/NURBSNode.hpp>

#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/renderer/NURBSLoader.hpp>

// guacamole headers

namespace gua {
namespace node {

  ////////////////////////////////////////////////////////////////////////////////
  NURBSNode::NURBSNode(std::string const& name,
                       std::string const& filename,
                       std::string const& material,
                       math::mat4 const& transform)
    : GeometryNode(name, filename, material, transform)
  {}


  ////////////////////////////////////////////////////////////////////////////////

  void NURBSNode::ray_test_impl(Ray const& ray,
                                PickResult::Options options,
                                Mask const& mask,
                                std::set<PickResult>& hits) {
    Logger::LOG_WARNING << "NURBSNode::ray_test_impl() : Ray test not implemented yet for NURBS" << std::endl;
  }

  ////////////////////////////////////////////////////////////////////////////////

  void NURBSNode::update_cache() {

    // The code below auto-loads a geometry if it's not already supported by
    // the GeometryDatabase. It expects a geometry name like
    //
    // "type='file'&file='data/objects/monkey.obj'&id=0&flags=0"

    if (filename_changed_)
    {
      if (filename_ != "")
      {
        if (!GeometryDatabase::instance()->is_supported(filename_))
        {
          auto params(string_utils::split(filename_, '&'));

          if (params.size() == 4)
          {
            if (params[0] == "type=file")
            {
              auto tmp_filename(string_utils::split(params[1], '='));
              auto tmp_flags(string_utils::split(params[2], '='));

              if (tmp_filename.size() == 2 && tmp_flags.size() == 2)
              {
                std::string filename(tmp_filename[1]);
                std::string flags_string(tmp_flags[1]);
                unsigned flags(0);
                std::stringstream sstr(flags_string);
                sstr >> flags;

                NURBSLoader loader;

                // node is deleted directly after initialization, but ressources remain in database
                loader.create_geometry_from_file("dummy", filename, material_, flags);
              }
              else {
                Logger::LOG_WARNING << "Failed to auto-load geometry " << filename_ << ": Failed to extract filename and/or loading flags!" << std::endl;
              }
            }
            else {
              Logger::LOG_WARNING << "Failed to auto-load geometry " << filename_ << ": Type is not supported!" << std::endl;
            }
          }
          else {
            Logger::LOG_WARNING << "Failed to auto-load geometry " << filename_ << ": The name does not contain a type, file, id and flag parameter!" << std::endl;
          }
        }
      }

      filename_changed_ = false;
    }

    // The code below auto-loads a material if it's not already supported by
    // the MaterialDatabase. It expects a material name like
    //
    // data/materials/Stones.gmd

    if (material_changed_)
    {
      if (material_ != "")
      {
        if (!MaterialDatabase::instance()->is_supported(material_))
        {
          MaterialDatabase::instance()->load_material(material_);
        }
      }

      material_changed_ = false;
    }

    GeometryNode::update_cache();
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<Node> NURBSNode::copy() const {
    auto result(std::make_shared<NURBSNode>(get_name(), filename_, material_, get_transform()));
    result->shadow_mode_ = shadow_mode_;
    return result;
  }

}
}
#endif
