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

#ifndef GUA_RESSOURCE_RENDERER_HPP
#define GUA_RESSOURCE_RENDERER_HPP

// guacamole_headers
#include <gua/platform.hpp>

// external headers
#include <functional>
#include <memory>
#include <map>
#include <unordered_map>
#include <vector>
#include <typeindex>
#include <string>

namespace gua {

class Pipeline;

namespace node {
  class GeometryNode;
}

class Pipeline;

class RessourceRenderer;
typedef std::function<std::shared_ptr<RessourceRenderer>(void)> creation_function;
typedef std::map<std::type_index, creation_function>            creation_function_map;

class GUA_DLL RessourceRenderer {
  public:

    virtual void draw(std::unordered_map<std::string, std::vector<node::GeometryNode*>> const& sorted_objects,
                      Pipeline* pipe) const = 0;

    static void register_renderer(std::type_index const& id,
                                  creation_function const& creation_function);

    static std::shared_ptr<RessourceRenderer> get_renderer(std::type_index const& id);

  private:
    static creation_function_map creation_functions_;

};

}

#endif  // GUA_RESSOURCE_RENDERER_HPP
