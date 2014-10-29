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
#include <gua/renderer/RessourceRenderer.hpp>


namespace gua {

////////////////////////////////////////////////////////////////////////////////

creation_function_map RessourceRenderer::creation_functions_ = creation_function_map();

////////////////////////////////////////////////////////////////////////////////

void RessourceRenderer::register_renderer(std::type_index const& id,
                                          creation_function const& creation_function) {

  creation_functions_[id] = creation_function;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<RessourceRenderer> RessourceRenderer::get_renderer(std::type_index const& id) {
  auto function(creation_functions_.find(id));

  if (function != creation_functions_.end()) {
    return function->second();
  }

  return nullptr;
}

////////////////////////////////////////////////////////////////////////////////

}

