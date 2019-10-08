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

#ifndef GUA_PBS_MATERIAL_FACTORY_HPP
#define GUA_PBS_MATERIAL_FACTORY_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Material.hpp>

// external headers
#include <string>
#include <memory>

namespace gua
{
class GUA_DLL PBSMaterialFactory
{
  public:
    enum Capabilities
    {
        COLOR_VALUE = 1 << 0,
        COLOR_MAP = 1 << 1,
        COLOR_VALUE_AND_MAP = 1 << 2,
        ROUGHNESS_VALUE = 1 << 3,
        ROUGHNESS_MAP = 1 << 4,
        METALNESS_VALUE = 1 << 5,
        METALNESS_MAP = 1 << 6,
        EMISSIVITY_VALUE = 1 << 7,
        EMISSIVITY_MAP = 1 << 8,
        NORMAL_MAP = 1 << 9,
        ALL = 1 << 10
    };

    static std::shared_ptr<Material> create_material(Capabilities const& capabilities);
    static std::string const material_name_from_capabilites(Capabilities const& capabilities);
};

} // namespace gua

#endif // GUA_PBS_MATERIAL_FACTORY_HPP
