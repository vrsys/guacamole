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
#include <gua/guacamole.hpp>
#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/BuiltInTextures.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/renderer/PBSMaterialFactory.hpp>

#ifdef GUACAMOLE_GLFW3
#include <GLFW/glfw3.h>
#endif

namespace gua
{
void init(int argc, char** argv)
{
    static scm::shared_ptr<scm::core> scm_core(new scm::core(argc, argv));

#ifdef GUACAMOLE_GLFW3
    if(!glfwInit())
    {
        Logger::LOG_ERROR << "Failed to initialize GLFW!" << std::endl;
    }
    else
    {
        Logger::LOG_DEBUG << "Succeed to initialize GLFW!" << std::endl;
    }
#endif

    scm::gl::sampler_state_desc sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE, scm::gl::WRAP_CLAMP_TO_EDGE);

    gua::TextureDatabase::instance()->add("gua_loading_texture", std::make_shared<Texture2D>(gua::make_loading_image(), 1, sampler_state));

    gua::TextureDatabase::instance()->add("gua_default_texture", std::make_shared<Texture2D>(gua::make_default_image(), 1, sampler_state));
    gua::TextureDatabase::instance()->add("gua_noise_texture", std::make_shared<Texture2D>(gua::make_noise_image(), 1, sampler_state));

    TriMeshLoader mesh_loader;

#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;

    auto light_sphere_obj = factory.read_plain_file("resources/geometry/gua_light_sphere.obj");
    auto light_cone_obj = factory.read_plain_file("resources/geometry/gua_light_cone.obj");

    auto combined_light_sphere_cone_obj = factory.read_plain_file("resources/geometry/gua_combined_light_cone_sphere.obj");

    GeometryDatabase::instance()->add("gua_light_sphere_proxy",
                                      std::shared_ptr<GeometryResource>(static_cast<GeometryResource*>(mesh_loader.load_from_buffer(light_sphere_obj.c_str(), light_sphere_obj.size(), false)[0])));

    GeometryDatabase::instance()->add("gua_light_cone_proxy",
                                      std::shared_ptr<GeometryResource>(static_cast<GeometryResource*>(mesh_loader.load_from_buffer(light_cone_obj.c_str(), light_cone_obj.size(), false)[0])));

    GeometryDatabase::instance()->add("gua_combined_light_sphere_cone_proxy",
                                      std::shared_ptr<GeometryResource>(static_cast<GeometryResource*>(mesh_loader.load_from_buffer_unoptimized(combined_light_sphere_cone_obj.c_str(), combined_light_sphere_cone_obj.size(), false)[0])));

#else
    GeometryDatabase::instance()->add("gua_light_sphere_proxy",
                                      std::shared_ptr<GeometryResource>(static_cast<GeometryResource*>(mesh_loader.load_from_buffer(
                                          Resources::lookup_string(Resources::geometry_gua_light_sphere_obj).c_str(), Resources::geometry_gua_light_sphere_obj.size(), false)[0])));

    GeometryDatabase::instance()->add("gua_light_cone_proxy",
                                      std::shared_ptr<GeometryResource>(static_cast<GeometryResource*>(mesh_loader.load_from_buffer(
                                          Resources::lookup_string(Resources::geometry_gua_light_cone_obj).c_str(), Resources::geometry_gua_light_cone_obj.size(), false)[0])));

    GeometryDatabase::instance()->add("gua_combined_light_sphere_cone_proxy",
                                      std::shared_ptr<GeometryResource>(static_cast<GeometryResource*>(mesh_loader.load_from_buffer_unoptimized(
                                          Resources::lookup_string(Resources::geometry_gua_combined_light_sphere_cone_obj).c_str(), Resources::geometry_gua_combined_light_sphere_cone_obj.size(), false)[0])));


#endif

    PBSMaterialFactory::create_material(PBSMaterialFactory::ALL);

    Logger::LOG_DEBUG << "Guacamole initialization finished." << std::endl;
}

} // namespace gua
