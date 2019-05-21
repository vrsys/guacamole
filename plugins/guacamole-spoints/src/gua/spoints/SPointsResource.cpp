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
#include <gua/spoints/SPointsResource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/spoints/SPointsRenderer.hpp>
#include <gua/spoints/SPointsLoader.hpp>

#include <gua/utils/Logger.hpp>

#include <scm/gl_core/render_device/opengl/gl_core.h>

#include <boost/filesystem.hpp>
#include <boost/asio/ip/host_name.hpp>
#include <boost/algorithm/string.hpp>

// external headers
#include <iostream>
#include <fstream>
#include <gua/utils/string_utils.hpp>

namespace gua
{
SPointsResource::SPointsData::SPointsData(RenderContext const& ctx, SPointsResource const& spoints_resource)
{
    rstate_solid_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, true);

    nka_ = std::make_shared<spoints::NetKinectArray>(spoints_resource.server_endpoint(), spoints_resource.feedback_endpoint());
}

////////////////////////////////////////////////////////////////////////////////

SPointsResource::SPointsResource(std::string const& server_endpoint, std::string const& feedback_endpoint, unsigned flags)
    : server_endpoint_(server_endpoint), feedback_endpoint_(feedback_endpoint), is_pickable_(flags & SPointsLoader::MAKE_PICKABLE)
{
    init();
}

std::string SPointsResource::get_socket_string() const
{
    std::lock_guard<std::mutex> lock(m_push_matrix_package_mutex_);

    if(spointsdata_)
    {
        if(spointsdata_->nka_)
        {
            return spointsdata_->nka_->get_socket_string();
        }
    }

    return "";
}

void SPointsResource::push_matrix_package(spoints::camera_matrix_package const& cam_mat_package)
{
    // std::cout << "SpointsResource PushMatrixPackage: " << cam_mat_package.k_package.is_camera << "\n";

    std::lock_guard<std::mutex> lock(m_push_matrix_package_mutex_);

    if(spointsdata_)
    {
        if(spointsdata_->nka_)
        {
            spointsdata_->nka_->push_matrix_package(cam_mat_package);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
void SPointsResource::update_buffers(RenderContext const& ctx, Pipeline& pipe)
{
    {
        std::lock_guard<std::mutex> lock(m_push_matrix_package_mutex_);
        // lazy resource initialization
        if(nullptr == spointsdata_)
        {
            spointsdata_ = std::make_shared<SPointsData>(ctx, *this);
        }
    }

    // synchronize vertex data
    spointsdata_->nka_->update(ctx, bounding_box_);
}


bool SPointsResource::has_calibration(RenderContext const& ctx) const
{
    if(spointsdata_)
    {
        if(spointsdata_->nka_)
        {
            return spointsdata_->nka_->has_calibration(ctx);
        }
    }

    return false;
}

bool SPointsResource::is_vertex_data_fully_encoded()
{
    if(spointsdata_)
    {
        if(spointsdata_->nka_)
        {
            return spointsdata_->nka_->is_vertex_data_fully_encoded();
        }
    }

    return false;
}

void SPointsResource::draw_textured_triangle_soup(RenderContext const& ctx, std::shared_ptr<gua::ShaderProgram>& shader_program)
{
    if(spointsdata_)
    {
        spointsdata_->nka_->draw_textured_triangle_soup(ctx, shader_program);
    }
}

////////////////////////////////////////////////////////////////////////////////
void SPointsResource::init()
{
    // approximately local space - can be overwritten from .ks file
    bounding_box_ = math::BoundingBox<math::vec3>(math::vec3(-2.5, -2.5, -2.5), math::vec3(2.5, 2.5, 2.5));
}

} // namespace gua
