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

#include <fstream>
#include <regex>
namespace gua
{


std::string _strip_whitespace(std::string const& in_string) { return std::regex_replace(in_string, std::regex("^ +| +$|( ) +"), "$1"); }

bool _starts_with(std::string const& in_string, std::string const& comparison_string) {
    std::string const& white_space_stripped_string = _strip_whitespace(in_string);
    std::string const& white_space_stripped_comp_string = _strip_whitespace(comparison_string);

    if(white_space_stripped_string.size() < white_space_stripped_comp_string.size()) {
        return false;
    }

    auto const& start_of_string = white_space_stripped_string.substr(0, white_space_stripped_comp_string.size());
    if(start_of_string == white_space_stripped_comp_string) {
        return true;
    } else {
        return false;
    }

}

void _split_filename(std::string const& in_line_buffer, std::vector<std::string> const& registered_tokens, std::map<std::string, std::string>& tokens) {
    std::string whitespace_removed_line_buffer = _strip_whitespace(in_line_buffer);

    std::regex non_negative_number_regex("[[:digit:]]+");

    for(auto const& potentially_matching_token : registered_tokens)
    {
        if(whitespace_removed_line_buffer.find(potentially_matching_token) == 0)
        {
            uint64_t length_of_token = potentially_matching_token.size();
            std::string remaining_string = whitespace_removed_line_buffer.substr(length_of_token + 1);

            tokens[potentially_matching_token] = remaining_string.c_str();
            break;
        }
    }
}



SPointsResource::SPointsData::SPointsData(RenderContext const& ctx, SPointsResource const& spoints_resource)
{
    rstate_solid_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, true);

    nka_ = std::make_shared<spoints::NetKinectArray>(spoints_resource.server_endpoint(), spoints_resource.feedback_endpoint());
}

////////////////////////////////////////////////////////////////////////////////

SPointsResource::SPointsResource(std::string const& spoints_resource_filename, unsigned flags)
    : spoints_resource_filename_(spoints_resource_filename), is_pickable_(flags & SPointsLoader::MAKE_PICKABLE)
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
    spointsdata_->nka_->update(ctx, bounding_box_, inv_xyz_vol_res_, uv_vol_res_);
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


        {
            std::string line_buffer;
            std::ifstream in_spoints_resource_file(spoints_resource_filename_, std::ios::in);

            std::string const serversocket_identifier("serversocket");
            std::string const feedbacksocket_identifier("feedbacksocket");
            std::string const inv_xyz_volume_res_identifier("inv_xyz_volume_res");
            std::string const uv_volume_res_identifier("uv_volume_res");

            std::vector<std::string> registered_resource_file_tokens = {serversocket_identifier, feedbacksocket_identifier};

            while(std::getline(in_spoints_resource_file, line_buffer))
            {
                std::map<std::string, std::string> parsed_line_tokens;
                _split_filename(line_buffer, registered_resource_file_tokens, parsed_line_tokens);

                auto map_iterator = parsed_line_tokens.end();

                map_iterator = parsed_line_tokens.find(serversocket_identifier);
                if(map_iterator != parsed_line_tokens.end())
                {
                    server_endpoint_ = map_iterator->second;
                }

                map_iterator = parsed_line_tokens.find(feedbacksocket_identifier);
                if(map_iterator != parsed_line_tokens.end())
                {
                    feedback_endpoint_ = map_iterator->second;
                }

                bool matches_inv_xyz_vol_identifier = _starts_with(line_buffer, inv_xyz_volume_res_identifier);
                if(matches_inv_xyz_vol_identifier) {
                    std::istringstream line_string_stream(line_buffer);

                    std::string dummy;
                    line_string_stream >> dummy;

                    for(uint32_t dim_idx = 0; dim_idx < 3; ++dim_idx) {
                        line_string_stream >> inv_xyz_vol_res_[dim_idx];
                    }
                }

                bool matches_uv_vol_identifier =_starts_with(line_buffer, uv_volume_res_identifier);
                if(matches_uv_vol_identifier) {
                    std::istringstream line_string_stream(line_buffer);

                    std::string dummy;
                    line_string_stream >> dummy;

                    for(uint32_t dim_idx = 0; dim_idx < 3; ++dim_idx) {
                        line_string_stream >> uv_vol_res_[dim_idx];
                    }
                }

            }
        }

        std::cout << "Used receive socket: " << server_endpoint_ << std::endl;
        std::cout << "Used feedback socket: " << feedback_endpoint_ << std::endl;
        std::cout << "Parsed uv volume res: " << uv_vol_res_[0] << " " << uv_vol_res_[1] << " " << uv_vol_res_[2] << std::endl;
        std::cout << "Parsed inv_xyz volume res: " << inv_xyz_vol_res_[0] << " " << inv_xyz_vol_res_[1] << " " << inv_xyz_vol_res_[2] << std::endl;

    //server_endpoint_(server_endpoint), feedback_endpoint_(feedback_endpoint),
}

} // namespace gua
