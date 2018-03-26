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

namespace gua {

SPointsResource::SPointsData::SPointsData(
  RenderContext const& ctx,
  SPointsResource const& spoints_resource) {

  rstate_solid_ = ctx.render_device->create_rasterizer_state(
                  scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, true);

  nka_ = std::make_shared<spoints::NetKinectArray>(spoints_resource.server_endpoint(), spoints_resource.feedback_endpoint());

}

////////////////////////////////////////////////////////////////////////////////

SPointsResource::SPointsResource(std::string const& server_endpoint, 
                                 std::string const& feedback_endpoint, 
                                 unsigned flags)
    : server_endpoint_(server_endpoint),
      feedback_endpoint_(feedback_endpoint),
      is_pickable_(flags & SPointsLoader::MAKE_PICKABLE),
      global_grid_dimension_x(-1),
      global_grid_dimension_y(-1),
      global_grid_dimension_z(-1),
      global_point_precision_x(-1),
      global_point_precision_y(-1),
      global_point_precision_z(-1),
      global_color_precision_x(-1),
      global_color_precision_y(-1),
      global_color_precision_z(-1) {
  init();
}

void
SPointsResource::push_matrix_package(spoints::camera_matrix_package& cam_mat_package) {
  //std::cout << "SpointsResource PushMatrixPackage: " << cam_mat_package.k_package.is_camera << "\n";

  std::lock_guard<std::mutex> lock(m_push_matrix_package_mutex);

  if(spointsdata_) {
    if(spointsdata_->nka_) {
      cam_mat_package.mat_package.global_grid_dimension_x = global_grid_dimension_x;
      cam_mat_package.mat_package.global_grid_dimension_y = global_grid_dimension_y;
      cam_mat_package.mat_package.global_grid_dimension_z = global_grid_dimension_z;
      cam_mat_package.mat_package.global_point_precision_x = global_point_precision_x;
      cam_mat_package.mat_package.global_point_precision_y = global_point_precision_y;
      cam_mat_package.mat_package.global_point_precision_z = global_point_precision_z;
      cam_mat_package.mat_package.global_color_precision_x = global_color_precision_x;
      cam_mat_package.mat_package.global_color_precision_y = global_color_precision_y;
      cam_mat_package.mat_package.global_color_precision_z = global_color_precision_z;
      spointsdata_->nka_->push_matrix_package(cam_mat_package);
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
void SPointsResource::update_buffers(RenderContext const& ctx,
                                     Pipeline& pipe) {

  {
    std::lock_guard<std::mutex> lock(m_push_matrix_package_mutex);
    // lazy resource initialization
    if (nullptr == spointsdata_) {
      spointsdata_ = std::make_shared<SPointsData>(ctx, *this);
    }
  

    //std::cout << "PRECONDITION CONTEXT: " << ctx.id << "\n";
    // synchronize feedback
    spointsdata_->nka_->update_feedback(ctx);
  }

  // synchronize vertex data
  spointsdata_->nka_->update(ctx);
  
}


void SPointsResource::draw(RenderContext const& ctx) {
  spointsdata_->nka_->draw(ctx);
}

Vec<float> const SPointsResource::getQuantizationStepSize(int cell_idx) const {
  if(!spointsdata_ || !spointsdata_->nka_)
    return Vec<float>(-1,-1,-1);
  return spointsdata_->nka_->getQuantizationStepSize(cell_idx);
}

////////////////////////////////////////////////////////////////////////////////
void SPointsResource::init() {
  // approximately local space - can be overwritten from .ks file
  bounding_box_ = math::BoundingBox<math::vec3>(math::vec3(-1.5, -0.1, -1.0),
                                                math::vec3(1.5, 2.2, 1.5));

}

}
