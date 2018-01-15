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

  frame_counter_ = 0;
}

////////////////////////////////////////////////////////////////////////////////

SPointsResource::SPointsResource(std::string const& server_endpoint, 
                                 std::string const& feedback_endpoint, 
                                 unsigned flags)
    : server_endpoint_(server_endpoint),
      feedback_endpoint_(feedback_endpoint),
      is_pickable_(flags & SPointsLoader::MAKE_PICKABLE) {
  init();
}

/*
void 
SPointsResource::push_matrix_package(bool is_camera, std::size_t view_uuid, bool stereo_mode, spoints::matrix_package mp) {
  if (nullptr != spointsdata_) {

    std::cout << "SpointsResource PushMatrixPackage: " << is_camera << "\n";
    spointsdata_->nka_->push_matrix_package(is_camera, view_uuid, stereo_mode, mp);
  }
}
*/

void
SPointsResource::push_matrix_package(spoints::camera_matrix_package const& cam_mat_package) {
  std::cout << "SpointsResource PushMatrixPackage: " << cam_mat_package.k_package.is_camera << "\n";

  if(spointsdata_) {
    if(spointsdata_->nka_) {
      spointsdata_->nka_->push_matrix_package(cam_mat_package);
    }
  }


  std::cout << "After second call\n";
}

////////////////////////////////////////////////////////////////////////////////
void SPointsResource::update_buffers(RenderContext const& ctx,
                                     Pipeline& pipe) {


  // lazy resource initialization
  if (nullptr == spointsdata_) {
    spointsdata_ = std::make_shared<SPointsData>(ctx, *this);
  }
  


  // synchronize feedback
  spointsdata_->nka_->update_feedback(ctx);


  if (spointsdata_->frame_counter_ != ctx.framecount) {
    spointsdata_->frame_counter_ = ctx.framecount;
  } else {
  
    return;
  }



  // synchronize vertex data
  spointsdata_->nka_->update(ctx);
  
}


void SPointsResource::draw(RenderContext const& ctx) {
  spointsdata_->nka_->draw(ctx);
}




////////////////////////////////////////////////////////////////////////////////
void SPointsResource::init() {
  // approximately local space - can be overwritten from .ks file
  bounding_box_ = math::BoundingBox<math::vec3>(math::vec3(-1.5, -0.1, -1.0),
                                                math::vec3(1.5, 2.2, 1.5));

}

}
