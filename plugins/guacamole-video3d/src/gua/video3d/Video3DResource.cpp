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
#include <gua/video3d/Video3DResource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/video3d/Video3DRenderer.hpp>
#include <gua/video3d/Video3DLoader.hpp>
#include <gua/video3d/video3d_geometry/DXTCompressor.h>
#include <gua/video3d/video3d_geometry/NetKinectArray.hpp>
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

////////////////////////////////////////////////////////////////////////////////

Video3DResource::Video3DResource(std::string const& video3d, unsigned flags)
    : ks_filename_(video3d),
      calib_files_(),
      server_endpoint_(),
      depth_size_(),
      depth_size_byte_(),
      color_size_(),
      per_render_context_(),
      width_depthimage_(),
      height_depthimage_(),
      width_colorimage_(),
      height_colorimage_(),
      overwrite_normal_(false),
      o_normal_(),
      is_pickable_(flags & Video3DLoader::MAKE_PICKABLE) {
  init();

  // pre resize for video shooting VRHyperspace
  per_render_context_.resize(10);
}

////////////////////////////////////////////////////////////////////////////////
void Video3DResource::init() {
  // approximately local space - can be overwritten from .ks file
  bounding_box_ = math::BoundingBox<math::vec3>(math::vec3(-1.5, -0.1, -1.0),
                                                math::vec3(1.5, 2.2, 1.5));

  std::fstream istr;
  istr.open(ks_filename_.c_str(), std::ios::in);

  boost::filesystem::path ks_filepath(ks_filename_);
  boost::filesystem::path ks_dir = ks_filepath.parent_path();

  if (istr.good()) {
    std::string token;

    while (istr >> token) {
      if (token == "serverport") {
        istr >> server_endpoint_;
      } else if (token == "kinect") {
        istr >> token;
        std::string cf_absolute_path = ks_dir.string() + "/" + token;

        auto calib_file_ptr =
            std::make_shared<KinectCalibrationFile>(cf_absolute_path.c_str());

        calib_file_ptr->parse();
        calib_file_ptr->updateMatrices();

        calib_files_.push_back(calib_file_ptr);
      } else if (token == "bbx") {
        float x_min, y_min, z_min, x_max, y_max, z_max;
        istr >> x_min >> y_min >> z_min >> x_max >> y_max >> z_max;
        bounding_box_ = math::BoundingBox<math::vec3>(
            math::vec3(x_min, y_min, z_min), math::vec3(x_max, y_max, z_max));
      } else if (token == "normal") {
        float x, y, z;
        istr >> x >> y >> z;
        overwrite_normal_ = true;
        o_normal_ = scm::math::vec3f(x, y, z);
      }
    }

    for (auto calib_file : calib_files_) {
      const unsigned pixelcountc =
          calib_file->getWidthC() * calib_file->getHeightC();

      depth_size_ =
          calib_file->getWidth() * calib_file->getHeight();  //== pixelcount
      color_size_ = pixelcountc * 3 * sizeof(unsigned char);
      depth_size_byte_ = depth_size_ * sizeof(float);

      if (calib_file->isCompressedRGB()) {
        mvt::DXTCompressor dxt;
        dxt.init(
            calib_file->getWidthC(), calib_file->getHeightC(), FORMAT_DXT1);
        color_size_ = dxt.getStorageSize();
      }
    }

    assert(calib_files_.size() > 0);

    width_depthimage_ = calib_files_[0]->getWidth();
    height_depthimage_ = calib_files_[0]->getHeight();

    width_colorimage_ = calib_files_[0]->getWidthC();
    height_colorimage_ = calib_files_[0]->getHeightC();

    istr.close();
  } else {
    throw std::runtime_error("Couldn't open calib file");
  }
}

////////////////////////////////////////////////////////////////////////////////
void Video3DResource::upload_video_textures(RenderContext& ctx) const {
  if (per_render_context_.size() > ctx.id) {
    if (per_render_context_[ctx.id].color_tex_) {
      return;
    } else {
      // continue initialization
    }
  } else {
    per_render_context_.resize(ctx.id + 1);
  }

  std::cout << "Upload video textures" << std::endl;

  auto& per_context = per_render_context_[ctx.id];

  // initialize Texture Arrays (kinect depths & colors)
  per_context.depth_tex_ =
      ctx.render_device->create_texture_2d(
          scm::math::vec2ui(width_depthimage_, height_depthimage_),
          scm::gl::FORMAT_R_32F,
          0,
          calib_files_.size(),
          1);

  per_context.color_tex_ =
      ctx.render_device->create_texture_2d(
          scm::math::vec2ui(width_colorimage_, height_colorimage_),
          calib_files_[0]->isCompressedRGB() ? scm::gl::FORMAT_BC1_RGBA
                                             : scm::gl::FORMAT_RGB_8,
          0,
          calib_files_.size(),
          1);

  per_context.rstate_solid_ =
      ctx.render_device->create_rasterizer_state(
          scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, true);

  per_context.nka_ = new video3d::NetKinectArray(
      calib_files_, server_endpoint_, color_size_, depth_size_byte_);

  // generate and download calibvolumes for this context
  for (unsigned i = 0; i < number_of_cameras(); ++i) {
    std::vector<void*> raw_cv_xyz;
    raw_cv_xyz.push_back(calib_files_[i]->cv_xyz);
    // should be: glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB32F, cv_width, cv_height,
    // cv_depth, 0, GL_RGB, GL_FLOAT, (unsigned char*) cv_xyz);
    per_context.cv_xyz_
        .push_back(ctx.render_device->create_texture_3d(
            scm::math::vec3ui(calib_files_[i]->cv_width,
                              calib_files_[i]->cv_height,
                              calib_files_[i]->cv_depth),
            scm::gl::FORMAT_RGB_32F,
            0,
            scm::gl::FORMAT_RGB_32F,
            raw_cv_xyz));

    std::vector<void*> raw_cv_uv;
    raw_cv_uv.push_back(calib_files_[i]->cv_uv);
    // should be: glTexImage3D(GL_TEXTURE_3D, 0, GL_RG32F, cv_width, cv_height,
    // cv_depth, 0, GL_RG, GL_FLOAT, (unsigned char*) cv_uv);
    per_context.cv_uv_
        .push_back(ctx.render_device->create_texture_3d(
            scm::math::vec3ui(calib_files_[i]->cv_width,
                              calib_files_[i]->cv_height,
                              calib_files_[i]->cv_depth),
            scm::gl::FORMAT_RG_32F,
            0,
            scm::gl::FORMAT_RG_32F,
            raw_cv_uv));
  }

  per_context.frame_counter_ = 0;
}

////////////////////////////////////////////////////////////////////////////////

void Video3DResource::update_buffers(RenderContext& ctx) const {
  upload_video_textures(ctx);

  if (per_render_context_[ctx.id].frame_counter_ != ctx.framecount) {
    per_render_context_[ctx.id].frame_counter_ = ctx.framecount;
  } else {
    return;
  }
  auto & per_context = per_render_context_[ctx.id];
  if (per_context.nka_->update()) {
    unsigned char* buff = per_context.nka_->getBuffer();
    for (int i = 0; i < number_of_cameras(); ++i) {

      ctx.render_context->update_sub_texture(
          per_context.color_tex_,
          scm::gl::texture_region(
              scm::math::vec3ui(0, 0, i),
              scm::math::vec3ui(per_context.color_tex_->dimensions(), 1)),
          0,  //mip-mapping level
          calib_files_[0]->isCompressedRGB() ? scm::gl::FORMAT_BC1_RGBA
                                             : scm::gl::FORMAT_RGB_8,
          static_cast<void*>(buff));
      buff += color_size_;
      ctx.render_context->update_sub_texture(
          per_context.depth_tex_,
          scm::gl::texture_region(
              scm::math::vec3ui(0, 0, i),
              scm::math::vec3ui(per_context.depth_tex_->dimensions(), 1)),
          0,  //mip-mapping level
          scm::gl::FORMAT_R_32F,
          static_cast<void*>(buff));
      buff += depth_size_byte_;
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
KinectCalibrationFile const& Video3DResource::calibration_file(
    unsigned i) const {
  assert(i < calib_files_.size());
  return *calib_files_[i];
}

}
