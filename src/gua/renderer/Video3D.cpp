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
#include <gua/renderer/Video3D.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/video3d_geometry/DXTCompressor.h>
#include <gua/utils/logger.hpp>

// external headers
#include <iostream>
#include <gua/utils/string_utils.hpp>

namespace {
struct Vertex {
  scm::math::vec3f pos;
  scm::math::vec2f tex;
  scm::math::vec3f normal;
  scm::math::vec3f tangent;
  scm::math::vec3f bitangent;
};
}

namespace gua {

////////////////////////////////////////////////////////////////////////////////

Video3D::Video3D(std::string const& video3d) :
  video3d_(video3d),
  proxy_vertices_(),
  proxy_indices_(),
  proxy_vertex_array_(),
  color_texArrays_(),
  color_buffers_(),
  depth_texArrays_(),
  depth_buffers_(),
  calib_file_(nullptr),
  depth_size_(),
  depth_size_byte_(),
  color_size_(),
  width_(640),//640
  height_(480),//480
  file_buffers_(),
  upload_mutex_()
{
  std::vector<std::string> filename_decomposition =
  gua::string_utils::split(video3d_, '.');
  calib_file_ = new KinectCalibrationFile(("." +
                                          filename_decomposition[filename_decomposition.size() - 2] +
                                          ".yml").c_str()
                                          );
  calib_file_->parse();
  calib_file_->updateMatrices();
}

////////////////////////////////////////////////////////////////////////////////

void Video3D::upload_to(RenderContext const& ctx) const {
  
  int num_vertices = height_ * width_;
  int num_indices  = height_ * width_;
  int num_triangles           = ((height_-1) * (width_-1)) * 2;
  int num_triangle_indices    = 3 * num_triangles;
  int num_line_indices        = (height_ -1)*((width_-1)*3 + 1) + (width_-1);

  float step = 1.0f / width_;


  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (proxy_vertices_.size() <= ctx.id) {
    proxy_vertices_.resize(ctx.id + 1);
    proxy_indices_.resize(ctx.id + 1);
    proxy_vertex_array_.resize(ctx.id + 1);
    color_texArrays_.resize(ctx.id + 1);
    color_buffers_.resize(ctx.id + 1);
    depth_texArrays_.resize(ctx.id + 1);
    depth_buffers_.resize(ctx.id + 1);
    file_buffers_.resize(ctx.id + 1);
  }

  proxy_vertices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       num_vertices * sizeof(Vertex),
                                       0);



  Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(
      proxy_vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));
  
  //compute vertex data (proxy mesh)ddd
  //const scm::math::vec3f& p(0.0f); //point of origin

  unsigned v(0);
  int pCount(0);
  for (float h = 0.5*step; h < height_*step; h += step)
  {
    for (float w = 0.5*step; w < width_*step; w += step)
    {
        data[v].pos = scm::math::vec3f(w, h, 0.0f);
        data[v].tex = scm::math::vec2f(w, h);
        data[v].normal = scm::math::vec3f(0.0f, 0.0f, 1.0f);
        data[v].tangent = scm::math::vec3(0.f, 0.f, 0.f);
        data[v].bitangent = scm::math::vec3(0.f, 0.f, 0.f);
        ++v;
        ++pCount;
    }
  }

  ctx.render_context->unmap_buffer(proxy_vertices_[ctx.id]);

  std::vector<unsigned> index_array(num_triangle_indices);

  //coumpute index array (proxy mesh)
  v = 0;
  for (unsigned h(0); h < (height_-1); ++h)
  {
    for (unsigned w(0); w < (width_-1); ++w)
    {
            index_array[v] = (w + h * width_);
            ++v;
            index_array[v] = (w + h * width_ + 1);
            ++v;
            index_array[v] = (w + h * width_ + width_);
            ++v;
            index_array[v] = (w + h * width_ + width_);
            ++v;
            index_array[v] = (w + h * width_ + 1);
            ++v;
            index_array[v] = (w + h * width_ + 1 + width_);
            ++v;
    }
  }

  std::cout << "Vertex Count: " << pCount << "\n";
  std::cout << "Triangles Count: " << num_triangles << "\n";
  std::cout << "Triangle Indices: " << num_triangle_indices << "\n";

  proxy_indices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       num_triangle_indices * sizeof(unsigned int),
                                       &index_array[0]);

  std::vector<scm::gl::buffer_ptr> buffer_arrays;
  buffer_arrays.push_back(proxy_vertices_[ctx.id]);

  proxy_vertex_array_[ctx.id] = ctx.render_device->create_vertex_array
                                      (scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))
                                                    (0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))
                                                    (0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))
                                                    (0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))
                                                    (0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex)),
                                       buffer_arrays);

  // initialize Texture Arrays (kinect depths & colors)
  depth_texArrays_[ctx.id] = ctx.render_device->create_texture_2d(scm::math::vec2ui(width_, height_),
                                                       scm::gl::FORMAT_R_32F,
                                                       0,
                                                       1, //kinect count
                                                       1
                                                    );

  color_texArrays_[ctx.id] = ctx.render_device->create_texture_2d( scm::math::vec2ui(width_, height_),
                                                       scm::gl::FORMAT_BC1_RGBA,
                                                       0,
                                                       1, //kinect count
                                                       1
                                                   );

  // init filebuffers
  std::vector<std::string> filename_decomposition =
  gua::string_utils::split(video3d_, '.');
  file_buffers_[ctx.id] = new sys::FileBuffer(("." +
                                               filename_decomposition[filename_decomposition.size() - 2] +
                                               ".stream").c_str()
                                              );

  if(!file_buffers_[ctx.id]->open("r")){
      std::cerr << "ERROR opening " << filename_decomposition[0] << ".stream exiting..." << std::endl;
        exit(1);
    }
  file_buffers_[ctx.id]->setLooping(true);

  const unsigned pixelcountc = calib_file_->getWidthC() * calib_file_->getHeightC();

  depth_size_ = calib_file_->getWidth() * calib_file_->getHeight(); //== pixelcount

  color_size_ = pixelcountc * 3 * sizeof(unsigned char);

    if(calib_file_->isCompressedRGB()){
        mvt::DXTCompressor dxt;
        dxt.init(calib_file_->getWidthC(), calib_file_->getHeightC(), FORMAT_DXT1);
        color_size_ = dxt.getStorageSize();
    }

    if(calib_file_->isCompressedDepth()){
        depth_size_ =  calib_file_->getWidth() * calib_file_->getHeight() * sizeof(unsigned char);
    }

    color_buffers_[ctx.id] = new unsigned char[color_size_];
    depth_buffers_[ctx.id] = new float[depth_size_];
    depth_size_byte_ = depth_size_ * sizeof(float);

    //
}

////////////////////////////////////////////////////////////////////////////////

void Video3D::draw(RenderContext const& ctx) const {
  // upload to GPU if neccessary
  if (proxy_vertices_.size() <= ctx.id || proxy_vertices_[ctx.id] == nullptr) {
    upload_to(ctx);
  }

  scm::gl::context_vertex_input_guard vig(ctx.render_context);

  ctx.render_context->bind_vertex_array(proxy_vertex_array_[ctx.id]);

  ctx.render_context->bind_index_buffer(
      proxy_indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);

  update_buffers(ctx);

  //update kinect color & depth texture array for given context
  ctx.render_context->update_sub_texture(depth_texArrays_[ctx.id],
                                //scm::gl::texture_region(scm::math::vec3ui(0, 0, i),
                                scm::gl::texture_region(scm::math::vec3ui(0, 0, 0),
                                                       scm::math::vec3ui(width_, height_, 1)),
                                0, //mip-mapping level
                                scm::gl::FORMAT_R_32F,
                                (void*) depth_buffers_[ctx.id]
                              );

  ctx.render_context->update_sub_texture(color_texArrays_[ctx.id],
                                //scm::gl::texture_region(scm::math::vec3ui(0, 0, i),
                                scm::gl::texture_region(scm::math::vec3ui(0, 0 , 0),
                                                       scm::math::vec3ui(width_, height_, 1)),
                                0, //mip-mapping level
                                scm::gl::FORMAT_BC1_RGBA,
                                (void*) color_buffers_[ctx.id]
                              );


  // last lines*
  ctx.render_context->apply();
  ctx.render_context->draw_elements(6 * (height_-1) * (width_-1));//mesh_->mNumFaces * 3
}

////////////////////////////////////////////////////////////////////////////////

void Video3D::update_buffers(RenderContext const& ctx) const
{
  if(file_buffers_[ctx.id]->read( (void*) color_buffers_[ctx.id], color_size_) != color_size_){
        std::cerr << "ERROR reading color BufferData\n";
  }

  if(file_buffers_[ctx.id]->read( (void*) depth_buffers_[ctx.id], depth_size_byte_) != depth_size_byte_){
        std::cerr << "ERROR reading depth BufferData\n";
  }
}

void Video3D::set_uniforms(RenderContext const& ctx, ShaderProgram* cs){
     // TO DO
    //cs->set_uniform(ctx, color_texArrays_[ctx.id], "color_video3d_texture");
}

}
