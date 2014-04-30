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
#include <gua/renderer/Video3DRessource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/Video3DUberShader.hpp>
#include <gua/renderer/video3d_geometry/DXTCompressor.h>
#include <gua/utils/Logger.hpp>

#include <scm/gl_core/render_device/opengl/gl_core.h>

#include <boost/filesystem.hpp>
#include <boost/asio/ip/host_name.hpp>
#include <boost/algorithm/string.hpp> 

// external headers
#include <iostream>
#include <fstream>
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

  Video3DRessource::Video3DRessource(std::string const& video3d) :
  ks_filename_(video3d),
  calib_files_(),
  server_endpoint_(),
  proxy_vertices_(),
  proxy_indices_(),
  proxy_vertex_array_(),
  rstate_solid_(),
  color_texArrays_(),
  color_buffers_(),
  depth_texArrays_(),
  depth_buffers_(),
  depth_size_(),
  depth_size_byte_(),
  color_size_(),
  file_buffers_(),
  width_depthimage_(),
  height_depthimage_(),
  width_colorimage_(),
  height_colorimage_(),
  upload_mutex_()
  {
    init(); 
  }

  ////////////////////////////////////////////////////////////////////////////////
  Video3DRessource::~Video3DRessource()
  {
    for (auto db : depth_buffers_ )
    {
      delete [] db;
    }

    for (auto cb : color_buffers_ )
    {
      delete [] cb;
    }

    for (auto fb : file_buffers_ )
    {
      delete fb;
    }
  }

////////////////////////////////////////////////////////////////////////////////
void Video3DRessource::init()
{
  std::fstream istr;
  istr.open(ks_filename_.c_str(), std::ios::in);

  boost::filesystem::path ks_filepath(ks_filename_);
  boost::filesystem::path ks_dir = ks_filepath.parent_path();

  if (istr.good())
  {
    std::string token;
    std::vector<std::string> hostnames;

    while (istr >> token)
    {
      if ( token == "serverport" )
      {
        istr >> server_endpoint_;
      } else if ( token == "kinect" ) 
      {
        istr >> token;
        std::string cf_absolute_path = ks_dir.string() + "/" + token;

        auto calib_file_ptr = std::make_shared<KinectCalibrationFile>(cf_absolute_path.c_str());

        calib_file_ptr->parse();
        calib_file_ptr->updateMatrices();

        calib_files_.push_back(calib_file_ptr);
      } else if ( token == "hostname" )
      {
        istr >> token;
        hostnames.push_back(token);
      }
    }

    std::string hostname = boost::asio::ip::host_name();
    boost::algorithm::to_lower(hostname);

    if ( std::find(hostnames.begin(), hostnames.end(), hostname) != hostnames.end() )
    {
      for ( auto calib_file : calib_files_ )
      {
        sys::FileBuffer* tmp = new sys::FileBuffer(calib_file->get_stream_filename());
        tmp->open();
        tmp->setLooping(true);
        file_buffers_.push_back(tmp);

        const unsigned pixelcountc = calib_file->getWidthC() * calib_file->getHeightC();

        depth_size_ = calib_file->getWidth() * calib_file->getHeight(); //== pixelcount
        color_size_ = pixelcountc * 3 * sizeof(unsigned char);
        depth_size_byte_ = depth_size_ * sizeof(float);

        if(calib_file->isCompressedRGB()){
            mvt::DXTCompressor dxt;
            dxt.init(calib_file->getWidthC(), calib_file->getHeightC(), FORMAT_DXT1);
            color_size_ = dxt.getStorageSize();
        }

        color_buffers_.push_back(new unsigned char[color_size_]);
        depth_buffers_.push_back(new float[depth_size_]);
      }

      assert (calib_files_.size() > 0);

      width_depthimage_  = calib_files_[0]->getWidth();
      height_depthimage_ = calib_files_[0]->getHeight();

      width_colorimage_  = calib_files_[0]->getWidthC();
      height_colorimage_ = calib_files_[0]->getHeightC();

    } else { // host is hostname list
      throw std::runtime_error("to implement");
    }

    istr.close();
  } else {
    throw std::runtime_error("Couldn't open calib file");
  }

}

///////////////////////////////////////////////////////////////////////////////
void Video3DRessource::upload_to(RenderContext const& ctx) const
{
  upload_proxy_mesh(ctx);

  upload_video_textures(ctx);
}

////////////////////////////////////////////////////////////////////////////////
void Video3DRessource::upload_proxy_mesh(RenderContext const& ctx) const
{
  if (proxy_vertices_.size() > ctx.id ) {
    return;
  } else {
    proxy_vertices_.resize(ctx.id + 1);
    proxy_indices_.resize(ctx.id + 1);
    proxy_vertex_array_.resize(ctx.id + 1);
  }

  int num_vertices = height_depthimage_ * width_depthimage_;
  int num_indices = height_depthimage_ * width_depthimage_;
  int num_triangles = ((height_depthimage_ - 1) * (width_depthimage_ - 1)) * 2;
  int num_triangle_indices = 3 * num_triangles;
  int num_line_indices = (height_depthimage_ - 1)*((width_depthimage_ - 1) * 3 + 1) + (width_depthimage_ - 1);

  float step = 1.0f / width_depthimage_;

  std::unique_lock<std::mutex> lock(upload_mutex_);

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
  for (float h = 0.5*step; h < height_depthimage_*step; h += step)
  {
    for (float w = 0.5*step; w < width_depthimage_*step; w += step)
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
  for (unsigned h(0); h < (height_depthimage_ - 1); ++h)
  {
    for (unsigned w(0); w < (width_depthimage_ - 1); ++w)
    {
      index_array[v] = (w + h * width_depthimage_);
      ++v;
      index_array[v] = (w + h * width_depthimage_ + 1);
      ++v;
      index_array[v] = (w + h * width_depthimage_ + width_depthimage_);
      ++v;
      index_array[v] = (w + h * width_depthimage_ + width_depthimage_);
      ++v;
      index_array[v] = (w + h * width_depthimage_ + 1);
      ++v;
      index_array[v] = (w + h * width_depthimage_ + 1 + width_depthimage_);
      ++v;
    }
  }

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

}

////////////////////////////////////////////////////////////////////////////////
void Video3DRessource::upload_video_textures(RenderContext const& ctx) const
{
  if ( color_texArrays_.size() > ctx.id ) {
    return;
  } else {
    rstate_solid_.resize(ctx.id + 1);
    color_texArrays_.resize(ctx.id + 1);
    depth_texArrays_.resize(ctx.id + 1);
  }

  // initialize Texture Arrays (kinect depths & colors)
  depth_texArrays_[ctx.id] = ctx.render_device->create_texture_2d(scm::math::vec2ui(width_depthimage_, height_depthimage_),
    scm::gl::FORMAT_R_32F,
    0,
    calib_files_.size(),
    1
    );

  color_texArrays_[ctx.id] = ctx.render_device->create_texture_2d(scm::math::vec2ui(width_colorimage_, height_colorimage_),
    calib_files_[0]->isCompressedRGB() ? scm::gl::FORMAT_BC1_RGBA : scm::gl::FORMAT_RGB_8,
    0,
    calib_files_.size(),
    1
    );

  rstate_solid_[ctx.id] = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID,
    scm::gl::CULL_NONE,
    scm::gl::ORIENT_CCW,
    true);
}

////////////////////////////////////////////////////////////////////////////////

void Video3DRessource::draw(RenderContext const& ctx) const
{
  scm::gl::context_vertex_input_guard vig(ctx.render_context);
  ctx.render_context->bind_vertex_array(proxy_vertex_array_[ctx.id]);
  ctx.render_context->bind_index_buffer(proxy_indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);

  ctx.render_context->apply();
  ctx.render_context->draw_elements(6 * (height_depthimage_ - 1) * (width_depthimage_ - 1));
}


////////////////////////////////////////////////////////////////////////////////
unsigned Video3DRessource::number_of_cameras() const {
  return unsigned(calib_files_.size());
}

////////////////////////////////////////////////////////////////////////////////
scm::gl::texture_2d_ptr const&
Video3DRessource::color_array(RenderContext const& context) const
{
  return color_texArrays_[context.id];
}

////////////////////////////////////////////////////////////////////////////////
scm::gl::texture_2d_ptr const&
Video3DRessource::depth_array(RenderContext const& context) const
{
  return depth_texArrays_[context.id];
}

////////////////////////////////////////////////////////////////////////////////

void Video3DRessource::update_buffers(RenderContext const& ctx) const
{
  // todo: if new frame -> use framecount instead
  if (ctx.id == 0)
  {
    for (unsigned i = 0; i < number_of_cameras(); ++i)
    {
      if (file_buffers_[i]->read((void*)color_buffers_[i], color_size_) != color_size_){
        std::cerr << "ERROR reading color BufferData\n";
      }

      if (file_buffers_[i]->read((void*)depth_buffers_[i], depth_size_byte_) != depth_size_byte_){
        std::cerr << "ERROR reading depth BufferData\n";
      }
    }
  }

  upload_to(ctx);

  // todo: if new frame 
  for(int i = 0; i < number_of_cameras(); ++i) 
  {
    ctx.render_context->update_sub_texture(depth_texArrays_[ctx.id],
                                  scm::gl::texture_region(scm::math::vec3ui(0, 0, i),
                                                          scm::math::vec3ui(width_depthimage_, height_depthimage_, 1)),
                                  0, //mip-mapping level
                                  scm::gl::FORMAT_R_32F,
                                  (void*) depth_buffers_[i]
                                );
  
    ctx.render_context->update_sub_texture(color_texArrays_[ctx.id],
                                  scm::gl::texture_region(scm::math::vec3ui(0, 0 , i),
                                                          scm::math::vec3ui(width_colorimage_, height_colorimage_, 1)),
                                  0, //mip-mapping level
                                  scm::gl::FORMAT_BC1_RGBA,
                                  (void*) color_buffers_[i]
                                );
  }
}

////////////////////////////////////////////////////////////////////////////////
KinectCalibrationFile const& Video3DRessource::calibration_file(unsigned i) const
{
  assert(i < calib_files_.size());
  return *calib_files_[i];
}

////////////////////////////////////////////////////////////////////////////////
/*virtual*/ GeometryUberShader* Video3DRessource::get_ubershader() const {
  return Singleton<Video3DUberShader>::instance();
}

}
