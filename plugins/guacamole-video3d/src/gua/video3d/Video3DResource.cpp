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



namespace {

#if 0
  // not needed for proxy mesh
  struct Vertex {
    scm::math::vec3f pos;
    scm::math::vec2f tex;
    scm::math::vec3f normal;
    scm::math::vec3f tangent;
    scm::math::vec3f bitangent;
  };
#endif

  struct VertexOnly {
    scm::math::vec3f pos;
  };

}

namespace gua {

////////////////////////////////////////////////////////////////////////////////

Video3DResource::Video3DResource(std::string const& video3d, unsigned flags) :
  ks_filename_(video3d),
  calib_files_(),
  server_endpoint_(),
  rstate_solid_(),
  color_texArrays_(),
  depth_texArrays_(),
  depth_size_(),
  depth_size_byte_(),
  color_size_(),
  nka_per_context_(),
  cv_xyz_per_context_(),
  cv_uv_per_context_(),
  framecounter_per_context_(),
  width_depthimage_(),
  height_depthimage_(),
  width_colorimage_(),
  height_colorimage_(),
  upload_mutex_(),
  overwrite_normal_(false),
  o_normal_(),
  is_pickable_(flags & Video3DLoader::MAKE_PICKABLE)
{
  init();

  // pre resize for video shooting VRHyperspace
  rstate_solid_.resize(10);
  color_texArrays_.resize(10);
  depth_texArrays_.resize(10);
  nka_per_context_.resize(10);
  cv_xyz_per_context_.resize(10);
  cv_uv_per_context_.resize(10);
  framecounter_per_context_.resize(10);
}

////////////////////////////////////////////////////////////////////////////////
void Video3DResource::init()
{

  // approximately local space - can be overwritten from .ks file
  bounding_box_ = math::BoundingBox<math::vec3>(math::vec3(-1.5,-0.1,-1.0),
            math::vec3( 1.5, 2.2, 1.5));

  std::fstream istr;
  istr.open(ks_filename_.c_str(), std::ios::in);

  boost::filesystem::path ks_filepath(ks_filename_);
  boost::filesystem::path ks_dir = ks_filepath.parent_path();

  if (istr.good()) {
    std::string token;

    while (istr >> token) {
      if ( token == "serverport" ) {
        istr >> server_endpoint_;
      } else if ( token == "kinect" ) {
        istr >> token;
        std::string cf_absolute_path = ks_dir.string() + "/" + token;

        auto calib_file_ptr = std::make_shared<KinectCalibrationFile>(cf_absolute_path.c_str());

        calib_file_ptr->parse();
        calib_file_ptr->updateMatrices();
        //calib_file_ptr->printInfo();

        calib_files_.push_back(calib_file_ptr);
      }
      else if ( token == "bbx" ) {
  float x_min,y_min,z_min,x_max,y_max,z_max;
  istr >> x_min
       >> y_min
       >> z_min
       >> x_max
       >> y_max
       >> z_max;
  bounding_box_ = math::BoundingBox<math::vec3>(math::vec3(x_min,y_min,z_min),
                  math::vec3(x_max,y_max,z_max));
      }
      else if ( token == "normal" ){
  float x,y,z;
  istr >> x
       >> y
       >> z;
  overwrite_normal_ = true;
  o_normal_ = scm::math::vec3f(x,y,z);
      }

    }

    for ( auto calib_file : calib_files_ ){
      const unsigned pixelcountc = calib_file->getWidthC() * calib_file->getHeightC();

      depth_size_ = calib_file->getWidth() * calib_file->getHeight(); //== pixelcount
      color_size_ = pixelcountc * 3 * sizeof(unsigned char);
      depth_size_byte_ = depth_size_ * sizeof(float);

      if (calib_file->isCompressedRGB()) {
        mvt::DXTCompressor dxt;
        dxt.init(calib_file->getWidthC(), calib_file->getHeightC(), FORMAT_DXT1);
        color_size_ = dxt.getStorageSize();
      }
    }

    assert (calib_files_.size() > 0);

    width_depthimage_  = calib_files_[0]->getWidth();
    height_depthimage_ = calib_files_[0]->getHeight();

    width_colorimage_  = calib_files_[0]->getWidthC();
    height_colorimage_ = calib_files_[0]->getHeightC();

    istr.close();
  } else {
    throw std::runtime_error("Couldn't open calib file");
  }
}

///////////////////////////////////////////////////////////////////////////////
void Video3DResource::upload_to(RenderContext& ctx) const
{
  auto iter = ctx.meshes.find(uuid());
  if (iter == ctx.meshes.end()) {
    upload_proxy_mesh(ctx);
  }

  upload_video_textures(ctx);
}

////////////////////////////////////////////////////////////////////////////////
void Video3DResource::upload_proxy_mesh(RenderContext& ctx) const
{
  int num_vertices = height_depthimage_ * width_depthimage_;
  int num_indices = height_depthimage_ * width_depthimage_;
  int num_triangles = ((height_depthimage_ - 1) * (width_depthimage_ - 1)) * 2;
  int num_triangle_indices = 3 * num_triangles;
  int num_line_indices = (height_depthimage_ - 1)*((width_depthimage_ - 1) * 3 + 1) + (width_depthimage_ - 1);

  float step = 1.0f / width_depthimage_;

  RenderContext::Mesh proxy_mesh{};
  proxy_mesh.indices_topology = scm::gl::PRIMITIVE_TRIANGLE_LIST;
  proxy_mesh.indices_type = scm::gl::TYPE_UINT;
  //proxy_mesh.indices_count = 6 * (height_depthimage_ - 1) * (width_depthimage_ - 1);
  proxy_mesh.indices_count = num_triangle_indices;

  proxy_mesh.vertices =
    ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    num_vertices * sizeof(VertexOnly),
    0);

  VertexOnly* data(static_cast<VertexOnly*>(ctx.render_context->map_buffer(
                     proxy_mesh.vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  unsigned v(0);
  int pCount(0);
  for (float h = 0.5*step; h < height_depthimage_*step; h += step) {
    for (float w = 0.5*step; w < width_depthimage_*step; w += step) {
      data[v].pos = scm::math::vec3f(w, h, 0.0f);
#if 0
      //data[v].tex = scm::math::vec2f(w, h);
      //data[v].normal = scm::math::vec3f(0.0f, 0.0f, 1.0f);
      //data[v].tangent = scm::math::vec3(0.f, 0.f, 0.f);
      //data[v].bitangent = scm::math::vec3(0.f, 0.f, 0.f);
#endif
      ++v;
      ++pCount;
    }
  }

  ctx.render_context->unmap_buffer(proxy_mesh.vertices);

  std::vector<unsigned> index_array(num_triangle_indices);

  // compute index array (proxy mesh)
  v = 0;
  for (unsigned h(0); h < (height_depthimage_ - 1); ++h) {
    for (unsigned w(0); w < (width_depthimage_ - 1); ++w) {
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

  proxy_mesh.indices =
    ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    num_triangle_indices * sizeof(unsigned int),
    &index_array[0]);

  proxy_mesh.vertex_array = ctx.render_device->create_vertex_array
    (scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(VertexOnly))
#if 0
    (0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))
    (0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))
    (0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))
    (0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex))
#endif
     , { proxy_mesh.vertices });
  ctx.meshes[uuid()] = proxy_mesh;
  //ctx.render_context->apply(); // necessary ???
}

////////////////////////////////////////////////////////////////////////////////
void Video3DResource::upload_video_textures(RenderContext& ctx) const
{
  if ( color_texArrays_.size() > ctx.id ) {
    if (color_texArrays_[ctx.id]) {
      return;
    } else {
      // continue initialization
    }
  } else {
    rstate_solid_.resize(ctx.id + 1);
    color_texArrays_.resize(ctx.id + 1);
    depth_texArrays_.resize(ctx.id + 1);
    nka_per_context_.resize(ctx.id + 1);
    cv_xyz_per_context_.resize(ctx.id + 1);
    cv_uv_per_context_.resize(ctx.id + 1);
    framecounter_per_context_.resize(ctx.id + 1);
  }

  std::cout << "Upload video textures" << std::endl;

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

  nka_per_context_[ctx.id] = new video3d::NetKinectArray(calib_files_, server_endpoint_, color_size_, depth_size_byte_);

  // generate and download calibvolumes for this context
  for(unsigned i = 0; i < number_of_cameras(); ++i){
    std::vector< void* > raw_cv_xyz;
    raw_cv_xyz.push_back(calib_files_[i]->cv_xyz);
    // should be: glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB32F, cv_width, cv_height, cv_depth, 0, GL_RGB, GL_FLOAT, (unsigned char*) cv_xyz);
    cv_xyz_per_context_[ctx.id].push_back( ctx.render_device->create_texture_3d(scm::math::vec3ui(calib_files_[i]->cv_width,
                          calib_files_[i]->cv_height,
                          calib_files_[i]->cv_depth),
                    scm::gl::FORMAT_RGB_32F,
                    0,
                    scm::gl::FORMAT_RGB_32F,
                    raw_cv_xyz)
             );

    std::vector< void* > raw_cv_uv;
    raw_cv_uv.push_back(calib_files_[i]->cv_uv);
    // should be: glTexImage3D(GL_TEXTURE_3D, 0, GL_RG32F, cv_width, cv_height, cv_depth, 0, GL_RG, GL_FLOAT, (unsigned char*) cv_uv);
    cv_uv_per_context_[ctx.id].push_back( ctx.render_device->create_texture_3d(scm::math::vec3ui(calib_files_[i]->cv_width,
                         calib_files_[i]->cv_height,
                         calib_files_[i]->cv_depth),
                    scm::gl::FORMAT_RG_32F,
                    0,
                    scm::gl::FORMAT_RG_32F,
                    raw_cv_uv)
             );

  }

  framecounter_per_context_[ctx.id] = 0;
}

////////////////////////////////////////////////////////////////////////////////

void Video3DResource::draw(RenderContext& ctx) const
{
  auto iter = ctx.meshes.find(uuid());
  if (iter == ctx.meshes.end()) {
    upload_proxy_mesh(ctx);
  }

  iter = ctx.meshes.find(uuid());
  if (iter != ctx.meshes.end()) {
    scm::gl::context_vertex_input_guard vig(ctx.render_context);
    ctx.render_context->bind_vertex_array(iter->second.vertex_array);
    ctx.render_context->bind_index_buffer(iter->second.indices, iter->second.indices_topology, iter->second.indices_type);

    ctx.render_context->apply();
    ctx.render_context->draw_elements(iter->second.indices_count);
  }
}

////////////////////////////////////////////////////////////////////////////////
scm::gl::texture_2d_ptr const&
Video3DResource::color_array(RenderContext const& context) const
{
  return color_texArrays_[context.id];
}

////////////////////////////////////////////////////////////////////////////////
scm::gl::texture_2d_ptr const&
Video3DResource::depth_array(RenderContext const& context) const
{
  return depth_texArrays_[context.id];
}

////////////////////////////////////////////////////////////////////////////////
scm::gl::texture_3d_ptr const&
Video3DResource::cv_xyz (RenderContext const& context, unsigned camera_id) const
{
  return cv_xyz_per_context_[context.id][camera_id];
}

////////////////////////////////////////////////////////////////////////////////
scm::gl::texture_3d_ptr const&
Video3DResource::cv_uv (RenderContext const& context, unsigned camera_id) const
{
  return cv_uv_per_context_[context.id][camera_id];
}

////////////////////////////////////////////////////////////////////////////////

void Video3DResource::update_buffers(RenderContext& ctx) const
{
  upload_to(ctx);

  if(framecounter_per_context_[ctx.id] != ctx.framecount){
    framecounter_per_context_[ctx.id] = ctx.framecount;
  } else {
    return;
  }
  if (nka_per_context_[ctx.id]->update()) {
    unsigned char* buff = nka_per_context_[ctx.id]->getBuffer();
    for(int i = 0; i < number_of_cameras(); ++i) {

      ctx.render_context->update_sub_texture(color_texArrays_[ctx.id],
               scm::gl::texture_region(scm::math::vec3ui(0, 0 , i),
                     scm::math::vec3ui(width_colorimage_, height_colorimage_, 1)),
               0, //mip-mapping level
               calib_files_[0]->isCompressedRGB() ? scm::gl::FORMAT_BC1_RGBA : scm::gl::FORMAT_RGB_8,
               (void*) buff
               );
      buff += color_size_;
      ctx.render_context->update_sub_texture(depth_texArrays_[ctx.id],
               scm::gl::texture_region(scm::math::vec3ui(0, 0, i),
                     scm::math::vec3ui(width_depthimage_, height_depthimage_, 1)),
               0, //mip-mapping level
               scm::gl::FORMAT_R_32F,
               (void*) buff
               );
      buff += depth_size_byte_;
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
KinectCalibrationFile const& Video3DResource::calibration_file(unsigned i) const
{
  assert(i < calib_files_.size());
  return *calib_files_[i];
}

}
