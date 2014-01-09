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
#include <gua/utils/logger.hpp>

namespace {
struct Vertex {
  scm::math::vec3f pos;
  scm::math::vec3f nrm;
  scm::math::vec2f tex;
};
}

namespace gua {

////////////////////////////////////////////////////////////////////////////////

Video3D::Video3D(std::string const& kinectFile) :
  proxy_vertices_(),
  proxy_indices_(),
  proxy_vertex_array_(),
  color_texArrays_(),
  color_buffers_(),
  depth_texArrays_(),
  depth_buffers_(),
  upload_mutex_()
{}

////////////////////////////////////////////////////////////////////////////////

void Video3D::upload_to(RenderContext const& ctx) const {
  
  //proxy mesh config
  unsigned int height = 480;
  unsigned int width  = 640;

  int num_vertices = height * width;
  int num_indices  = height * width;
  int num_triangles           = ((height-1) * (width-1)) * 2;
  int num_triangle_indices    = 3 * num_triangles;
  int num_line_indices        = (height -1)*((width-1)*3 + 1) + (width-1);

  float stepX = 1.0f / width * height;
  float stepY = 1.0f / height * height;
  float step = 1.0f / width;


  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (proxy_vertices_.size() <= ctx.id) {
    proxy_vertices_.resize(ctx.id + 1);
    proxy_indices_.resize(ctx.id + 1);
    proxy_vertex_array_.resize(ctx.id + 1);
  }

  proxy_vertices_[ctx.id] =
      ctx.render_device->create_buffer(BIND_VERTEX_BUFFER,
                                       USAGE_STATIC_DRAW,
                                       num_vertices * sizeof(Vertex),
                                       0);



  Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(
      proxy_vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));
  
  //compute vertex data (proxy mesh)
  const scm::math::vec3f& p(0.0f); //point of origin
  unsigned v(0);
  for (float h = 0.5*step; h < height*step; h += step)
  {
    for (float w = 0.5*step; w < width*step; w += step)
    {
        data[v].pos = scm::math::vec3f(p.x+w, p.y+h,0);
        data[v].nrm = scm::math::vec3f(0.0f, 0.0f, 1.0f);
        data[v].tex = scm::math::vec2f(p.x+w, p.y+h);
        ++v;
    }
  }

  ctx.render_context->unmap_buffer(proxy_vertices_[ctx.id]);

  std::vector<unsigned> index_array(num_triangle_indices);

  //coumpute index array (proxy mesh)
  v = 0;
  for (unsigned h(0); h < (height-1); ++h)
  {
    for (unsigned w(0); w < (width-1); ++w)
    {
            index_array[v] = (w + h * width);
            ++v;
            index_array[v] = (w + h * width + 1);
            ++v;
            index_array[v] = (w + h * width + width);
            ++v;
            index_array[v] = (w + h * width + width);
            ++v;
            index_array[v] = (w + h * width + 1);
            ++v;
            index_array[v] = (w + h * width + 1 + width);
            ++v;
    }
  }

  proxy_indices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       num_triangle_indices * sizeof(unsigned),
                                       &index_array[0]);

  std::vector<scm::gl::buffer_ptr> buffer_arrays;
  buffer_arrays.push_back(proxy_vertices_[ctx.id]);

  proxy_vertex_array_[ctx.id] = ctx.render_device->create_vertex_array
                                      (vertex_format(0, 0, TYPE_VEC3F, sizeof(Vertex))
                                                    (0, 1, TYPE_VEC3F, sizeof(Vertex))
                                                    (0, 2, TYPE_VEC2F, sizeof(Vertex)),
                                       buffer_arrays);

  //initialize Texture Arrays (kinect depths & colors)
  depth_texArrays[ctx.id] = ctx.render_device->create_texture_2d(scm::math::vec2ui(640, 480),
                                                       scm::gl::FORMAT_R_32F,
                                                       0,
                                                       1, //kinect count
                                                       1
                                                    );

  color_texArrays[ctx.id] = ctx.render_device->create_texture_2d( scm::math::vec2ui(640, 480),
                                                       scm::gl::FORMAT_BC1_RGBA,
                                                       0,
                                                       1, //kinect count
                                                       1
                                                   );
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

  //update kinect color & depth texture array for given context
  _context->update_sub_texture(depth_texArrays[ctx.id],
                                //scm::gl::texture_region(scm::math::vec3ui(0, 0, i),
                                scm::gl::texture_region(scm::math::vec3ui(0, 0, 0),
                                                       scm::math::vec3ui(640, 480, 1)),
                                0, //mip-mapping level
                                scm::gl::FORMAT_R_32F,
                                (void*) depth_buffers_[ctx.id]
                              );

  _context->update_sub_texture(color_texArrays[ctx.id],
                                //scm::gl::texture_region(scm::math::vec3ui(0, 0, i),
                                scm::gl::texture_region(scm::math::vec3ui(0, 0 , 0),
                                                       scm::math::vec3ui(640, 480, 1)),
                                0, //mip-mapping level
                                scm::gl::FORMAT_BC1_RGBA,
                                (void*) color_buffers_[ctx.id]
                              );

  ctx.render_context->apply();
  ctx.render_context->draw_elements(479 * 639 * 2);//mesh_->mNumFaces * 3
}

}
