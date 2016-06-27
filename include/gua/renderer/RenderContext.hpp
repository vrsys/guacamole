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

#ifndef GUA_RENDERCONTEXT_HPP
#define GUA_RENDERCONTEXT_HPP

#include <gua/platform.hpp>
#include <gua/renderer/enums.hpp>
#include <gua/utils/InstanceCollection.hpp>
 
// external headers
#include <scm/gl_core/config.h>
#include <scm/gl_core/data_formats.h>
#include <scm/core.h>
#include <scm/gl_core.h>
#include <scm/gl_core/window_management/context.h>
#include <scm/gl_core/window_management/display.h>
#include <scm/gl_core/window_management/surface.h>
#include <atomic>

namespace gua {

class Pipeline;
class WindowBase;

namespace node{
  class Node;
}

/**
 * Information on a specific context.
 *
 * Stores all relevant information on a OpenGL context.
 */
struct GUA_DLL RenderContext {

  /**
  * c'tor
  */
  RenderContext();

  /**
   * The schism render device associated with this context.
   */
  scm::gl::render_device_ptr render_device;

  /**
   * The schism render constext associated with this context.
   */
  scm::gl::render_context_ptr render_context;

   /**
   * The schism context of this RenderContext.
   */
  scm::gl::wm::context_ptr context;

  /**
   * The display where this context was opened.
   */
  scm::gl::wm::display_ptr display;

  /**
   * The window which is rendered into.
   */
  WindowBase* render_window;

  /**
   * A unique ID for this context.
   */
  unsigned id;

  /**
  * framecounter for this context
  */
  unsigned framecount;

  gua::CameraMode mode;

  /**
  * Resources associated with this context
  */
  InstanceCollection resources;

  class Mesh
  {
    public:
      Mesh() = default;
      Mesh( scm::gl::vertex_array_ptr const& a
          , scm::gl::buffer_ptr const& v
          , scm::gl::buffer_ptr const& i)
        : vertex_array(a)
        , vertices(v)
        , indices(i)
        , indices_topology(scm::gl::PRIMITIVE_TRIANGLE_LIST)
        , indices_type(scm::gl::TYPE_UINT)
        , indices_count(0)
      {}
      scm::gl::vertex_array_ptr   vertex_array;
      scm::gl::buffer_ptr         vertices;
      scm::gl::buffer_ptr         indices;
      scm::gl::primitive_topology indices_topology;
      scm::gl::data_type          indices_type;
      int                         indices_count;
  };

  mutable std::unordered_map<std::size_t, Mesh> meshes;

  class Texture
  {
    public:
      Texture() = default;
      Texture(scm::gl::texture_image_ptr const& t,
              scm::gl::sampler_state_ptr const& s)
        : texture(t), sampler_state(s)
      {}
      scm::gl::texture_image_ptr texture;
      scm::gl::sampler_state_ptr sampler_state;
  };

  /**
  * Textures associated with this context
  */
  mutable std::unordered_map<std::size_t, Texture> textures;

  /**
  * Resources associated with this context
  */
  std::unordered_map<std::size_t, std::shared_ptr<Pipeline>> render_pipelines;
};

}

#endif  // GUA_RENDERCONTEXT_HPP

