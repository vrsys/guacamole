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

  /**
  * Resources associated with this context
  */
  std::unordered_map<std::size_t, std::shared_ptr<Pipeline>> render_pipelines;

 /**
  * Animated Bone Uniforms
  */
  std::unordered_map<node::Node*, std::shared_ptr<BoneTransformUniformBlock>> bone_transform_blocks;
};

}

#endif  // GUA_RENDERCONTEXT_HPP

