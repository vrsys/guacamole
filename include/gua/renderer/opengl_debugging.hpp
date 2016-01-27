/******************************************************************************
 * guacamole - delicious VR                                Release                   *
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

#ifndef GUA_OPENGL_DEBUGGING_HPP
#define GUA_OPENGL_DEBUGGING_HPP

// includes  -------------------------------------------------------------------
#include <scm/gl_core/render_device/opengl/gl_core.h>

#define GUA_PUSH_GL_RANGE(ctx, name) (ctx).render_context->opengl_api().glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, std::string(name).length(), std::string(name).c_str())
#define GUA_POP_GL_RANGE(ctx)        (ctx).render_context->opengl_api().glPopDebugGroup()

#endif  // GUA_OPENGL_DEBUGGING_HPP
