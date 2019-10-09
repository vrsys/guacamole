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

#ifndef GUA_INCLUDE_RENDERER_HPP
#define GUA_INCLUDE_RENDERER_HPP

// renderer headers
#include <gua/config.hpp>
#include <gua/renderer/enums.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/LineStripLoader.hpp>
#include <gua/renderer/DynamicGeometryLoader.hpp>
#include <gua/renderer/DynamicLineLoader.hpp>
#include <gua/renderer/DynamicTriangleLoader.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/TriMeshPass.hpp>
#include <gua/renderer/LineStripPass.hpp>
#include <gua/renderer/DynamicGeometryPass.hpp>
#include <gua/renderer/DynamicLinePass.hpp>
#include <gua/renderer/DynamicTrianglePass.hpp>
#include <gua/renderer/LightVisibilityPass.hpp>
#include <gua/renderer/BackgroundPass.hpp>
#include <gua/renderer/ResolvePass.hpp>
#include <gua/renderer/SkyMapPass.hpp>
#include <gua/renderer/SSAOPass.hpp>
#include <gua/renderer/FullscreenPass.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/Renderer.hpp>
#include <gua/renderer/Window.hpp>
#include <gua/renderer/HeadlessSurface.hpp>
#include <gua/renderer/MaterialShader.hpp>
#include <gua/renderer/MaterialShaderDescription.hpp>
#include <gua/renderer/Material.hpp>
#ifdef GUACAMOLE_GLFW3
#include <gua/renderer/GlfwWindow.hpp>
#endif

#endif // GUA_INCLUDE_RENDERER_HPP
