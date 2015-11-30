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
#include <gua/renderer/PipelineFactory.hpp>

// guacamole headers
#include <gua/renderer/StencilPass.hpp>
#include <gua/renderer/TriMeshPass.hpp>
#include <gua/renderer/EmissivePass.hpp>
#include <gua/renderer/PhysicallyBasedShadingPass.hpp>
#include <gua/renderer/LightVisibilityPass.hpp>
#include <gua/renderer/SSAOPass.hpp>
#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/TexturedQuadPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/ClearPass.hpp>
#include <gua/renderer/BackgroundPass.hpp>
#include <gua/renderer/ResolvePass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/SSAAPass.hpp>
#include <gua/renderer/GenerateWarpGridPass.hpp>
#include <gua/renderer/RenderWarpGridPass.hpp>
#include <gua/renderer/WarpPass.hpp>

#ifdef GUA_ENABLE_VOLUME
  #include <gua/volume.hpp>
#endif

#ifdef GUA_ENABLE_PLOD
  #include <gua/renderer/PLODPass.hpp>
#endif

namespace gua {

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelineDescription> PipelineFactory::make_pipeline(int caps) {
  auto pipe(std::make_shared<PipelineDescription>());
 
  pipe->add_pass(std::make_shared<ClearPassDescription>());

  if ((caps & DRAW_TEXTURED_QUADS) > 0) {
    pipe->add_pass(std::make_shared<TexturedQuadPassDescription>());
  }

    
  if ((caps & DRAW_TRIMESHES) > 0) {
    pipe->add_pass(std::make_shared<TriMeshPassDescription>());
  }

  #ifdef GUA_ENABLE_PLOD
    if ((caps & DRAW_PLODS) > 0) {
      pipe->add_pass(std::make_shared<PLODPassDescription>());
    }
  #endif

  // if ((caps & DRAW_NURBS) > 0) {
  //   pipe->add_pass(std::make_shared<NurbsPassDescription>());
  // }

  // if ((caps & DRAW_VIDEO3D) > 0) {
  //   pipe->add_pass(std::make_shared<Video3DPassDescription>());
  // }


  if ((caps & DRAW_BBOXES) > 0) {
    pipe->add_pass(std::make_shared<BBoxPassDescription>());
  }

  pipe->add_pass(std::make_shared<LightVisibilityPassDescription>());
  pipe->add_pass(std::make_shared<ResolvePassDescription>());

  if ((caps & WARPING) > 0) {
    pipe->add_pass(std::make_shared<GenerateWarpGridPassDescription>());
    pipe->add_pass(std::make_shared<WarpPassDescription>());
  }

  if ((caps & DEBUG_WARPING) > 0 && (caps & WARPING) > 0) {
    pipe->add_pass(std::make_shared<RenderWarpGridPassDescription>());
  }
  
  #ifdef GUA_ENABLE_VOLUME
    if ((caps & DRAW_VOLUMES) > 0) {
      pipe->add_pass(std::make_shared<VolumePassDescription>());
    }
  #endif

  if ((caps & DRAW_SCREEN_SPACE_TEXTURED_QUADS) > 0) {
    pipe->add_pass(std::make_shared<TexturedScreenSpaceQuadPassDescription>());
  }

  if ((caps & DEBUG_GBUFFER) > 0) {
    pipe->add_pass(std::make_shared<DebugViewPassDescription>());
  }

  pipe->set_enable_abuffer((caps & ABUFFER) > 0);

  return pipe; 
}

////////////////////////////////////////////////////////////////////////////////

}