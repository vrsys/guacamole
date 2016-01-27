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
#include <gua/renderer/TriMeshPass.hpp>
#include <gua/renderer/LightVisibilityPass.hpp>
#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/TexturedQuadPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/ClearPass.hpp>
#include <gua/renderer/ResolvePass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/GenerateWarpGridPass.hpp>
#include <gua/renderer/WarpPass.hpp>

#ifdef GUA_ENABLE_VOLUME
  #include <gua/volume.hpp>
#endif

#ifdef GUA_ENABLE_PLOD
  #include <gua/renderer/PLODPass.hpp>
#endif

#ifdef GUA_ENABLE_LOD
  #include <gua/renderer/LodPass.hpp>
#endif

#ifdef GUA_ENABLE_VIDEO3D
  #include <gua/video3d/Video3DPass.hpp>
#endif

#ifdef GUA_ENABLE_NURBS
  #include <gua/renderer/NurbsPass.hpp>
#endif

#ifdef GUA_ENABLE_SKELANIM
  #include <gua/renderer/SkeletalAnimationPass.hpp>
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

  #ifdef GUA_ENABLE_SKELANIM
    if ((caps & DRAW_ANIMATED_TRIMESHES) > 0) {
      pipe->add_pass(std::make_shared<SkeletalAnimationPassDescription>());
    }
  #endif

  #ifdef GUA_ENABLE_PLOD
    if ((caps & DRAW_PLODS) > 0) {
      pipe->add_pass(std::make_shared<PLODPassDescription>());
    }
  #endif

  #ifdef GUA_ENABLE_LOD
    if ((caps & DRAW_LODS) > 0) {
      pipe->add_pass(std::make_shared<LodPassDescription>());
    }
  #endif

  #ifdef GUA_ENABLE_VIDEO3D
    if ((caps & DRAW_VIDEO3D) > 0) {
      pipe->add_pass(std::make_shared<Video3DPassDescription>());
    }
  #endif

  #ifdef GUA_ENABLE_NURBS
    if ((caps & DRAW_NURBS) > 0) {
      pipe->add_pass(std::make_shared<NurbsPassDescription>());
    }
  #endif


  if ((caps & DRAW_BBOXES) > 0) {
    pipe->add_pass(std::make_shared<BBoxPassDescription>());
  }

  pipe->add_pass(std::make_shared<LightVisibilityPassDescription>());
  pipe->add_pass(std::make_shared<ResolvePassDescription>());

  if ((caps & WARPING) > 0) {
    pipe->add_pass(std::make_shared<GenerateWarpGridPassDescription>());
    pipe->add_pass(std::make_shared<WarpPassDescription>());
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