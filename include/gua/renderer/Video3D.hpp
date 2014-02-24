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

#ifndef GUA_VIDEO3D_HPP
#define GUA_VIDEO3D_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Geometry.hpp>
#include <gua/renderer/video3d_geometry/KinectCalibrationFile.hpp>
#include <gua/renderer/video3d_geometry/FileBuffer.h>
#include <gua/renderer/Texture2D.hpp>
#include <gua/renderer/ShaderProgram.hpp>

// external headers
#include <scm/gl_core.h>

#include <mutex>
#include <thread>

#include <vector>
#include <string>



namespace gua {

struct RenderContext;

/**
 * Stores geometry data.
 *
 * A mesh can be loaded from an Assimp mesh and the draw onto multiple
 * contexts.
 * Do not use this class directly, it is just used by the Geometry class to
 * store the individual meshes of a file.
 */
class Video3D : public Geometry {
 public:

  /**
   * constructor.
   *
   * Creates a new Video3D from a given video3d string.
   * \param video3d      Holds information about kinect streams.
  */
  Video3D(std::string const& video3d);

  /**
   * Draws the Video3D
   *
   * Draws the Mesh to the given context.
   *
   * \param context          The RenderContext to draw onto.
   */
  void draw(RenderContext const& context) const;
  
  /**
   * Raytest for Video3D
   *
   * Not implemented yet.
   *
   */
  void ray_test(Ray const& ray, PickResult::Options options,
                Node* owner, std::set<PickResult>& hits) 
  {}

  void set_uniforms(RenderContext const& ctx, ShaderProgram* cs);

 private:
  void upload_to(RenderContext const& context) const;
  void update_buffers(RenderContext const& context) const;
  
  std::string video3d_;

  mutable std::vector<scm::gl::buffer_ptr> proxy_vertices_;
  mutable std::vector<scm::gl::buffer_ptr> proxy_indices_;
  mutable std::vector<scm::gl::vertex_array_ptr> proxy_vertex_array_;

  mutable std::vector<scm::gl::sampler_state_ptr> sstate_;
  mutable std::vector<scm::gl::rasterizer_state_ptr> rstate_solid_;

  mutable std::vector<scm::gl::texture_2d_ptr> color_texArrays_;
  mutable std::vector<unsigned char*> color_buffers_; //std::vector< std::vector<unsigned char*> > for multiple kinects per Video3D
  mutable std::vector<scm::gl::texture_2d_ptr> depth_texArrays_;
  mutable std::vector<float*> depth_buffers_; //std::vector< std::vector<float*> > for multiple kinects per Video3D

  KinectCalibrationFile* calib_file_;
  mutable unsigned depth_size_;
  mutable unsigned depth_size_byte_;
  mutable unsigned color_size_;

  mutable unsigned width_;
  mutable unsigned height_;

  mutable std::vector<sys::FileBuffer*> file_buffers_;
  
  //members

  //width & height??
  //receiver??
  //flags??
  //toss out buffers

  mutable std::mutex upload_mutex_;
};

}

#endif  // GUA_VIDEO3D_HPP
