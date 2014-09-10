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

#ifndef GUA_GEOMETRY_UBER_SHADER_HPP
#define GUA_GEOMETRY_UBER_SHADER_HPP

// guacamole headers
#include <gua/renderer/UberShader.hpp>

namespace gua {

struct View;

/**
 * This class represents a (multipass-) stage for rendering geometry into a layered fbo
 */
class GeometryUberShader : public UberShader {

 public: // typedefs, enums etc

   /**
   * logical combination of used render_stages provides valid rendermask, e.g:
   *
   * The rendermask ( PRE_FRAME_STAGE | PRE_DRAW_STAGE | DRAW_STAGE ) defines that
   * the ubershader provides an implementation of the defined stages
   */
   enum render_stage {
     NO_STAGE         = 0x00,
     PRE_FRAME_STAGE  = 0x01,
     PRE_DRAW_STAGE   = 0x02,
     DRAW_STAGE       = 0x04,
     POST_DRAW_STAGE  = 0x08,
     POST_FRAME_STAGE = 0x10
   };

   typedef unsigned stage_mask;

 public:

  /**
  * c'tor
  */
  GeometryUberShader();

  /**
  * d'tor
  */
  virtual ~GeometryUberShader();

  /**
  * provides information about which passes/stages area provided by this UberShader
  */
  virtual stage_mask get_stage_mask() const = 0;

  /**
  * This callback is called ONCE per frame BEFORE rendering all drawables of this type 
  *
  * default: no operations performed
  */
  virtual void preframe(RenderContext const& context) const = 0;

  /**
  * This method is called for ONCE per drawable to perform predraw operations
  *
  * default: no operations performed
  */
  virtual void predraw (  RenderContext const& context,
                          std::string const& name,
                          std::string const& material,
                          scm::math::mat4 const& model_matrix,
                          scm::math::mat4 const& normal_matrix,
                          Frustum const& frustum, 
                          View const& view ) const = 0;
   
  /**
  * This method is called for ONCE per drawable to perform draw operations
  *
  * default: no implementation provided
  */
  virtual void draw     ( RenderContext const& context,
                          std::string const& name,
                          std::string const& material,
                          scm::math::mat4 const& model_matrix,
                          scm::math::mat4 const& normal_matrix,
                          Frustum const& frustum,
                          View const& view) const = 0;

  /**
  * This method is called for ONCE per drawable to perform postdraw operations
  *
  * default: no operations performed
  */
  virtual void postdraw ( RenderContext const& context,
                          std::string const& name,
                          std::string const& material,
                          scm::math::mat4 const& model_matrix,
                          scm::math::mat4 const& normal_matrix,
                          Frustum const& frustum,
                          View const& view) const = 0;

  /**
  * This callback is called ONCE per frame AFTER rendering all drawables of this type
  *
  * default: no operations performed
  */
  virtual void postframe ( RenderContext const& context) const = 0;

};

}

#endif  // GUA_GEOMETRY_UBER_SHADER_HPP
