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

#ifndef GUA_OCULUS_WINDOW_HPP
#define GUA_OCULUS_WINDOW_HPP

#if defined (_MSC_VER)
  #if defined (GUA_OCULUS_LIBRARY)
    #define GUA_OCULUS_DLL __declspec( dllexport )
  #else
#define GUA_OCULUS_DLL __declspec( dllimport )
  #endif
#else
  #define GUA_OCULUS_DLL
#endif // #if defined(_MSC_VER)

// guacamole headers
#include <gua/renderer/GlfwWindow.hpp>

//for the OVR members
#if defined (_WIN32)
    #include <OVR_CAPI.h>
    #include <OVR_CAPI_GL.h>
#else
    #include <gua/utils/OculusDistortionMesh.hpp>
#endif

#include <scm/gl_core/state_objects/state_objects_fwd.h>

namespace gua {


class GUA_OCULUS_DLL OculusWindow : public GlfwWindow {
 public:


  static void initialize_oculus_environment();
  static void shutdown_oculus_environment();

  OculusWindow(std::string const& display);
  virtual ~OculusWindow();

  void init_context() override;

  void recenter();

  // virtual
  void display(std::shared_ptr<Texture> const& texture, bool is_left);

  math::vec2 const get_left_screen_size() const;
  math::vec2 const get_right_screen_size() const;
  math::vec3 const get_left_screen_translation() const;
  math::vec3 const get_right_screen_translation() const;
  float const get_IPD() const;
  

  math::vec2ui get_eye_resolution() const;
  math::vec2ui get_window_resolution() const;
  math::mat4 get_oculus_sensor_orientation() const;

  void start_frame() override;
  void finish_frame() override;


  private:

    void calculate_viewing_setup();

    static bool oculus_environment_initialized_;
    static unsigned registered_oculus_device_count_;

    math::vec2 screen_size_[2];
    math::vec3 screen_translation_[2];

    math::mat4 oculus_sensor_orientation_;

    #ifdef _WIN32
        ovrSwapTextureSet* swap_texures_ = 0;
        ovrLayerEyeFov color_layer_;
        GLuint blit_fbo_read_;
        GLuint blit_fbo_write_;
    #else
        void initialize_distortion_meshes(ovrHmd const& hmd, RenderContext const& ctx);
        scm::gl::buffer_ptr distortion_mesh_vertices_[2];
        scm::gl::buffer_ptr distortion_mesh_indices_[2];
        scm::gl::vertex_array_ptr distortion_mesh_vertex_array_[2];

        // for distorted rendering as a replacement of the distortion shader
        unsigned num_distortion_mesh_indices_[2];
        scm::gl::rasterizer_state_ptr no_backface_culling_state_;
        bool has_vertical_display_;
    #endif
    // oculus device associated with the window
    ovrHmd registered_HMD_;
};

}

#endif  // GUA_OCULUS_WINDOW_HPP
