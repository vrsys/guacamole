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

#ifndef GUA_OCULUSSDK2_WINDOW_HPP
#define GUA_OCULUSSDK2_WINDOW_HPP

#if defined (_MSC_VER)
  #if defined (GUA_OCULUSSDK2_LIBRARY)
    #define GUA_OCULUSSDK2_DLL __declspec( dllexport )
  #else
#define GUA_OCULUSSDK2_DLL __declspec( dllimport )
  #endif
#else
  #define GUA_OCULUSSDK2_DLL
#endif // #if defined(_MSC_VER)

// guacamole headers
#include <gua/renderer/GlfwWindow.hpp>

//for the OVR members
#if defined (_WIN32)
    #include <OVR_CAPI.h>
#else
    #include <gua/utils/OculusSDK2DistortionMesh.hpp>
#endif

#include <scm/gl_core/state_objects/state_objects_fwd.h>

namespace gua {


class GUA_OCULUSSDK2_DLL OculusSDK2Window : public GlfwWindow {
 public:


  static void initialize_oculus_environment();
  static void shutdown_oculus_environment();

  OculusSDK2Window(std::string const& display);
  virtual ~OculusSDK2Window();

  void init_context() override;

  // virtual
  void display(std::shared_ptr<Texture> const& texture, bool is_left);

  gua::math::mat4 get_oculus_sensor_orientation() const;
  gua::math::vec2ui get_full_oculus_resolution() const;
  private:

    static bool oculus_environment_initialized_;
    static unsigned registered_oculus_device_count_;

    
    void retrieve_oculus_sensor_orientation(double absolute_time);

    #ifndef _WIN32
        void initialize_distortion_meshes(ovrHmd const& hmd, RenderContext const& ctx);
        scm::gl::buffer_ptr distortion_mesh_vertices_[2];
        scm::gl::buffer_ptr distortion_mesh_indices_[2];
        scm::gl::vertex_array_ptr distortion_mesh_vertex_array_[2];

        // for distorted rendering as a replacement of the distortion shader
        unsigned num_distortion_mesh_indices_[2];
    #endif

    // oculus device associated with the window
    ovrHmd registered_HMD_;

    // sensor orientation
    gua::math::mat4 oculus_sensor_orientation_;

    scm::gl::rasterizer_state_ptr no_backface_culling_state_;
};

}

#endif  // GUA_OCULUSSDK2_WINDOW_HPP
