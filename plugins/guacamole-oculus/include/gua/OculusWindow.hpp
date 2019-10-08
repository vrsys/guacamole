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

#if defined(_MSC_VER)
#if defined(GUA_OCULUS_LIBRARY)
#define GUA_OCULUS_DLL __declspec(dllexport)
#else
#define GUA_OCULUS_DLL __declspec(dllimport)
#endif
#else
#define GUA_OCULUS_DLL
#endif // #if defined(_MSC_VER)

// guacamole headers
#include <gua/renderer/GlfwWindow.hpp>

// for the Oculus SDK members
#if defined(_WIN32)
#include <OVR_CAPI.h>
#include <Extras/OVR_Math.h>
#include <OVR_CAPI_GL.h>
#else
#include <gua/utils/OculusDistortionMesh.hpp>
#endif

#include <scm/gl_core/state_objects/state_objects_fwd.h>

namespace gua
{
class GUA_OCULUS_DLL OculusWindow : public GlfwWindow
{
  public:
    OculusWindow(std::string const& display);
    virtual ~OculusWindow();

    float const get_IPD() const;
    math::vec2ui get_window_resolution() const;
    math::mat4 const& get_hmd_sensor_orientation() const;

    math::vec2 const& get_left_screen_size() const;
    math::vec2 const& get_right_screen_size() const;
    math::vec3 const& get_left_screen_translation() const;
    math::vec3 const& get_right_screen_translation() const;

    void display(std::shared_ptr<Texture> const& texture, bool is_left);

    void open() override;
    void init_context() override;
    void start_frame() override;
    void finish_frame() override;

    void recenter();

  private:
    void initialize_hmd_environment();
    void calculate_viewing_setup();

    std::string display_name_;
    math::mat4 hmd_sensor_orientation_;

    math::vec2 screen_size_[2];
    math::vec3 screen_translation_[2];

    unsigned int blit_fbo_read_;
    unsigned int blit_fbo_write_;

    ovrTextureSwapChain texture_swap_chain_ = 0;
    ovrTextureSwapChainDesc texture_swap_chain_desc_ = {};

    ovrLayerEyeFov color_layer_;

    ovrHmdDesc hmd_desc_;
    ovrSession hmd_session_;
    unsigned framecount_ = 0;
};

} // namespace gua

#endif // GUA_OCULUS_WINDOW_HPP
