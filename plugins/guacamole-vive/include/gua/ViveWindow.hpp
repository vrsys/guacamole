/***************************************************************************************
* guacamole - delicious VR                                                             *
*                                                                                      *
* Copyright: (c) 2011-2016 Bauhaus-Universität Weimar                                  *
* Contact:   philipp.rudloff@uni-weimar.de / nathalie.jolanthe.dittriech@uni-weimar.de *
*                                                                                      *
* This program is free software: you can redistribute it and/or modify it              *
* under the terms of the GNU General Public License as published by the Free           *
* Software Foundation, either version 3 of the License, or (at your option)            *
* any later version.                                                                   *
*                                                                                      *
* This program is distributed in the hope that it will be useful, but                  *
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY           *
* or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License             *
* for more details.                                                                    *
*                                                                                      *
* You should have received a copy of the GNU General Public License along              *
* with this program. If not, see <http://www.gnu.org/licenses/>.                       *
*                                                                                      *
***************************************************************************************/

#ifndef GUA_VIVE_WINDOW_HPP
#define GUA_VIVE_WINDOW_HPP

#if defined (_MSC_VER)
    #if defined (GUA_VIVE_LIBRARY)
        #define GUA_VIVE_DLL __declspec( dllexport )
    #else
        #define GUA_VIVE_DLL __declspec( dllimport )
    #endif
#else
    #define GUA_VIVE_DLL
#endif

// guacamole headers
#include <gua/renderer/GlfwWindow.hpp>

//for the OpenVR members
#if defined (_WIN32)
    #include <openvr.h>
    #include <openvr_capi.h>
#endif

namespace gua {

class GUA_VIVE_DLL ViveWindow : public GlfwWindow {
 public:

    ViveWindow(std::string const& display = ":0.0");
    virtual ~ViveWindow();

    float const get_IPD() const;
    math::vec2ui get_window_resolution() const;
    math::mat4 const& get_hmd_sensor_orientation() const;

    math::vec2 const& get_left_screen_size() const;
    math::vec2 const& get_right_screen_size() const;
    math::vec3 const& get_left_screen_translation() const;
    math::vec3 const& get_right_screen_translation() const;

    void display(scm::gl::texture_2d_ptr const& texture, bool is_left) override;

    void open() override;
    void init_context() override;
    void start_frame() override;
    void finish_frame() override;

 private:

    void initialize_hmd_environment();
    void calculate_viewing_setup();

    std::string display_name_;
    vr::IVRSystem *pVRSystem = nullptr;
    math::mat4 hmd_sensor_orientation_;

    math::vec2 screen_size_[2];
    math::vec3 screen_translation_[2];

    // GL Frame Buffers
    unsigned int blit_fbo_read_;
    unsigned int blit_fbo_write_;

    unsigned left_tex_id_ = 0;
    unsigned right_tex_id_ = 0;

    scm::gl::texture_2d_ptr left_texture_ = nullptr;
    scm::gl::texture_2d_ptr right_texture_ = nullptr;
};

}

#endif  // GUA_VIVE_WINDOW_HPP
