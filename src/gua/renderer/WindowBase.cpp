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
#include <gua/renderer/WindowBase.hpp>

// guacamole headers
#include <gua/config.hpp>
#include <gua/platform.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ResourceFactory.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>

#include <scm/gl_core/render_device/opengl/gl_core.h>

// external headers
#include <sstream>
#include <iostream>

#ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
extern "C"
{
#include <nvstusb/nvstusb.h>
}

#include <X11/Xlib.h>
#include <X11/extensions/xf86vmode.h>
#endif

namespace gua
{
std::string subroutine_from_mode(WindowBase::TextureDisplayMode mode)
{
    switch(mode)
    {
    case WindowBase::RED:
        return "get_red";
        break;
    case WindowBase::GREEN:
        return "get_green";
        break;
    case WindowBase::CYAN:
        return "get_cyan";
        break;
    case WindowBase::CHECKER_EVEN:
        return "get_checker_even";
        break;
    case WindowBase::CHECKER_ODD:
        return "get_checker_odd";
        break;
    case WindowBase::QUAD_BUFFERED_LEFT:
        return "get_quad_buffer_left";
        break;
    case WindowBase::QUAD_BUFFERED_RIGHT:
        return "get_quad_buffer_right";
        break;
    default:
        return "get_full";
    }
}

////////////////////////////////////////////////////////////////////////////////

std::atomic_uint WindowBase::last_context_id_{0};
std::mutex WindowBase::last_context_id_mutex_{};
WindowBase* WindowBase::current_instance_ = nullptr;

////////////////////////////////////////////////////////////////////////////////

WindowBase::WindowBase(Configuration const& configuration)
    : rendering_fps(1.0f), config(configuration), fullscreen_shader_(), fullscreen_quad_(), depth_stencil_state_(),
#ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
      nv_context_(nullptr),
#endif
      warpRR_(nullptr), warpGR_(nullptr), warpBR_(nullptr), warpRL_(nullptr), warpGL_(nullptr), warpBL_(nullptr)
{
}

////////////////////////////////////////////////////////////////////////////////

WindowBase::~WindowBase() { destroy_context(); }

void WindowBase::destroy_context()
{
    warpRR_ = nullptr;
    warpGR_ = nullptr;
    warpBR_ = nullptr;
    warpRL_ = nullptr;
    warpGL_ = nullptr;
    warpBL_ = nullptr;

    blend_state_.reset();
    depth_stencil_state_.reset();
    fullscreen_quad_.reset();
    fullscreen_shader_.program_.reset();

    ctx_.render_pipelines.clear();
    ctx_.render_context.reset();
    // ctx_.display.reset();
    ctx_.render_device.reset();

#ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
    if(nv_context_)
    {
        nvstusb_deinit(nv_context_);
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::init_context()
{
    if(config.get_warp_matrix_red_right() == "" || config.get_warp_matrix_green_right() == "" || config.get_warp_matrix_blue_right() == "" || config.get_warp_matrix_red_left() == "" ||
       config.get_warp_matrix_green_left() == "" || config.get_warp_matrix_blue_left() == "")
    {
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
        ResourceFactory factory;
        fullscreen_shader_.create_from_sources(factory.read_shader_file("resources/shaders/display_shader.vert"), factory.read_shader_file("resources/shaders/display_shader.frag"));
#else
        fullscreen_shader_.create_from_sources(Resources::lookup_shader(Resources::shaders_display_shader_vert), Resources::lookup_shader(Resources::shaders_display_shader_frag));
#endif
    }
    else
    {
        warpRR_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_red_right());

        warpGR_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_green_right());

        warpBR_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_blue_right());

        warpRL_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_red_left());

        warpGL_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_green_left());

        warpBL_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_blue_left());

#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
        ResourceFactory factory;
        fullscreen_shader_.create_from_sources(factory.read_shader_file("resources/shaders/display_shader.vert"), factory.read_shader_file("resources/shaders/display_shader_warped.frag"));
#else
        fullscreen_shader_.create_from_sources(Resources::lookup_shader(Resources::shaders_display_shader_vert), Resources::lookup_shader(Resources::shaders_display_shader_warped_frag));
#endif
    }
    ctx_.render_device = scm::gl::render_device_ptr(new scm::gl::render_device());
    ctx_.render_context = ctx_.render_device->main_context();

    {
        std::lock_guard<std::mutex> lock(last_context_id_mutex_);
        ctx_.id = last_context_id_++;
    }

    ctx_.render_window = this;

    fullscreen_quad_ = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(ctx_.render_device, scm::math::vec2f(-1.f, -1.f), scm::math::vec2f(1.f, 1.f)));

    depth_stencil_state_ = ctx_.render_device->create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER);

    blend_state_ = ctx_.render_device->create_blend_state(true, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE);
    if(config.get_debug())
    {
        ctx_.render_context->register_debug_callback(boost::make_shared<DebugOutput>());
    }

    ctx_.render_context->clear_default_color_buffer(scm::gl::FRAMEBUFFER_BACK, scm::math::vec4f(0.f, 0.f, 0.f, 1.0f));
    ctx_.render_context->clear_default_depth_stencil_buffer(); 
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::start_frame()
{
    process_events();

#ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
    // init nv_context_ for NVIDIA 3D Vision
    if(config.get_stereo_mode() == StereoMode::NVIDIA_3D_VISION && !nv_context_)
    {
        nv_context_ = nvstusb_init(GUACAMOLE_NVIDIA_3D_VISION_FIRMWARE_PATH);
        if(!nv_context_)
        {
            Logger::LOG_ERROR << "Could not initialize NVIDIA 3D Vision IR emitter!" << std::endl;
        }
        else
        {
            Display* display = XOpenDisplay(0);
            double display_num = DefaultScreen(display);
            XF86VidModeModeLine mode_line;
            int pixel_clk = 0;
            XF86VidModeGetModeLine(display, display_num, &pixel_clk, &mode_line);
            double frame_rate = (double)pixel_clk * 1000.0 / mode_line.htotal / mode_line.vtotal;
            Logger::LOG_MESSAGE << "Detected refresh rate of " << frame_rate << " Hz." << std::endl;
            nvstusb_set_rate(nv_context_, frame_rate);
        }
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::finish_frame() { 
    swap_buffers();
    ctx_.render_context->clear_default_color_buffer(scm::gl::FRAMEBUFFER_BACK, scm::math::vec4f(0.f, 0.f, 0.f, 1.0f));
    ctx_.render_context->clear_default_depth_stencil_buffer();   
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::display(scm::gl::texture_2d_ptr const& center_texture)
{
    display(center_texture, true);

    if(config.get_stereo_mode() != StereoMode::MONO)
    {
        display(center_texture, false);
    }
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::display(scm::gl::texture_2d_ptr const& texture, bool is_left)
{
    switch(config.get_stereo_mode())
    {
    case StereoMode::SIDE_BY_SIDE_SOFTWARE_MVR: 
    case StereoMode::SIDE_BY_SIDE_HARDWARE_MVR: 
    if(is_left) {
        display(texture, config.get_left_resolution(), config.get_left_position(), WindowBase::FULL, is_left, true);
        //display(texture, config.get_left_resolution(), config.get_right_position(), WindowBase::FULL, is_left, true);
    } else {
        //display(texture, config.get_right_resolution(), config.get_right_position(), WindowBase::FULL, is_left, true);        
    }

    break;
    case StereoMode::SIDE_BY_SIDE: // turn this to MVR rendering pass (software and hardware)
    case StereoMode::SEPARATE_WINDOWS:
    case StereoMode::MONO:



#ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
    case StereoMode::NVIDIA_3D_VISION:
#endif
        display(texture, is_left ? config.get_left_resolution() : config.get_right_resolution(), is_left ? config.get_left_position() : config.get_right_position(), WindowBase::FULL, is_left, true);
        break;
    case StereoMode::ANAGLYPH_RED_CYAN:
        display(texture,
                is_left ? config.get_left_resolution() : config.get_right_resolution(),
                is_left ? config.get_left_position() : config.get_right_position(),
                is_left ? WindowBase::RED : WindowBase::CYAN,
                is_left,
                is_left);
        break;
    case StereoMode::ANAGLYPH_RED_GREEN:
        display(texture,
                is_left ? config.get_left_resolution() : config.get_right_resolution(),
                is_left ? config.get_left_position() : config.get_right_position(),
                is_left ? WindowBase::RED : WindowBase::GREEN,
                is_left,
                is_left);
        break;
    case StereoMode::CHECKERBOARD:
        display(texture,
                is_left ? config.get_left_resolution() : config.get_right_resolution(),
                is_left ? config.get_left_position() : config.get_right_position(),
                is_left ? WindowBase::CHECKER_EVEN : WindowBase::CHECKER_ODD,
                is_left,
                true);
        break;
    case StereoMode::QUAD_BUFFERED:
        display(texture,
                is_left ? config.get_left_resolution() : config.get_right_resolution(),
                is_left ? config.get_left_position() : config.get_right_position(),
                is_left ? WindowBase::QUAD_BUFFERED_LEFT : WindowBase::QUAD_BUFFERED_RIGHT,
                is_left,
                true);
        break;
    default:
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////

RenderContext* WindowBase::get_context() { return &ctx_; }

////////////////////////////////////////////////////////////////////////////////

void WindowBase::display(scm::gl::texture_2d_ptr const& texture, math::vec2ui const& size, math::vec2ui const& position, TextureDisplayMode mode, bool is_left, bool clear)
{
    auto const& glapi = ctx_.render_context->opengl_api();
    glapi.glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if(config.get_stereo_mode() == StereoMode::QUAD_BUFFERED)
    {
        glapi.glDrawBuffer(is_left ? GL_BACK_LEFT : GL_BACK_RIGHT);
    }

    fullscreen_shader_.use(ctx_);
    uint64_t h = texture->native_handle();
    math::vec2ui handle(h & 0x00000000ffffffff, h & 0xffffffff00000000);
    fullscreen_shader_.set_uniform(ctx_, handle, "sampler");

    if(is_left)
    {
        if(warpRL_)
            fullscreen_shader_.set_uniform(ctx_, warpRL_->get_handle(ctx_), "warpR");
        if(warpGL_)
            fullscreen_shader_.set_uniform(ctx_, warpGL_->get_handle(ctx_), "warpG");
        if(warpBL_)
            fullscreen_shader_.set_uniform(ctx_, warpBL_->get_handle(ctx_), "warpB");
    }
    else
    {
        if(warpRR_)
            fullscreen_shader_.set_uniform(ctx_, warpRR_->get_handle(ctx_), "warpR");
        if(warpGR_)
            fullscreen_shader_.set_uniform(ctx_, warpGR_->get_handle(ctx_), "warpG");
        if(warpBR_)
            fullscreen_shader_.set_uniform(ctx_, warpBR_->get_handle(ctx_), "warpB");
    }

    std::string subroutine = subroutine_from_mode(mode);

    fullscreen_shader_.set_subroutine(ctx_, scm::gl::STAGE_FRAGMENT_SHADER, "get_color", subroutine);


    ctx_.render_context->set_depth_stencil_state(depth_stencil_state_);

    if(!clear)
    {
        ctx_.render_context->set_blend_state(blend_state_);
    }

    if(config.get_stereo_mode() == StereoMode::SIDE_BY_SIDE_SOFTWARE_MVR ||
       config.get_stereo_mode() == StereoMode::SIDE_BY_SIDE_HARDWARE_MVR ) {

        scm::math::vec2f const left_cam_pos = scm::math::vec2f(config.get_left_position());
        scm::math::vec2f const left_cam_res = scm::math::vec2f(config.get_left_resolution());
        scm::math::vec2f const right_cam_pos = scm::math::vec2f(config.get_right_position());
        scm::math::vec2f const right_cam_res = scm::math::vec2f(config.get_right_resolution());
        float const viewport_array[8]  = { left_cam_pos[0], left_cam_pos[1], left_cam_res[0], left_cam_res[1],
                                           right_cam_pos[0], right_cam_pos[1], right_cam_res[0], right_cam_res[1] };


        glapi.glViewportArrayv(0, 2, viewport_array);

        fullscreen_quad_->draw_instanced(ctx_.render_context, 2);
    } else {
        ctx_.render_context->set_viewport(scm::gl::viewport(position, size));
        fullscreen_quad_->draw(ctx_.render_context);        
    }
    ctx_.render_context->reset_state_objects();
    fullscreen_shader_.unuse(ctx_);
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::swap_buffers_callback() { WindowBase::current_instance_->swap_buffers_impl(); }

////////////////////////////////////////////////////////////////////////////////

void WindowBase::swap_buffers()
{
#ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
    if(config.get_stereo_mode() == StereoMode::NVIDIA_3D_VISION)
    {
        if(nv_context_)
        {
            current_instance_ = this;
            if((ctx_.framecount % 2) == 0)
            {
                nvstusb_swap(nv_context_, nvstusb_right, WindowBase::swap_buffers_callback);
            }
            else
            {
                nvstusb_swap(nv_context_, nvstusb_left, WindowBase::swap_buffers_callback);
            }
        }
    }
    else
    {
        swap_buffers_impl();
    }
#else
    swap_buffers_impl();
#endif
}

/* virtual */ math::mat4 WindowBase::get_latest_matrices(unsigned id) const { return math::mat4::identity(); }

////////////////////////////////////////////////////////////////////////////////

void WindowBase::DebugOutput::operator()(scm::gl::debug_source source, scm::gl::debug_type type, scm::gl::debug_severity severity, const std::string& message) const
{
    Logger::LOG_MESSAGE << "[Source: " << scm::gl::debug_source_string(source) << ", type: " << scm::gl::debug_type_string(type) << ", severity: " << scm::gl::debug_severity_string(severity)
                        << "]: " << message << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
