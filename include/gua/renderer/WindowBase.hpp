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

#ifndef GUA_WINDOW_BASE_HPP
#define GUA_WINDOW_BASE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/WarpMatrix.hpp>
#include <gua/renderer/enums.hpp>
#include <gua/math/math.hpp>
#include <gua/utils/configuration_macro.hpp>
#include <gua/config.hpp>

// external headers
#include <atomic>
#include <memory>
#include <string>
#include <scm/gl_util/primitives/quad.h>

#ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
struct nvstusb_context;
#endif

namespace gua
{
class Geometry;
class Texture;

/**
 * A base class for various windows.
 *
 * It's a window which can display OpenGL stuff.
 */
class GUA_DLL WindowBase
{
  public:
    enum TextureDisplayMode
    {
        FULL,
        RED,
        GREEN,
        CYAN,
        CHECKER_EVEN,
        CHECKER_ODD,
        QUAD_BUFFERED_LEFT,
        QUAD_BUFFERED_RIGHT
    };

    /**
     * A description of a window.
     *
     * Used when creating a window.
     */
    struct Configuration
    {
        GUA_ADD_PROPERTY(math::vec2ui, size, math::vec2ui(800, 600));
        GUA_ADD_PROPERTY(std::string, title, "guacamole");
#if WIN32
        GUA_ADD_PROPERTY(std::string, display_name, "\\\\.\\DISPLAY1");
#else
        GUA_ADD_PROPERTY(std::string, display_name, ":0.0");
#endif
        GUA_ADD_PROPERTY(int, monitor, 0);
        GUA_ADD_PROPERTY(StereoMode, stereo_mode, StereoMode::MONO);
        GUA_ADD_PROPERTY(math::vec2ui, left_resolution, math::vec2ui(800, 600));
        GUA_ADD_PROPERTY(math::vec2ui, left_position, math::vec2ui(0, 0));
        GUA_ADD_PROPERTY(math::vec2ui, right_resolution, math::vec2ui(800, 600));
        GUA_ADD_PROPERTY(math::vec2ui, right_position, math::vec2ui(0, 0));
        GUA_ADD_PROPERTY(math::vec2i, window_position, math::vec2i(0, 0));
        GUA_ADD_PROPERTY(bool, fullscreen_mode, false);
        GUA_ADD_PROPERTY(bool, enable_vsync, true);
        GUA_ADD_PROPERTY(bool, debug, false);
        GUA_ADD_PROPERTY(std::string, warp_matrix_red_right, "");
        GUA_ADD_PROPERTY(std::string, warp_matrix_green_right, "");
        GUA_ADD_PROPERTY(std::string, warp_matrix_blue_right, "");
        GUA_ADD_PROPERTY(std::string, warp_matrix_red_left, "");
        GUA_ADD_PROPERTY(std::string, warp_matrix_green_left, "");
        GUA_ADD_PROPERTY(std::string, warp_matrix_blue_left, "");

        // convenience access to resolution
        void set_resolution(math::vec2ui const& res) { left_resolution() = right_resolution() = res; }

        math::vec2ui const& get_resolution() { return get_left_resolution(); }

        // convenience access to position
        void set_position(math::vec2ui const& pos) { left_position() = right_position() = pos; }

        math::vec2ui const& get_position() { return get_left_position(); }

    } config;

    /**
     * Constructor.
     *
     * Creates a new WindowBase. It owns a RenderContext where Geomtries
     * can be drawn to.
     *
     * \param description   The description of the window.
     */
    WindowBase(Configuration const& configuration = Configuration());

    /**
     * Destructor.
     *
     * Cleans all associated memory.
     */
    virtual ~WindowBase();

    virtual void open() = 0;
    virtual bool get_is_open() const = 0;
    virtual bool should_close() const = 0;
    virtual void close() = 0;
    virtual void init_context();
    void destroy_context();

    /**
     * Activate the context of this window.
     *
     * Makes the RenderContext of this window current. All preceeding
     * OpenGL calls will be invoked on this window.
     */
    virtual void set_active(bool active) = 0;

    /**
     * Starts the drawing of a new frame.
     *
     * This should be called when a new frame is about to be drawn.
     */
    virtual void start_frame();

    /**
     * Ends the drawing of a new frame.
     *
     * This should be called when drawing a frame has been done.
     */
    virtual void finish_frame();

    /**
     *
     */
    virtual void display(scm::gl::texture_2d_ptr const& center_texture);

    virtual void display(scm::gl::texture_2d_ptr const& center_texture, bool is_left);

    virtual void process_events() = 0;

    /**
     * Get the RenderContext of this window.
     *
     * Can be called in order to retrieve the RenderContext of this
     * WindowBase.
     *
     * \return The context owned by this window.
     */
    RenderContext* get_context();

    float get_rendering_fps() { return rendering_fps; }
    std::atomic<float> rendering_fps;

	virtual math::mat4 get_latest_matrices(unsigned id) const;
	virtual int is_node_registered(std::string const& path) const { return -1; };

  protected:
    std::shared_ptr<WarpMatrix> warpRR_, warpGR_, warpBR_, warpRL_, warpGL_, warpBL_;

    virtual void swap_buffers_impl(){};

    struct GUA_DLL DebugOutput : public scm::gl::render_context::debug_output
    {
        /*virtual*/ void operator()(scm::gl::debug_source source, scm::gl::debug_type type, scm::gl::debug_severity severity, const std::string& message) const;
    };

    mutable RenderContext ctx_;
    ShaderProgram fullscreen_shader_;
    scm::gl::quad_geometry_ptr fullscreen_quad_;

    scm::gl::depth_stencil_state_ptr depth_stencil_state_;
    scm::gl::blend_state_ptr blend_state_;

    static std::atomic_uint last_context_id_;

    static std::mutex last_context_id_mutex_;

  private:
    void display(scm::gl::texture_2d_ptr const& texture, math::vec2ui const& size, math::vec2ui const& position, TextureDisplayMode mode = FULL, bool is_left = true, bool clear = true);

    void swap_buffers();

    static void swap_buffers_callback();
    static WindowBase* current_instance_;

#ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
    nvstusb_context* nv_context_;
#endif
};

} // namespace gua

#endif // GUA_WINDOW_BASE_HPP
