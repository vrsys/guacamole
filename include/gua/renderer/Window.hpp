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

#ifndef GUA_WINDOW_HPP
#define GUA_WINDOW_HPP

// guacamole headers
#include <gua/renderer/WindowBase.hpp>
#include <atomic>

#if WIN32
#include <windows.h>
#include <scm/gl_core/gl_core_fwd.h>
#else
#include <X11/Xlib.h>
#include <GL/glx.h>
#endif

namespace gua
{
/**
 * A window for displaying stuff.
 *
 * It's a window which can display OpenGL stuff.
 */
class GUA_DLL Window : public WindowBase
{
  public:
    /**
     * Constructor.
     *
     * Creates a new Window. It owns a RenderContext where Geomtries
     * can be drawn to.
     *
     * \param description   The description of the window.
     */
    Window(Configuration const& configuration = Configuration());

    /**
     * Destructor.
     *
     * Cleans all associated memory.
     */
    virtual ~Window();

    void open() override;
    bool get_is_open() const override;
    bool should_close() const override;
    void close() override;

    void process_events() override;

    /**
     * Activate the context of this window.
     *
     * Makes the RenderContext of this window current. All preceeding
     * OpenGL calls will be invoked on this window.
     */
    void set_active(bool active) override;

    /**
     * join_swap_group adds window to the swap group specified by
     * group.  If window is already a member of a different group,
     * it is implicitly removed from that group first.
     * If group is zero, the window is unbound from its current group, if any.
     *
     * Swap Groups : windows in a single GPU
     */
    void join_swap_group(uint32_t group)
    {
        swap_group_ = group;
        ++swap_group_changes_;
    }
    void leave_swap_group()
    {
        swap_group_ = 0;
        ++swap_group_changes_;
    }
    uint32_t get_swap_group() const { return swap_group_; }
    uint32_t get_max_swap_groups() const { return max_swap_groups_; }

    /**
     * bind_swap_barrier
     * Swap Barrier : Swap Groups across GPUs
     */

    void bind_swap_barrier(uint32_t barrier)
    {
        swap_barrier_ = barrier;
        ++swap_barrier_changes_;
    }
    uint32_t get_swap_barrier() const { return swap_barrier_; }
    uint32_t get_max_swap_barriers() const { return max_swap_barriers_; }

  private:
    void swap_buffers_impl() override;

    scm::gl::wm::window_ptr scm_window_;

    bool has_NV_swap_group_ext_{false};
    uint32_t swap_group_{0u};
    std::atomic_uint swap_group_changes_{0u};
    uint32_t last_swap_group_changes_{0u};
    uint32_t max_swap_groups_{0u};

    uint32_t swap_barrier_{0u};
    std::atomic_uint swap_barrier_changes_{0u};
    uint32_t last_swap_barrier_changes_{0u};
    uint32_t max_swap_barriers_{0u};

    uint32_t frame_count_{0};

#if WIN32
    typedef unsigned int GLuint;
    typedef BOOL(WINAPI* PFNWGLBINDSWAPBARRIERNVPROC)(GLuint group, GLuint barrier);
    typedef BOOL(WINAPI* PFNWGLJOINSWAPGROUPNVPROC)(HDC hDC, GLuint group);
    typedef BOOL(WINAPI* PFNWGLQUERYFRAMECOUNTNVPROC)(HDC hDC, GLuint* count);
    typedef BOOL(WINAPI* PFNWGLQUERYMAXSWAPGROUPSNVPROC)(HDC hDC, GLuint* maxGroups, GLuint* maxBarriers);
    typedef BOOL(WINAPI* PFNWGLQUERYSWAPGROUPNVPROC)(HDC hDC, GLuint* group, GLuint* barrier);
    typedef BOOL(WINAPI* PFNWGLRESETFRAMECOUNTNVPROC)(HDC hDC);

    PFNWGLJOINSWAPGROUPNVPROC fpJoinSwapGroupNV;
    PFNWGLBINDSWAPBARRIERNVPROC fpBindSwapBarrierNV;
    PFNWGLQUERYSWAPGROUPNVPROC fpQuerySwapGroupNV;
    PFNWGLQUERYMAXSWAPGROUPSNVPROC fpQueryMaxSwapGroupsNV;
    PFNWGLQUERYFRAMECOUNTNVPROC fpQueryFrameCountNV;
    PFNWGLRESETFRAMECOUNTNVPROC fpResetFrameCountNV;
#else
    PFNGLXJOINSWAPGROUPNVPROC fpJoinSwapGroupNV;
    PFNGLXBINDSWAPBARRIERNVPROC fpBindSwapBarrierNV;
    PFNGLXQUERYSWAPGROUPNVPROC fpQuerySwapGroupNV;
    PFNGLXQUERYMAXSWAPGROUPSNVPROC fpQueryMaxSwapGroupsNV;
    PFNGLXQUERYFRAMECOUNTNVPROC fpQueryFrameCountNV;
    PFNGLXRESETFRAMECOUNTNVPROC fpResetFrameCountNV;
#endif
};

} // namespace gua

#endif // GUA_WINDOW_HPP
