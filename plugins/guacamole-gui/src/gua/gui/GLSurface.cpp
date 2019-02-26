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

#include "GLSurface.inl"

#include <gua/gui/GuiTexture.hpp>

namespace gua
{
///////////////////////////////////////////////////////////////////////////////
// ----------------------------------------------------------- public interface

// ----------------------------------------------------- contruction interface
GLSurface::GLSurface(unsigned width, unsigned height) : buffer_(width * height * 4), width_(width), height_(height), needs_update_() {}

// ------------------------------------------------------------ public methods

//////////////////////////////////////////////////////////////////////////////

bool GLSurface::bind(RenderContext const& ctx, const GuiTexture* gui_texture)
{
    while(needs_update_.size() <= ctx.id)
    {
        needs_update_.push_back(true);
    }

    if(needs_update_[ctx.id])
    {
        std::unique_lock<std::mutex> lock(mutex_);
        needs_update_[ctx.id] = false;
        gui_texture->update_sub_data(ctx, scm::gl::texture_region(math::vec3ui(0, 0, 0), math::vec3ui(width_, height_, 1)), 0u, scm::gl::FORMAT_BGRA_8, &buffer_.front());
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////

void GLSurface::Paint(unsigned char* src_buffer, int src_row_span, Awesomium::Rect const& src_rect, Awesomium::Rect const& dest_rect)
{
    std::unique_lock<std::mutex> lock(mutex_);

    for(int r = 0; r < dest_rect.height; r++)
    {
        auto row(height_ - r - dest_rect.y - 1);
        memcpy(&buffer_.front() + row * width_ * 4 + (dest_rect.x * 4), src_buffer + (r + src_rect.y) * src_row_span + (src_rect.x * 4), dest_rect.width * 4);
    }

    for(int i(0); i < needs_update_.size(); ++i)
    {
        needs_update_[i] = true;
    }
}

//////////////////////////////////////////////////////////////////////////////

void GLSurface::Scroll(int dx, int dy, Awesomium::Rect const& clip_rect)
{
    if(abs(dx) >= clip_rect.width || abs(dy) >= clip_rect.height)
    {
        return;
    }

    std::unique_lock<std::mutex> lock(mutex_);

    if(dx < 0 && dy == 0)
    {
        // Area shifted left by dx
        unsigned char* tempBuffer = new unsigned char[(clip_rect.width + dx) * 4];

        for(int i = 0; i < clip_rect.height; i++)
        {
            memcpy(tempBuffer, &buffer_.front() + (i + clip_rect.y) * width_ * 4 + (clip_rect.x - dx) * 4, (clip_rect.width + dx) * 4);
            memcpy(&buffer_.front() + (i + clip_rect.y) * width_ * 4 + (clip_rect.x) * 4, tempBuffer, (clip_rect.width + dx) * 4);
        }

        delete[] tempBuffer;
    }
    else if(dx > 0 && dy == 0)
    {
        // Area shifted right by dx
        unsigned char* tempBuffer = new unsigned char[(clip_rect.width - dx) * 4];

        for(int i = 0; i < clip_rect.height; i++)
        {
            memcpy(tempBuffer, &buffer_.front() + (i + clip_rect.y) * width_ * 4 + (clip_rect.x) * 4, (clip_rect.width - dx) * 4);
            memcpy(&buffer_.front() + (i + clip_rect.y) * width_ * 4 + (clip_rect.x + dx) * 4, tempBuffer, (clip_rect.width - dx) * 4);
        }

        delete[] tempBuffer;
    }
    else if(dy < 0 && dx == 0)
    {
        // Area shifted down by dy
        for(int i = 0; i < clip_rect.height + dy; i++)
        {
            memcpy(&buffer_.front() + (clip_rect.height - 1 - i - clip_rect.y) * width_ * 4 + (clip_rect.x * 4),
                   &buffer_.front() + (clip_rect.height - 1 - i - clip_rect.y + dy) * width_ * 4 + (clip_rect.x * 4),
                   clip_rect.width * 4);
        }
    }
    else if(dy > 0 && dx == 0)
    {
        // Area shifted up by dy
        for(int i = clip_rect.height - 1; i >= dy; i--)
        {
            memcpy(&buffer_.front() + (clip_rect.height - 1 - i - clip_rect.y) * width_ * 4 + (clip_rect.x * 4),
                   &buffer_.front() + (clip_rect.height - 1 - i - clip_rect.y + dy) * width_ * 4 + (clip_rect.x * 4),
                   clip_rect.width * 4);
        }
    }

    for(int i(0); i < needs_update_.size(); ++i)
    {
        needs_update_[i] = true;
    }
}

} // namespace gua
