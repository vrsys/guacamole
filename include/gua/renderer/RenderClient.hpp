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

#ifndef GUA_RENDERCLIENT_HPP
#define GUA_RENDERCLIENT_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Timer.hpp>
#include <gua/utils/Doublebuffer.hpp>

// external headers
#include <thread>
#include <functional>

namespace gua {

/**
 * This class represents one render thread.
 *
 * The queue_draw method is directly called by the Renderer. Internally it
 * uses a threaded rendering loop which always waits for queue_draw calls. When
 * it fails to finish rendering before the next queue_draw is called, it will
 * ignore this call.
 */
template <typename T> class RenderClient {
 public:
  /**
   * Constructor.
   *
   * This constructs a new RenderClient.
   *
   */
  //RenderClient(std::function<void(T const&, float)> const& fun)
  template <typename F> RenderClient(F&& fun) : forever_(), doublebuffer_() {

#define FPS_CALCULATION_DELAY 20

    forever_ = std::thread([this, fun]() {
      float rendering_fps(0.f);
      unsigned rendering_frame_count(0);
      Timer rendering_timer;
      rendering_timer.start();

      while (true) {
        auto sg = this->doublebuffer_.read();
        fun(sg, rendering_fps);

        if (++rendering_frame_count == FPS_CALCULATION_DELAY) {
          rendering_fps = 1.f * FPS_CALCULATION_DELAY /
                          float(rendering_timer.get_elapsed());
          rendering_timer.reset();
          rendering_frame_count = 0;
        }
      }
    });
  }

  /**
   * Destructor.
   *
   * This destroys a RenderClient.
   */
  ~RenderClient() { forever_.detach(); }

  /**
   * Draw the scene.
   *
   * This requests a drawing operation of the given graph. If the client
   * is still processing the last call of this function it will be
   * ignored.
   *
   * \param graph            A pointer to the graph which
   *                         should be drawn.
   */
  inline void queue_draw(T const& scene_graphs) {
    doublebuffer_.write_blocked(scene_graphs);
  }

 private:
  std::thread forever_;
  utils::Doublebuffer<T> doublebuffer_;
};

}

#endif  // GUA_RENDERCLIENT_HPP
