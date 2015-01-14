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

#ifndef GUA_RENDERER_HPP
#define GUA_RENDERER_HPP

// external headers
#include <vector>
#include <string>
#include <memory>
#include <map>

#include <gua/platform.hpp>
#include <gua/utils/FpsCounter.hpp>
#include <gua/concurrent/Doublebuffer.hpp>

namespace gua {

class SceneGraph;
class Pipeline;

namespace node {
  class CameraNode;
  struct SerializedCameraNode;
}

/**
 * Manages the rendering on multiple contexts.
 *
 * This class is used to provide a renderer frontend interface to the user.
 */
class GUA_DLL Renderer {
 public:
  typedef std::vector<std::unique_ptr<const SceneGraph> > RenderVector;
  typedef RenderVector const                              ConstRenderVector;
  typedef std::shared_ptr<ConstRenderVector>              ConstRenderVectorPtr;

  /**
   * Constructor.
   *
   * This constructs a new Renderer.
   *
   * \param pipelines        A vector of Pipelines to process. For each
   *                         pipeline a RenderClient is created.
   */
  Renderer();
  Renderer(Renderer const&) = delete;
  Renderer& operator=(Renderer const&) = delete;

  /**
  *
  */
  ~Renderer();

  /**
   * Request a redraw of all RenderClients.
   *
   * Takes a Scenegraph and asks all clients to draw it.
   *
   * \param scene_graphs      The SceneGraphs to be processed.
   */
  void queue_draw(std::vector<SceneGraph const*> const& scene_graphs,
                  std::vector<std::shared_ptr<node::CameraNode>> const& cameras);

  void stop();

 private:

  typedef std::tuple<std::shared_ptr<node::SerializedCameraNode>, ConstRenderVectorPtr, std::shared_ptr<node::CameraNode>> Item;
  typedef std::shared_ptr<gua::concurrent::Doublebuffer<Item> > Mailbox;
  typedef std::pair<Mailbox, std::thread> Renderclient;

  static void renderclient(Mailbox in);

  std::map<std::string, Renderclient> render_clients_;

  FpsCounter application_fps_;
  bool stop_requested_;

};

}

#endif  // GUA_RENDERER_HPP
