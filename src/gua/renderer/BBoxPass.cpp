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
#include <gua/renderer/BBoxPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

BBoxPassDescription::BBoxPassDescription() : PipelinePassDescription() {
  vertex_shader_ = "shaders/bbox.vert";
  geometry_shader_ = "shaders/bbox.geom";
  fragment_shader_ = "shaders/bbox.frag";

  writes_only_color_buffer_ = false;
  rendermode_ = RenderMode::Callback;
  rasterizer_state_ = boost::make_optional(
      scm::gl::rasterizer_state_desc(scm::gl::FILL_SOLID,
                                     scm::gl::CULL_NONE,
                                     scm::gl::ORIENT_CCW,
                                     false,
                                     false,
                                     0.0f,
                                     false,
                                     true,
                                     scm::gl::point_raster_state(true)));
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* BBoxPassDescription::make_copy() const {
  return new BBoxPassDescription(*this);
}

PipelinePass BBoxPassDescription::make_pass(RenderContext const& ctx)
{
  PipelinePass pass{*this, ctx};

  auto count = 1;
  scm::gl::buffer_ptr buffer_ = ctx.render_device->create_buffer(
      scm::gl::BIND_VERTEX_BUFFER,
      scm::gl::USAGE_DYNAMIC_DRAW,
      count * 2 * sizeof(math::vec3),
      0);
  scm::gl::vertex_array_ptr vao_ = ctx.render_device->create_vertex_array(
      scm::gl::vertex_format(
        0, 0, scm::gl::TYPE_VEC3F, 2 * sizeof(math::vec3))(
        0, 1, scm::gl::TYPE_VEC3F, 2 * sizeof(math::vec3)), {buffer_});

  pass.process_ = [buffer_, vao_](
      PipelinePass &, PipelinePassDescription const&, Pipeline & pipe) {

    auto count(pipe.get_scene().bounding_boxes.size());

    if (count < 1)
      return;
    // else
    RenderContext const& ctx(pipe.get_context());

    ctx.render_device->resize_buffer(buffer_, count * 2 * sizeof(math::vec3));

    {
      auto data = static_cast<math::vec3*>(ctx.render_context->map_buffer(
          buffer_, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER));

      for (int i(0); i < count; ++i) {
        data[2 * i] = pipe.get_scene().bounding_boxes[i].min;
        data[2 * i + 1] = pipe.get_scene().bounding_boxes[i].max;
      }

      ctx.render_context->unmap_buffer(buffer_);
    }

    ctx.render_context->bind_vertex_array(vao_);

    ctx.render_context->apply();

    assert(count < std::numeric_limits<unsigned>::max());
    ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, unsigned(count));
  };

  return pass;
}

}
