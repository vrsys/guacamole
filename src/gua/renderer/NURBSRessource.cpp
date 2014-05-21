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
#include <gua/renderer/NURBSRessource.hpp>

#include <gua/utils/Singleton.hpp>
#include <gua/renderer/NURBSUberShader.hpp>

#include <scm/gl_core/render_device.h>
#include <scm/gl_core/buffer_objects.h>
#include <scm/gl_core/shader_objects.h>
#include <scm/gl_core/texture_objects.h>
#include <scm/gl_core/render_device/opengl/util/assert.h>
#include <scm/gl_core/data_formats.h>
#include <scm/gl_core/constants.h>

#include <boost/assign/list_of.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
NURBSRessource::NURBSRessource(std::shared_ptr<TrimmedBezierSurfaceObject> const& object,
             scm::gl::fill_mode in_fill_mode,
             std::size_t max_tf_size)
    : _data(new NURBSData(object)),
      _fill_mode(in_fill_mode),
      _max_transform_feedback_buffer_size(max_tf_size) {
  bounding_box_ = math::BoundingBox<math::vec3>(
      math::vec3(
          object->bbox().min[0], object->bbox().min[1], object->bbox().min[2]),
      math::vec3(
          object->bbox().max[0], object->bbox().max[1], object->bbox().max[2]));
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */
NURBSRessource::~NURBSRessource() 
{
  for (auto it  : _parametric_texture_buffer)    it.reset();
  for (auto it :  _attribute_texture_buffer) it.reset();  
  for (auto it :  _vertex_buffer) it.reset();  
  for (auto it :  _vertex_array) it.reset();  
  for (auto it :  _domain_texture_buffer) it.reset();  
  for (auto it :  _trim_partition_texture_buffer) it.reset();  
  for (auto it :  _trim_contourlist_texture_buffer) it.reset();  
  for (auto it :  _trim_curvelist_texture_buffer) it.reset();  
  for (auto it :  _trim_curvedata_texture_buffer) it.reset();  
  for (auto it :  _trim_pointdata_texture_buffer) it.reset();

  if (_data) {
    delete _data;
  }
}

////////////////////////////////////////////////////////////////////////////////

void NURBSRessource::upload_to(RenderContext const& context) const {

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (_vertex_array.size() <= context.id) {
    _vertex_array.resize(context.id + 1);
    _parametric_texture_buffer.resize(context.id + 1);
    _attribute_texture_buffer.resize(context.id + 1);
    _domain_texture_buffer.resize(context.id + 1);
    _trim_partition_texture_buffer.resize(context.id + 1);
    _trim_contourlist_texture_buffer.resize(context.id + 1);
    _trim_curvelist_texture_buffer.resize(context.id + 1);
    _trim_curvedata_texture_buffer.resize(context.id + 1);
    _trim_pointdata_texture_buffer.resize(context.id + 1);
    _vertex_buffer.resize(context.id + 1);
    _index_buffer.resize(context.id + 1);
    _transform_feedback_vbo.resize(context.id + 1);
    _transform_feedback.resize(context.id + 1);
    _transform_feedback_vao.resize(context.id + 1);
    _sstate.resize(context.id + 1);
    _rstate_ms_solid.resize(context.id + 1);
    _rstate_ms_wireframe.resize(context.id + 1);
    _rstate_ms_point.resize(context.id + 1);
    _bstate_no_blend.resize(context.id + 1);
  }

  auto in_device = context.render_device;

  _sstate[context.id] = in_device->create_sampler_state(
      scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
  _rstate_ms_solid[context.id] = in_device->create_rasterizer_state(
      scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false);
  _rstate_ms_wireframe[context.id] = in_device->create_rasterizer_state(
      scm::gl::FILL_WIREFRAME, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false);
  _rstate_ms_point[context.id] = in_device->create_rasterizer_state(
      scm::gl::FILL_POINT, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false);
  _bstate_no_blend[context.id] =
      in_device->create_blend_state(false,
                                    scm::gl::FUNC_ONE,
                                    scm::gl::FUNC_ZERO,
                                    scm::gl::FUNC_ONE,
                                    scm::gl::FUNC_ZERO);

  //Initialize Texture2D Buffers
  initialize_texture_buffers(context);

  //Initialize Vertex Data
  initialize_vertex_data(context);

  //Initialize Transform Feedback
  initialize_transform_feedback(context);
}

////////////////////////////////////////////////////////////////////////////////

void NURBSRessource::predraw(RenderContext const& context) const {
  // upload to GPU if neccessary
  if (_vertex_array.size() <= context.id || _vertex_array[context.id] == nullptr) {
    upload_to(context);
  }

  auto in_context = context.render_context;

  scm::gl::context_vertex_input_guard cvg(in_context);
  scm::gl::context_state_objects_guard csg(in_context);
  scm::gl::context_image_units_guard cig(in_context);
  scm::gl::context_texture_units_guard ctg(in_context);

  context.render_context->set_rasterizer_state(_rstate_ms_solid[context.id], 1.0f);

  //Transform Feedback Stage Begins
  in_context->begin_transform_feedback(_transform_feedback[context.id],
                                       scm::gl::PRIMITIVE_POINTS);
  {
    in_context->bind_vertex_array(_vertex_array[context.id]);
    in_context->bind_index_buffer(
        _index_buffer[context.id],
        scm::gl::PRIMITIVE_PATCH_LIST_4_CONTROL_POINTS,
        scm::gl::TYPE_UINT);

    in_context->bind_texture(
        _parametric_texture_buffer[context.id], _sstate[context.id], 5);
    in_context->bind_texture(
        _attribute_texture_buffer[context.id], _sstate[context.id], 6);
    in_context->bind_texture(
        _trim_partition_texture_buffer[context.id], _sstate[context.id], 7);
    in_context->bind_texture(
        _trim_contourlist_texture_buffer[context.id], _sstate[context.id], 8);
    in_context->bind_texture(
        _trim_curvelist_texture_buffer[context.id], _sstate[context.id], 9);
    in_context->bind_texture(
        _trim_curvedata_texture_buffer[context.id], _sstate[context.id], 10);
    in_context->bind_texture(
        _trim_pointdata_texture_buffer[context.id], _sstate[context.id], 11);

    in_context->current_program()->uniform_sampler("parameter_texture", 5);
    in_context->current_program()->uniform_sampler("attribute_texture", 6);
    in_context->current_program()->uniform_sampler("trim_partition", 7);
    in_context->current_program()->uniform_sampler("trim_contourlist", 8);
    in_context->current_program()->uniform_sampler("trim_curvelist", 9);
    in_context->current_program()->uniform_sampler("trim_curvedata", 10);
    in_context->current_program()->uniform_sampler("trim_pointdata", 11);

    in_context->apply();

    in_context->draw_elements(_data->index_data.size());
  }

  in_context->end_transform_feedback();

#if 0
  struct vertex {
    scm::math::vec3f pos;
    unsigned index;
    scm::math::vec2f uv;
  };

  std::vector<vertex> data(128);

  in_context->get_buffer_sub_data(_transform_feedback_vbo[context.id],
                                  0,
                                  sizeof(vertex) * data.size(),
                                  (void*)(&data[0]));

  for (auto k : data) {
    std::cout << k.pos << " " << k.index << " " << k.uv << " " << k.normal
              << std::endl;
  }
  std::cout << "===================" << std::endl;
#endif
}

////////////////////////////////////////////////////////////////////////////////

void NURBSRessource::draw(RenderContext const& context) const {
  // upload to GPU if neccessary
  if (_vertex_array.size() <= context.id || _vertex_array[context.id] == nullptr) {
    upload_to(context);
  }

  auto in_context = context.render_context;

  scm::gl::context_vertex_input_guard cvg1(in_context);
  scm::gl::context_state_objects_guard csg1(in_context);
  scm::gl::context_image_units_guard cig1(in_context);
  scm::gl::context_texture_units_guard ctg1(in_context);

  context.render_context->set_rasterizer_state(_rstate_ms_solid[context.id], 1.0f);

  in_context->bind_vertex_array(_transform_feedback_vao[context.id]);

  in_context->bind_texture(
      _parametric_texture_buffer[context.id], _sstate[context.id], 5);
  in_context->bind_texture(
      _attribute_texture_buffer[context.id], _sstate[context.id], 6);
  in_context->bind_texture(
      _trim_partition_texture_buffer[context.id], _sstate[context.id], 7);
  in_context->bind_texture(
      _trim_contourlist_texture_buffer[context.id], _sstate[context.id], 8);
  in_context->bind_texture(
      _trim_curvelist_texture_buffer[context.id], _sstate[context.id], 9);
  in_context->bind_texture(
      _trim_curvedata_texture_buffer[context.id], _sstate[context.id], 10);
  in_context->bind_texture(
      _trim_pointdata_texture_buffer[context.id], _sstate[context.id], 11);

  in_context->current_program()->uniform_sampler("parameter_texture", 5);
  in_context->current_program()->uniform_sampler("attribute_texture", 6);
  in_context->current_program()->uniform_sampler("trim_partition", 7);
  in_context->current_program()->uniform_sampler("trim_contourlist", 8);
  in_context->current_program()->uniform_sampler("trim_curvelist", 9);
  in_context->current_program()->uniform_sampler("trim_curvedata", 10);
  in_context->current_program()->uniform_sampler("trim_pointdata", 11);

  in_context->apply();

  in_context->draw_transform_feedback(
      scm::gl::PRIMITIVE_PATCH_LIST_4_CONTROL_POINTS,
      _transform_feedback[context.id]);
}

void NURBSRessource::initialize_texture_buffers(RenderContext const& context) const {
  auto in_device = context.render_device;

  if (_data->parametric_data.empty()) _data->parametric_data.resize(1);
  if (_data->attribute_data.empty()) _data->attribute_data.resize(1);
  if (_data->trim_partition.empty()) _data->trim_partition.resize(1);
  if (_data->trim_contourlist.empty()) _data->trim_contourlist.resize(1);
  if (_data->trim_curvelist.empty()) _data->trim_curvelist.resize(1);
  if (_data->trim_curvedata.empty()) _data->trim_curvedata.resize(1);
  if (_data->trim_pointdata.empty()) _data->trim_pointdata.resize(1);


  //Parametric Data
  _parametric_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       _data->parametric_data.data_size(),
                                       _data->parametric_data.get());

  //Attributes Data
  _attribute_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       _data->attribute_data.data_size(),
                                       _data->attribute_data.get());

  //Trim Data
  _trim_partition_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       _data->trim_partition.data_size(),
                                       _data->trim_partition.get());
  ;
  _trim_contourlist_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_RG_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       _data->trim_contourlist.data_size(),
                                       _data->trim_contourlist.get());
  ;
  _trim_curvelist_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       _data->trim_curvelist.data_size(),
                                       _data->trim_curvelist.get());
  ;
  _trim_curvedata_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_R_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       _data->trim_curvedata.data_size(),
                                       _data->trim_curvedata.get());
  ;
  _trim_pointdata_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_RGB_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       _data->trim_pointdata.data_size(),
                                       _data->trim_pointdata.get());
  ;
}

void NURBSRessource::initialize_vertex_data(RenderContext const& context) const {
  auto in_device = context.render_device;

  int stride =
      sizeof(scm::math::vec3f) + sizeof(unsigned) + sizeof(scm::math::vec4f);

  scm::gl::vertex_format v_fmt = scm::gl::vertex_format(
      0,
      0,
      scm::gl::TYPE_VEC3F,
      stride);  // Vertex Position (xf, yf, zf), v_size = a chunk you will pick
                // each time from buffer
  v_fmt(0, 1, scm::gl::TYPE_UINT, stride);   // Surface Index (indexf)
  v_fmt(0, 2, scm::gl::TYPE_VEC4F, stride);  // uv, tmp, tmp

  _vertex_buffer[context.id] =
      in_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                               scm::gl::USAGE_STATIC_DRAW,
                               _data->patch_data.data_size(),
                               _data->patch_data.get());
  _vertex_array[context.id] = in_device->create_vertex_array(
      v_fmt, boost::assign::list_of(_vertex_buffer[context.id]));
  _index_buffer[context.id] =
      in_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                               scm::gl::USAGE_STATIC_DRAW,
                               _data->index_data.data_size(),
                               _data->index_data.get());
}

void NURBSRessource::initialize_transform_feedback(RenderContext const& context) const {
  auto in_device = context.render_device;

  int stride =
      sizeof(scm::math::vec3f) + sizeof(unsigned) + sizeof(scm::math::vec2f);

  scm::gl::vertex_format v_fmt = scm::gl::vertex_format(
      0,
      0,
      scm::gl::TYPE_VEC3F,
      stride);  // Vertex Position (xf, yf, zf), v_size = a chunk you will pick
                // each time from buffer
  v_fmt(0, 1, scm::gl::TYPE_UINT, stride);   // Surface Index (indexf)
  v_fmt(0, 2, scm::gl::TYPE_VEC2F, stride);  // uv

  _transform_feedback_vbo[context.id] =
      in_device->create_buffer(scm::gl::BIND_TRANSFORM_FEEDBACK_BUFFER,
                               scm::gl::USAGE_DYNAMIC_COPY,
                               _max_transform_feedback_buffer_size);
  _transform_feedback[context.id] = in_device->create_transform_feedback(
      scm::gl::stream_output_setup(_transform_feedback_vbo[context.id]));
  _transform_feedback_vao[context.id] = in_device->create_vertex_array(
      v_fmt, boost::assign::list_of(_transform_feedback_vbo[context.id]));
}

////////////////////////////////////////////////////////////////////////////////
/*virtual*/ GeometryUberShader* NURBSRessource::get_ubershader() const {
  return Singleton<NURBSUberShader>::instance();
}

}  //namespace scm
