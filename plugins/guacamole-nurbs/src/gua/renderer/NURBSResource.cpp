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
#include <gua/renderer/NURBSResource.hpp>

#include <gua/utils/Singleton.hpp>
#include <gua/node/NURBSNode.hpp>

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
NURBSResource::NURBSResource(std::shared_ptr<gpucast::beziersurfaceobject> const& object,
             scm::gl::fill_mode in_fill_mode)
    : _data(std::make_shared<NURBSData>(object)),
      _fill_mode(in_fill_mode),
      _max_pre_tesselation(1.0)
{
  bounding_box_ = math::BoundingBox<math::vec3>(
      math::vec3(
          object->bbox().min[0], object->bbox().min[1], object->bbox().min[2]),
      math::vec3(
          object->bbox().max[0], object->bbox().max[1], object->bbox().max[2]));
}


////////////////////////////////////////////////////////////////////////////////
/* virtual */
NURBSResource::~NURBSResource() 
{
  for (auto it  : _surface_tesselation_data.parametric_texture_buffer) it.reset();
  for (auto it :  _surface_tesselation_data.attribute_texture_buffer) it.reset();  
  for (auto it :  _surface_tesselation_data.vertex_buffer) it.reset();  
  for (auto it :  _surface_tesselation_data.vertex_array) it.reset();  
  for (auto it :  _surface_tesselation_data.domain_texture_buffer) it.reset();  

  for (auto it : _surface_raycasting_data.vertex_array) it.reset();
  for (auto it : _surface_raycasting_data.vertex_attrib0) it.reset();
  for (auto it : _surface_raycasting_data.vertex_attrib1) it.reset();
  for (auto it : _surface_raycasting_data.vertex_attrib2) it.reset();
  for (auto it : _surface_raycasting_data.vertex_attrib3) it.reset();
  for (auto it : _surface_raycasting_data.index_buffer) it.reset();
  for (auto it : _surface_raycasting_data.controlpoints) it.reset();

  for (auto it : _contour_trimming_data.partition_texture_buffer) it.reset();
  for (auto it : _contour_trimming_data.contourlist_texture_buffer) it.reset();
  for (auto it : _contour_trimming_data.curvelist_texture_buffer) it.reset();
  for (auto it : _contour_trimming_data.curvedata_texture_buffer) it.reset();
  for (auto it : _contour_trimming_data.pointdata_texture_buffer) it.reset();
}


////////////////////////////////////////////////////////////////////////////////
void NURBSResource::predraw(RenderContext const& context) const
{
  // upload to GPU if neccessary
  if (_surface_tesselation_data.vertex_array.size() <= context.id || _surface_tesselation_data.vertex_array[context.id] == nullptr) {
    upload_to(context);
  }

  auto in_context = context.render_context;

  scm::gl::context_vertex_input_guard cvg(in_context);
  scm::gl::context_state_objects_guard csg(in_context);
  scm::gl::context_image_units_guard cig(in_context);
  scm::gl::context_texture_units_guard ctg(in_context);

  context.render_context->set_rasterizer_state(_rstate_no_cull[context.id], 1.0f);

  auto tfb = Singleton<TransformFeedbackBuffer>::instance();

  //Transform Feedback Stage Begins
  in_context->begin_transform_feedback(tfb->_transform_feedback[context.id],
    scm::gl::PRIMITIVE_POINTS);
  {
    in_context->bind_vertex_array(_surface_tesselation_data.vertex_array[context.id]);
    in_context->bind_index_buffer(
      _surface_tesselation_data.index_buffer[context.id],
      scm::gl::PRIMITIVE_PATCH_LIST_4_CONTROL_POINTS,
      scm::gl::TYPE_UINT);

    in_context->bind_texture(
      _surface_tesselation_data.parametric_texture_buffer[context.id], _sstate[context.id], 5);
    in_context->bind_texture(
      _surface_tesselation_data.attribute_texture_buffer[context.id], _sstate[context.id], 6);
    in_context->bind_texture(
      _contour_trimming_data.partition_texture_buffer[context.id], _sstate[context.id], 7);
    in_context->bind_texture(
      _contour_trimming_data.contourlist_texture_buffer[context.id], _sstate[context.id], 8);
    in_context->bind_texture(
      _contour_trimming_data.curvelist_texture_buffer[context.id], _sstate[context.id], 9);
    in_context->bind_texture(
      _contour_trimming_data.curvedata_texture_buffer[context.id], _sstate[context.id], 10);
    in_context->bind_texture(
      _contour_trimming_data.pointdata_texture_buffer[context.id], _sstate[context.id], 11);

    in_context->current_program()->uniform_sampler("parameter_texture", 5);
    in_context->current_program()->uniform_sampler("attribute_texture", 6);
    in_context->current_program()->uniform_sampler("trim_partition", 7);
    in_context->current_program()->uniform_sampler("trim_contourlist", 8);
    in_context->current_program()->uniform_sampler("trim_curvelist", 9);
    in_context->current_program()->uniform_sampler("trim_curvedata", 10);
    in_context->current_program()->uniform_sampler("trim_pointdata", 11);

    in_context->current_program()->uniform("max_pre_tesselation", _max_pre_tesselation);


    in_context->apply();

    in_context->draw_elements(_data->tess_index_data.size());
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
    sizeof(vertex)* data.size(),
    (void*)(&data[0]));

  for (auto k : data) {
    std::cout << k.pos << " " << k.index << " " << k.uv << " " << k.normal
      << std::endl;
  }
  std::cout << "===================" << std::endl;
#endif
}


////////////////////////////////////////////////////////////////////////////////
void NURBSResource::draw(RenderContext const& context, bool raycasting) const
{
  // upload to GPU if neccessary: todo: check if this is sufficient for thread-safety
  if (_surface_tesselation_data.vertex_array.size() <= context.id || _surface_tesselation_data.vertex_array[context.id] == nullptr) {
    upload_to(context);
  }

  auto in_context = context.render_context;

  scm::gl::context_vertex_input_guard cvg1(in_context);
  scm::gl::context_state_objects_guard csg1(in_context);
  scm::gl::context_image_units_guard cig1(in_context);
  scm::gl::context_texture_units_guard ctg1(in_context);

  context.render_context->set_rasterizer_state(_rstate_no_cull[context.id], 1.0f);

  if (raycasting)
  {
    in_context->bind_vertex_array(_surface_raycasting_data.vertex_array[context.id]);

    in_context->bind_index_buffer(_surface_raycasting_data.index_buffer[context.id],
                                  scm::gl::PRIMITIVE_TRIANGLE_LIST,
                                  scm::gl::TYPE_UINT, 0);

    in_context->bind_texture(
      _surface_raycasting_data.controlpoints[context.id], _sstate[context.id], 5);
    in_context->bind_texture(
      _contour_trimming_data.partition_texture_buffer[context.id], _sstate[context.id], 7);
    in_context->bind_texture(
      _contour_trimming_data.contourlist_texture_buffer[context.id], _sstate[context.id], 8);
    in_context->bind_texture(
      _contour_trimming_data.curvelist_texture_buffer[context.id], _sstate[context.id], 9);
    in_context->bind_texture(
      _contour_trimming_data.curvedata_texture_buffer[context.id], _sstate[context.id], 10);
    in_context->bind_texture(
      _contour_trimming_data.pointdata_texture_buffer[context.id], _sstate[context.id], 11);

    in_context->current_program()->uniform_sampler("vertexdata", 5);
    in_context->current_program()->uniform_sampler("trim_partition", 7);
    in_context->current_program()->uniform_sampler("trim_contourlist", 8);
    in_context->current_program()->uniform_sampler("trim_curvelist", 9);
    in_context->current_program()->uniform_sampler("trim_curvedata", 10);
    in_context->current_program()->uniform_sampler("trim_pointdata", 11);

    in_context->apply();

    in_context->draw_elements(_data->object->_indices.size());
    //in_context->draw_elements(3);

  } else { // adaptive tesselation
    auto tfb = Singleton<TransformFeedbackBuffer>::instance();
    in_context->bind_vertex_array(tfb->_transform_feedback_vao[context.id]);

    in_context->bind_texture(
      _surface_tesselation_data.parametric_texture_buffer[context.id], _sstate[context.id], 5);
    in_context->bind_texture(
      _surface_tesselation_data.attribute_texture_buffer[context.id], _sstate[context.id], 6);
    in_context->bind_texture(
      _contour_trimming_data.partition_texture_buffer[context.id], _sstate[context.id], 7);
    in_context->bind_texture(
      _contour_trimming_data.contourlist_texture_buffer[context.id], _sstate[context.id], 8);
    in_context->bind_texture(
      _contour_trimming_data.curvelist_texture_buffer[context.id], _sstate[context.id], 9);
    in_context->bind_texture(
      _contour_trimming_data.curvedata_texture_buffer[context.id], _sstate[context.id], 10);
    in_context->bind_texture(
      _contour_trimming_data.pointdata_texture_buffer[context.id], _sstate[context.id], 11);

    in_context->current_program()->uniform_sampler("parameter_texture", 5);
    in_context->current_program()->uniform_sampler("attribute_texture", 6);
    in_context->current_program()->uniform_sampler("trim_partition", 7);
    in_context->current_program()->uniform_sampler("trim_contourlist", 8);
    in_context->current_program()->uniform_sampler("trim_curvelist", 9);
    in_context->current_program()->uniform_sampler("trim_curvedata", 10);
    in_context->current_program()->uniform_sampler("trim_pointdata", 11);

    in_context->current_program()->uniform("max_final_tesselation", _max_final_tesselation);

    in_context->apply();

    in_context->draw_transform_feedback(
      scm::gl::PRIMITIVE_PATCH_LIST_4_CONTROL_POINTS,
      tfb->_transform_feedback[context.id]);
  }
}


////////////////////////////////////////////////////////////////////////////////
void NURBSResource::upload_to(RenderContext const& context) const 
{
  using namespace scm::gl;

  std::unique_lock<std::mutex> lock(upload_mutex_);

  initialize_ressources(context);

  initialize_states(context);

  initialize_texture_buffers(context);

  initialize_vertex_data(context);

  initialize_transform_feedback(context);
}


////////////////////////////////////////////////////////////////////////////////
void NURBSResource::initialize_ressources(RenderContext const& context) const
{
  if (_surface_tesselation_data.vertex_array.size() <= context.id) {
    _surface_tesselation_data.vertex_array.resize(context.id + 1);
    _surface_tesselation_data.parametric_texture_buffer.resize(context.id + 1);
    _surface_tesselation_data.attribute_texture_buffer.resize(context.id + 1);
    _surface_tesselation_data.domain_texture_buffer.resize(context.id + 1);
    _surface_raycasting_data.vertex_array.resize(context.id + 1);
    _surface_raycasting_data.vertex_attrib0.resize(context.id + 1);
    _surface_raycasting_data.vertex_attrib1.resize(context.id + 1);
    _surface_raycasting_data.vertex_attrib2.resize(context.id + 1);
    _surface_raycasting_data.vertex_attrib3.resize(context.id + 1);
    _surface_raycasting_data.index_buffer.resize(context.id + 1);
    _surface_raycasting_data.controlpoints.resize(context.id + 1);
    _contour_trimming_data.partition_texture_buffer.resize(context.id + 1);
    _contour_trimming_data.contourlist_texture_buffer.resize(context.id + 1);
    _contour_trimming_data.curvelist_texture_buffer.resize(context.id + 1);
    _contour_trimming_data.curvedata_texture_buffer.resize(context.id + 1);
    _contour_trimming_data.pointdata_texture_buffer.resize(context.id + 1);
    _surface_tesselation_data.vertex_buffer.resize(context.id + 1);
    _surface_tesselation_data.index_buffer.resize(context.id + 1);
    _sstate.resize(context.id + 1);
    _rstate_no_cull.resize(context.id + 1);
    _rstate_ms_point.resize(context.id + 1);
    _bstate_no_blend.resize(context.id + 1);
  }
}

////////////////////////////////////////////////////////////////////////////////
void NURBSResource::initialize_states(RenderContext const& context) const
{
  auto in_device = context.render_device;

  _sstate[context.id] = in_device->create_sampler_state(
    scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);

  _rstate_no_cull[context.id] = in_device->create_rasterizer_state(
    _fill_mode, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false);

  _rstate_ms_point[context.id] = in_device->create_rasterizer_state(
    scm::gl::FILL_POINT, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false);

  _bstate_no_blend[context.id] =
    in_device->create_blend_state(false, 
      scm::gl::FUNC_ONE, scm::gl::FUNC_ZERO, 
      scm::gl::FUNC_ONE, scm::gl::FUNC_ZERO);
}

////////////////////////////////////////////////////////////////////////////////
void NURBSResource::initialize_texture_buffers(RenderContext const& context) const 
{
  validate_texture_buffers();

  auto in_device = context.render_device;
  
  // surface tesselation data
  _surface_tesselation_data.parametric_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(_data->tess_parametric_data),
                                       &_data->tess_parametric_data[0]);

  _surface_tesselation_data.attribute_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(_data->tess_attribute_data),
                                       &_data->tess_attribute_data[0]);

  // raycasting data
  _surface_raycasting_data.controlpoints[context.id] =
                                       in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(_data->object->_controlpoints),
                                       &_data->object->_controlpoints[0]);

  // trimming data
  _contour_trimming_data.partition_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(_data->trim_partition),
                                       &_data->trim_partition[0]);
  ;
  _contour_trimming_data.contourlist_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_RG_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(_data->trim_contourlist),
                                       &_data->trim_contourlist[0]);
  ;
  _contour_trimming_data.curvelist_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(_data->trim_curvelist),
                                       &_data->trim_curvelist[0]);
  ;
  _contour_trimming_data.curvedata_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_R_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(_data->trim_curvedata),
                                       &_data->trim_curvedata[0]);
  ;
  _contour_trimming_data.pointdata_texture_buffer[context.id] =
      in_device->create_texture_buffer(scm::gl::FORMAT_RGB_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(_data->trim_pointdata),
                                       &_data->trim_pointdata[0]);
  ;
}


////////////////////////////////////////////////////////////////////////////////
void NURBSResource::validate_texture_buffers() const
{
  if (_data->tess_parametric_data.empty()) _data->tess_parametric_data.resize(1);
  if (_data->tess_attribute_data.empty())  _data->tess_attribute_data.resize(1);

  if (_data->trim_partition.empty())       _data->trim_partition.resize(1);
  if (_data->trim_contourlist.empty())     _data->trim_contourlist.resize(1);
  if (_data->trim_curvelist.empty())       _data->trim_curvelist.resize(1);
  if (_data->trim_curvedata.empty())       _data->trim_curvedata.resize(1);
  if (_data->trim_pointdata.empty())       _data->trim_pointdata.resize(1);

  if (_data->object->_controlpoints.empty()) _data->object->_controlpoints.resize(1);
}


////////////////////////////////////////////////////////////////////////////////
void NURBSResource::initialize_vertex_data(RenderContext const& context) const 
{
  auto in_device = context.render_device;

  // initialize vertex input for adaptive tesselation
  int stride = sizeof(scm::math::vec3f) + sizeof(unsigned) + sizeof(scm::math::vec4f);

  scm::gl::vertex_format v_fmt = scm::gl::vertex_format(
      0,
      0,
      scm::gl::TYPE_VEC3F,
      stride);  // Vertex Position (xf, yf, zf), v_size = a chunk you will pick
                // each time from buffer
  v_fmt(0, 1, scm::gl::TYPE_UINT, stride);   // Surface Index (indexf)
  v_fmt(0, 2, scm::gl::TYPE_VEC4F, stride);  // uv, tmp, tmp

  _surface_tesselation_data.vertex_buffer[context.id] =
      in_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                               scm::gl::USAGE_STATIC_DRAW,
                               size_in_bytes(_data->tess_patch_data),
                               &_data->tess_patch_data[0]);

  _surface_tesselation_data.vertex_array[context.id] = in_device->create_vertex_array(
    v_fmt, boost::assign::list_of(_surface_tesselation_data.vertex_buffer[context.id]));

  _surface_tesselation_data.index_buffer[context.id] =
      in_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                               scm::gl::USAGE_STATIC_DRAW,
                               size_in_bytes(_data->tess_index_data),
                               &_data->tess_index_data[0]);

  // initialize ray casting setup
  scm::gl::vertex_format v_fmt_rc =
    scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, 0, scm::gl::INT_PURE);
                  v_fmt_rc(1, 1, scm::gl::TYPE_VEC4F, 0, scm::gl::INT_PURE);
                  v_fmt_rc(2, 2, scm::gl::TYPE_VEC4F, 0, scm::gl::INT_PURE);
                  v_fmt_rc(3, 3, scm::gl::TYPE_VEC4F, 0, scm::gl::INT_PURE);

                  //auto a = _data->object->_indices[0];
                  //auto b = _data->object->_indices[1];
                  //auto c = _data->object->_indices[2];
                  //
                  //_data->object->_attrib0[a] = scm::math::vec3f(0.0, 3.0, 0.0);
                  //_data->object->_attrib0[b] = scm::math::vec3f(2.0, 3.0, 0.0);
                  //_data->object->_attrib0[c] = scm::math::vec3f(2.0, 0.0, 0.0);
                  //
                  //_data->object->_attrib1[a] = scm::math::vec4f(0.0, 1.0, 0.0, 0.0);
                  //_data->object->_attrib1[b] = scm::math::vec4f(1.0, 1.0, 0.0, 0.0);
                  //_data->object->_attrib1[c] = scm::math::vec4f(1.0, 0.0, 0.0, 0.0);
                  //
                  //std::cout << _data->object->_attrib1[a] << " , " <<
                  //  _data->object->_attrib1[b] << " , " <<
                  //  _data->object->_attrib1[c] << std::endl;



  _surface_raycasting_data.vertex_attrib0[context.id] =
    in_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(_data->object->_attrib0),
    &_data->object->_attrib0[0]);

  _surface_raycasting_data.vertex_attrib1[context.id] =
    in_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(_data->object->_attrib1),
    &_data->object->_attrib1[0]);

  _surface_raycasting_data.vertex_attrib2[context.id] =
    in_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(_data->object->_attrib2),
    &_data->object->_attrib2[0]);

  _surface_raycasting_data.vertex_attrib3[context.id] =
    in_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(_data->object->_attrib3),
    &_data->object->_attrib3[0]);


  _surface_raycasting_data.vertex_array[context.id] = in_device->create_vertex_array(
    v_fmt_rc, boost::assign::list_of(_surface_raycasting_data.vertex_attrib0[context.id])(
    _surface_raycasting_data.vertex_attrib1[context.id])(
    _surface_raycasting_data.vertex_attrib2[context.id])(
    _surface_raycasting_data.vertex_attrib3[context.id]));

  _surface_raycasting_data.index_buffer[context.id] =
    in_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(_data->object->_indices),
    &_data->object->_indices[0]);

}


////////////////////////////////////////////////////////////////////////////////
void NURBSResource::initialize_transform_feedback(RenderContext const& context) const 
{
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

  auto tfbuffer = Singleton<TransformFeedbackBuffer>::instance();

  if (tfbuffer->_transform_feedback.size() <= context.id)
  {
    tfbuffer->_transform_feedback.resize(context.id + 1);
    tfbuffer->_transform_feedback_vbo.resize(context.id + 1);
    tfbuffer->_transform_feedback_vao.resize(context.id + 1);
  }

  if ( tfbuffer->_transform_feedback[context.id] == 0)
  {
    tfbuffer->_transform_feedback_vbo[context.id] =
      in_device->create_buffer(scm::gl::BIND_TRANSFORM_FEEDBACK_BUFFER,
      scm::gl::USAGE_DYNAMIC_COPY,
      MAX_XFB_BUFFER_SIZE_IN_BYTES);

    tfbuffer->_transform_feedback[context.id] = in_device->create_transform_feedback(
      scm::gl::stream_output_setup(tfbuffer->_transform_feedback_vbo[context.id]));

    tfbuffer->_transform_feedback_vao[context.id] = in_device->create_vertex_array(
      v_fmt, boost::assign::list_of(tfbuffer->_transform_feedback_vbo[context.id]));
  }  
}

}  //namespace scm
