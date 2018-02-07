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

#include <gpucast/core/trimdomain_serializer_contour_map_kd.hpp>
#include <gpucast/core/conversion.hpp>
#include <gpucast/gl/util/hullvertexmap.hpp>

namespace gua {

  

////////////////////////////////////////////////////////////////////////////////
NURBSResource::NURBSResource(std::shared_ptr<gpucast::beziersurfaceobject> const& object,
             unsigned pre_subdivision_u, 
             unsigned pre_subdivision_v,
             unsigned trim_resolution,
             scm::gl::fill_mode in_fill_mode)
             : _data(std::make_shared<NURBSData>(object, pre_subdivision_u, pre_subdivision_v, trim_resolution)),
      _fill_mode(in_fill_mode)
{
  bounding_box_ = math::BoundingBox<math::vec3>(
      math::vec3(
          object->bbox().min[0], object->bbox().min[1], object->bbox().min[2]),
      math::vec3(
          object->bbox().max[0], object->bbox().max[1], object->bbox().max[2]));
}


////////////////////////////////////////////////////////////////////////////////
void NURBSResource::predraw(RenderContext const& context, bool cull_face) const
{
  // upload to GPU if neccessary
  if ( !_surface_tesselation_data.vertex_array ) {
    upload_to(context);
  }

  auto in_context = context.render_context;

  scm::gl::context_vertex_input_guard cvg(in_context);
  scm::gl::context_state_objects_guard csg(in_context);
  scm::gl::context_image_units_guard cig(in_context);
  scm::gl::context_texture_units_guard ctg(in_context);

  if (cull_face) {
    context.render_context->set_rasterizer_state(_rstate_cull, 1.0f);
  } else {
    context.render_context->set_rasterizer_state(_rstate_no_cull, 1.0f);
  }

  auto tfb = Singleton<TransformFeedbackBuffer>::instance();

  //Transform Feedback Stage Begins
  in_context->begin_transform_feedback(tfb->_transform_feedback,
    scm::gl::PRIMITIVE_POINTS);
  {
    in_context->bind_vertex_array(_surface_tesselation_data.vertex_array);
    in_context->bind_index_buffer(
      _surface_tesselation_data.index_buffer,
      scm::gl::PRIMITIVE_PATCH_LIST_4_CONTROL_POINTS,
      scm::gl::TYPE_UINT);

    // set ssbo and texturebuffer bindings
    std::map<std::string, texture_buffer_binding> texmap;
    std::map<std::string, ssbo_binding> ssbomap;

    unsigned texunit = 5;

    texmap["parameter_texture"] = { _surface_tesselation_data.parametric_texture_buffer, texunit++ };
    texmap["attribute_texture"] = { _surface_tesselation_data.attribute_texture_buffer, texunit++ };
    texmap["obb_texture"] = { _surface_tesselation_data.obb_texture_buffer, texunit++ };
    //texmap["trim_partition"] = { _contour_trimming_data.partition_texture_buffer, texunit++ };
    //texmap["trim_contourlist"] = { _contour_trimming_data.contourlist_texture_buffer, texunit++ };
    //texmap["trim_curvelist"] = { _contour_trimming_data.curvelist_texture_buffer, texunit++ };
    //texmap["trim_curvedata"] = { _contour_trimming_data.curvedata_texture_buffer, texunit++ };
    //texmap["trim_pointdata"] = { _contour_trimming_data.pointdata_texture_buffer, texunit++ };
    //texmap["trim_preclassification"] = { _contour_trimming_data.preclassification_buffer, texunit++ };

    ssbomap["vertexhullmap"] = { _surface_tesselation_data.hullvertexmap, 1 };
    ssbomap["attribute_buffer"] = { _surface_tesselation_data.attribute_buffer, 2 };

    // apply bindings and corresponding uniforms
    for (auto k : texmap) {
      in_context->bind_texture(k.second.buffer, _sstate, k.second.texunit);
      in_context->current_program()->uniform_sampler(k.first, k.second.texunit);
    }

    for (auto s : ssbomap) {
      in_context->bind_storage_buffer(s.second.buffer, s.second.unit);
      in_context->current_program()->uniform(s.first, s.second.unit);
    }

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

  in_context->get_buffer_sub_data(_transform_feedback_vbo,
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
void NURBSResource::draw(RenderContext const& context, bool raycasting, bool cull_face) const
{
  // upload to GPU if neccessary: todo: check if this is sufficient for thread-safety
  if (!_surface_tesselation_data.vertex_array) {
    upload_to(context);
  }

  auto in_context = context.render_context;

  scm::gl::context_vertex_input_guard cvg1(in_context);
  scm::gl::context_state_objects_guard csg1(in_context);
  scm::gl::context_image_units_guard cig1(in_context);
  scm::gl::context_texture_units_guard ctg1(in_context);

  if (cull_face) {
    context.render_context->set_rasterizer_state(_rstate_cull, 1.0f);
  } else {
    context.render_context->set_rasterizer_state(_rstate_no_cull, 1.0f);
  }

  if (raycasting)
  {
    // bind VAO
    in_context->bind_vertex_array(_surface_raycasting_data.vertex_array);

    in_context->bind_index_buffer(_surface_raycasting_data.index_buffer,
                                  scm::gl::PRIMITIVE_TRIANGLE_LIST,
                                  scm::gl::TYPE_UINT, 0);

    // create texture mapping to uniforms
    struct texbuffer_desc {
      scm::gl::texture_buffer_ptr buffer;
      unsigned texunit;
    };
    std::map<std::string, texbuffer_desc> texmap;
    unsigned texunit = 6;

    texmap["vertexdata"] = { _surface_raycasting_data.controlpoints, texunit++ };
    texmap["trim_partition"] = { _contour_trimming_data.partition_texture_buffer, texunit++ };
    texmap["trim_contourlist"] = { _contour_trimming_data.contourlist_texture_buffer, texunit++ };
    texmap["trim_curvelist"] = { _contour_trimming_data.curvelist_texture_buffer, texunit++ };
    texmap["trim_curvedata"] = { _contour_trimming_data.curvedata_texture_buffer, texunit++ };
    texmap["trim_pointdata"] = { _contour_trimming_data.pointdata_texture_buffer, texunit++ };
    texmap["trim_preclassification"] = { _contour_trimming_data.preclassification_buffer, texunit++ };

    for (auto k : texmap) {
      in_context->bind_texture(k.second.buffer, _sstate, k.second.texunit);
      in_context->current_program()->uniform_sampler(k.first, k.second.texunit);
    }

    in_context->apply();

    std::size_t nprimitives = _data->object->serialized_raycasting_data_indices().size();
    in_context->draw_elements(nprimitives);

  } else { // adaptive tesselation
    auto tfb = Singleton<TransformFeedbackBuffer>::instance();
    in_context->bind_vertex_array(tfb->_transform_feedback_vao);

    // set ssbo and texturebuffer bindings
    std::map<std::string, texture_buffer_binding> texmap;
    std::map<std::string, ssbo_binding> ssbomap;

    unsigned texunit = 5;

    texmap["parameter_texture"] = { _surface_tesselation_data.parametric_texture_buffer, texunit++ };
    texmap["attribute_texture"] = { _surface_tesselation_data.attribute_texture_buffer, texunit++ };
    texmap["obb_texture"] = { _surface_tesselation_data.obb_texture_buffer, texunit++ };

    texmap["trim_partition"] = { _contour_trimming_data.partition_texture_buffer, texunit++ };
    texmap["trim_contourlist"] = { _contour_trimming_data.contourlist_texture_buffer, texunit++ };
    texmap["trim_curvelist"] = { _contour_trimming_data.curvelist_texture_buffer, texunit++ };
    texmap["trim_curvedata"] = { _contour_trimming_data.curvedata_texture_buffer, texunit++ };
    texmap["trim_pointdata"] = { _contour_trimming_data.pointdata_texture_buffer, texunit++ };
    texmap["trim_preclassification"] = { _contour_trimming_data.preclassification_buffer, texunit++ };

    ssbomap["vertexhullmap"] = { _surface_tesselation_data.hullvertexmap, 1 };
    ssbomap["attribute_buffer"] = { _surface_tesselation_data.attribute_buffer, 2 };

    // apply bindings and corresponding uniforms
    for (auto k : texmap) {
      in_context->bind_texture(k.second.buffer, _sstate, k.second.texunit);
      in_context->current_program()->uniform_sampler(k.first, k.second.texunit);
    }

    for (auto s : ssbomap) {
      in_context->bind_storage_buffer(s.second.buffer, s.second.unit);
      in_context->current_program()->uniform(s.first, s.second.unit);
    }

    in_context->apply();

    in_context->draw_transform_feedback(
      scm::gl::PRIMITIVE_PATCH_LIST_4_CONTROL_POINTS,
      tfb->_transform_feedback);
  }
}


////////////////////////////////////////////////////////////////////////////////
void NURBSResource::upload_to(RenderContext const& context) const 
{
  using namespace scm::gl;

  std::unique_lock<std::mutex> lock(upload_mutex_);

  initialize_states(context);

  initialize_texture_buffers(context);

  initialize_vertex_data(context);

  initialize_transform_feedback(context);

  context.render_context->apply();
}

////////////////////////////////////////////////////////////////////////////////
void NURBSResource::initialize_states(RenderContext const& context) const
{
  auto in_device = context.render_device;

  _sstate = in_device->create_sampler_state(
    scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);

  _rstate_no_cull = in_device->create_rasterizer_state(
    _fill_mode, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false);

  _rstate_cull = in_device->create_rasterizer_state(
    _fill_mode, scm::gl::CULL_BACK, scm::gl::ORIENT_CCW, false);

  _rstate_ms_point = in_device->create_rasterizer_state(
    scm::gl::FILL_POINT, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false);

  _bstate_no_blend =
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
  _surface_tesselation_data.parametric_texture_buffer =
      in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(_data->tess_parametric_data),
                                       &_data->tess_parametric_data[0]);

  _surface_tesselation_data.obb_texture_buffer =
    in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(_data->object->serialized_tesselation_obbs()),
    &_data->object->serialized_tesselation_obbs()[0]);

  _surface_tesselation_data.attribute_texture_buffer =
    in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(_data->tess_attribute_data),
    &_data->tess_attribute_data[0]);

  // raycasting data
  _surface_raycasting_data.controlpoints =
                                       in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(_data->object->serialized_controlpoints()),
                                       &_data->object->serialized_controlpoints()[0]);

  // trimming data
  auto const& trimdata = _data->object->serialized_trimdata_as_contour_kd();
  _contour_trimming_data.partition_texture_buffer =
      in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(trimdata->partition),
                                       &trimdata->partition[0]);
  ;
  _contour_trimming_data.contourlist_texture_buffer =
    in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(trimdata->contourlist),
                                       &trimdata->contourlist[0]);
  ;
  _contour_trimming_data.curvelist_texture_buffer =
    in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(trimdata->curvelist),
                                       &trimdata->curvelist[0]);
  ;
  _contour_trimming_data.curvedata_texture_buffer =
      in_device->create_texture_buffer(scm::gl::FORMAT_R_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(trimdata->curvedata),
                                       &trimdata->curvedata[0]);
  ;
  _contour_trimming_data.pointdata_texture_buffer =
      in_device->create_texture_buffer(scm::gl::FORMAT_RGB_32F,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(trimdata->pointdata),
                                       &trimdata->pointdata[0]);
  ;

  _contour_trimming_data.preclassification_buffer =
      in_device->create_texture_buffer(scm::gl::FORMAT_R_8UI,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       size_in_bytes(trimdata->preclassification),
                                       &trimdata->preclassification[0]);
  ;
}


////////////////////////////////////////////////////////////////////////////////
void NURBSResource::validate_texture_buffers() const
{
  if (_data->tess_parametric_data.empty()) _data->tess_parametric_data.resize(1);
  if (_data->tess_attribute_data.empty())  _data->tess_attribute_data.resize(1);
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

  _surface_tesselation_data.vertex_buffer =
      in_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                               scm::gl::USAGE_STATIC_DRAW,
                               size_in_bytes(_data->tess_patch_data),
                               &_data->tess_patch_data[0]);

  _surface_tesselation_data.vertex_array = in_device->create_vertex_array(
    v_fmt, boost::assign::list_of(_surface_tesselation_data.vertex_buffer));

  _surface_tesselation_data.index_buffer =
      in_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                               scm::gl::USAGE_STATIC_DRAW,
                               size_in_bytes(_data->tess_index_data),
                               &_data->tess_index_data[0]);

  gpucast::gl::hullvertexmap hvm;
  _surface_tesselation_data.hullvertexmap = in_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(hvm.data),
    &hvm.data[0]);

  _surface_tesselation_data.attribute_buffer =
    in_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(_data->tess_attribute_data),
    &_data->tess_attribute_data[0]);

  // initialize ray casting setup
  scm::gl::vertex_format v_fmt_rc =
    scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, 0);
  v_fmt_rc(1, 1, scm::gl::TYPE_VEC4F, 0);
  v_fmt_rc(2, 2, scm::gl::TYPE_VEC4F, 0);
  v_fmt_rc(3, 3, scm::gl::TYPE_VEC4F, 0);

  _surface_raycasting_data.vertex_attrib0 =
    in_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(_data->object->serialized_raycasting_data_attrib0()),
    &_data->object->serialized_raycasting_data_attrib0()[0]);

  _surface_raycasting_data.vertex_attrib1=
    in_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(_data->object->serialized_raycasting_data_attrib1()),
    &_data->object->serialized_raycasting_data_attrib1()[0]);

  _surface_raycasting_data.vertex_attrib2 =
    in_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(_data->object->serialized_raycasting_data_attrib2()),
    &_data->object->serialized_raycasting_data_attrib2()[0]);

  _surface_raycasting_data.vertex_attrib3 =
    in_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(_data->object->serialized_raycasting_data_attrib3()),
    &_data->object->serialized_raycasting_data_attrib3()[0]);

  _surface_raycasting_data.vertex_array = in_device->create_vertex_array(
    v_fmt_rc, boost::assign::list_of(_surface_raycasting_data.vertex_attrib0)(
    _surface_raycasting_data.vertex_attrib1)(
    _surface_raycasting_data.vertex_attrib2)(
    _surface_raycasting_data.vertex_attrib3));

  _surface_raycasting_data.index_buffer =
    in_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
    scm::gl::USAGE_STATIC_DRAW,
    size_in_bytes(_data->object->serialized_raycasting_data_indices()),
    &_data->object->serialized_raycasting_data_indices()[0]);

}


////////////////////////////////////////////////////////////////////////////////
void NURBSResource::initialize_transform_feedback(RenderContext const& context) const 
{
  auto in_device = context.render_device;

  int stride = sizeof(scm::math::vec3f) + 
               sizeof(unsigned) + 
               sizeof(scm::math::vec2f) + 
               sizeof(scm::math::vec3f);

  scm::gl::vertex_format v_fmt = scm::gl::vertex_format(
      0,
      0,
      scm::gl::TYPE_VEC3F,
      stride);  // Vertex Position (xf, yf, zf), v_size = a chunk you will pick
                // each time from buffer
  v_fmt(0, 1, scm::gl::TYPE_UINT, stride);   // Surface Index (indexf)
  v_fmt(0, 2, scm::gl::TYPE_VEC2F, stride);  // uv
  v_fmt(0, 3, scm::gl::TYPE_VEC3F, stride);  // remaining tesselation level

  auto tfbuffer = Singleton<TransformFeedbackBuffer>::instance();

  if ( tfbuffer->_transform_feedback == 0)
  {
    tfbuffer->_transform_feedback_vbo =
      in_device->create_buffer(scm::gl::BIND_TRANSFORM_FEEDBACK_BUFFER,
      scm::gl::USAGE_DYNAMIC_COPY,
      MAX_XFB_BUFFER_SIZE_IN_BYTES);

    tfbuffer->_transform_feedback = in_device->create_transform_feedback(
      scm::gl::stream_output_setup(tfbuffer->_transform_feedback_vbo));

    tfbuffer->_transform_feedback_vao = in_device->create_vertex_array(
      v_fmt, boost::assign::list_of(tfbuffer->_transform_feedback_vbo));
  }  
}

}  //namespace scm
