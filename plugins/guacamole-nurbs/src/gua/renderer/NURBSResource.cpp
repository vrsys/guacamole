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
#include <gua/renderer/NURBSGPUResource.hpp>
#include <gua/renderer/NURBSRenderer.hpp>

#include <scm/gl_core/render_device.h>
#include <scm/gl_core/buffer_objects.h>
#include <scm/gl_core/shader_objects.h>
#include <scm/gl_core/texture_objects.h>
#include <scm/gl_core/render_device/opengl/util/assert.h>
#include <scm/gl_core/data_formats.h>
#include <scm/gl_core/constants.h>

#include <boost/assign/list_of.hpp>
#include <boost/log/trivial.hpp>

#include <gpucast/core/trimdomain_serializer_contour_map_kd.hpp>
#include <gpucast/core/conversion.hpp>
#include <gpucast/gl/util/hullvertexmap.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////
NURBSResource::NURBSResource(
    std::shared_ptr<gpucast::beziersurfaceobject> const& object, unsigned pre_subdivision_u, unsigned pre_subdivision_v, unsigned trim_resolution, scm::gl::fill_mode in_fill_mode)
    : _data(std::make_shared<NURBSData>(object, pre_subdivision_u, pre_subdivision_v, trim_resolution)), _fill_mode(in_fill_mode)
{
    bounding_box_ =
        math::BoundingBox<math::vec3>(math::vec3(object->bbox().min[0], object->bbox().min[1], object->bbox().min[2]), math::vec3(object->bbox().max[0], object->bbox().max[1], object->bbox().max[2]));
}

////////////////////////////////////////////////////////////////////////////////
void NURBSResource::predraw(RenderContext const& context) const
{
    // upload to GPU if neccessary
    auto iter = context.plugin_resources.find(uuid());
    if(iter == context.plugin_resources.end())
    {
        upload_to(context);
        iter = context.plugin_resources.find(uuid());
    }

    auto in_context = context.render_context;

    scm::gl::context_vertex_input_guard cvg(in_context);
    scm::gl::context_state_objects_guard csg(in_context);
    scm::gl::context_image_units_guard cig(in_context);
    scm::gl::context_texture_units_guard ctg(in_context);

    auto resource = std::dynamic_pointer_cast<NURBSGPUResource>(iter->second);
    auto xfb_ptr = std::dynamic_pointer_cast<NURBSTransformFeedbackBuffer>(context.plugin_resources[NURBSTransformFeedbackBuffer::GUA_TRANSFORM_FEEDBACK_BUFFER_BASE_CACHE_ID]);
    auto state_ptr = std::dynamic_pointer_cast<NURBSRasterizationState>(context.plugin_resources[NURBSRasterizationState::GUA_RASTERIZATION_BASE_CACHE_ID]);

    // Transform Feedback Stage Begins
    in_context->begin_transform_feedback(xfb_ptr->_transform_feedback, scm::gl::PRIMITIVE_POINTS);
    {
        // set ssbo and texturebuffer bindings
        std::map<std::string, texture_buffer_binding> texmap;
        std::map<std::string, ssbo_binding> ssbomap;

        unsigned texunit = 0;

        texmap["parameter_texture"] = {resource->_surface_tesselation_data.parametric_texture_buffer, texunit++};
        texmap["attribute_texture"] = {resource->_surface_tesselation_data.attribute_texture_buffer, texunit++};
        texmap["obb_texture"] = {resource->_surface_tesselation_data.obb_texture_buffer, texunit++};
        ssbomap["vertexhullmap"] = {resource->_surface_tesselation_data.hullvertexmap, NURBSRenderer::GUA_HULLVERTEXMAP_SSBO_BINDING};
        ssbomap["attribute_buffer"] = {resource->_surface_tesselation_data.attribute_buffer, NURBSRenderer::GUA_ATTRIBUTE_SSBO_BINDING};

        // apply bindings and corresponding uniforms
        for(auto k : texmap)
        {
            in_context->bind_texture(k.second.buffer, state_ptr->_sstate, k.second.texunit);
            in_context->current_program()->uniform_sampler(k.first, k.second.texunit);
        }

        for(auto s : ssbomap)
        {
            in_context->bind_storage_buffer(s.second.buffer, s.second.unit);
            in_context->current_program()->uniform(s.first, s.second.unit);
        }

        in_context->bind_vertex_array(resource->_surface_tesselation_data.vertex_array);
        in_context->bind_index_buffer(resource->_surface_tesselation_data.index_buffer, scm::gl::PRIMITIVE_PATCH_LIST_4_CONTROL_POINTS, scm::gl::TYPE_UINT);

        in_context->apply();

        in_context->draw_elements(_data->tess_index_data.size());
    }

    in_context->end_transform_feedback();
}

////////////////////////////////////////////////////////////////////////////////
void NURBSResource::draw(RenderContext const& context, bool pretessellation) const
{
    // upload to GPU if neccessary
    auto iter = context.plugin_resources.find(uuid());
    assert(iter != context.plugin_resources.end());

    auto in_context = context.render_context;

    scm::gl::context_vertex_input_guard cvg1(in_context);
    scm::gl::context_state_objects_guard csg1(in_context);
    scm::gl::context_image_units_guard cig1(in_context);
    scm::gl::context_texture_units_guard ctg1(in_context);

    auto resource = std::dynamic_pointer_cast<NURBSGPUResource>(iter->second);
    auto xfb_ptr = std::dynamic_pointer_cast<NURBSTransformFeedbackBuffer>(context.plugin_resources[NURBSTransformFeedbackBuffer::GUA_TRANSFORM_FEEDBACK_BUFFER_BASE_CACHE_ID]);
    auto state_ptr = std::dynamic_pointer_cast<NURBSRasterizationState>(context.plugin_resources[NURBSRasterizationState::GUA_RASTERIZATION_BASE_CACHE_ID]);

    in_context->bind_vertex_array(xfb_ptr->_transform_feedback_vao);

    // set ssbo and texturebuffer bindings
    std::map<std::string, texture_buffer_binding> texmap;
    std::map<std::string, ssbo_binding> ssbomap;

    unsigned texunit = 0;

    texmap["parameter_texture"] = {resource->_surface_tesselation_data.parametric_texture_buffer, texunit++};
    texmap["attribute_texture"] = {resource->_surface_tesselation_data.attribute_texture_buffer, texunit++};
    texmap["obb_texture"] = {resource->_surface_tesselation_data.obb_texture_buffer, texunit++};

    texmap["trim_partition"] = {resource->_contour_trimming_data.partition_texture_buffer, texunit++};
    texmap["trim_contourlist"] = {resource->_contour_trimming_data.contourlist_texture_buffer, texunit++};
    texmap["trim_curvelist"] = {resource->_contour_trimming_data.curvelist_texture_buffer, texunit++};
    texmap["trim_curvedata"] = {resource->_contour_trimming_data.curvedata_texture_buffer, texunit++};
    texmap["trim_pointdata"] = {resource->_contour_trimming_data.pointdata_texture_buffer, texunit++};
    texmap["trim_preclassification"] = {resource->_contour_trimming_data.preclassification_buffer, texunit++};

    ssbomap["vertexhullmap"] = {resource->_surface_tesselation_data.hullvertexmap, NURBSRenderer::GUA_HULLVERTEXMAP_SSBO_BINDING};
    ssbomap["attribute_buffer"] = {resource->_surface_tesselation_data.attribute_buffer, NURBSRenderer::GUA_ATTRIBUTE_SSBO_BINDING};

    // apply bindings and corresponding uniforms
    for(auto k : texmap)
    {
        in_context->bind_texture(k.second.buffer, state_ptr->_sstate, k.second.texunit);
        in_context->current_program()->uniform_sampler(k.first, k.second.texunit);
    }

    for(auto s : ssbomap)
    {
        in_context->bind_storage_buffer(s.second.buffer, s.second.unit);
        in_context->current_program()->uniform(s.first, s.second.unit);
    }

    if(_fill_mode == scm::gl::FILL_WIREFRAME)
    {
        in_context->set_rasterizer_state(state_ptr->_wire_no_cull);
    }
    else
    {
        in_context->set_rasterizer_state(state_ptr->_solid_no_cull);
    }

    if(pretessellation)
    {
        in_context->apply();
        in_context->draw_transform_feedback(scm::gl::PRIMITIVE_PATCH_LIST_4_CONTROL_POINTS, xfb_ptr->_transform_feedback);
    }
    else
    {
        in_context->bind_vertex_array(resource->_surface_tesselation_data.vertex_array);
        in_context->bind_index_buffer(resource->_surface_tesselation_data.index_buffer, scm::gl::PRIMITIVE_PATCH_LIST_4_CONTROL_POINTS, scm::gl::TYPE_UINT);
        in_context->apply();
        in_context->draw_elements(_data->tess_index_data.size());
    }
}

////////////////////////////////////////////////////////////////////////////////
void NURBSResource::upload_to(RenderContext const& context) const
{
    // create, if neccessary
    auto resource = std::make_shared<NURBSGPUResource>();
    context.plugin_resources[uuid()] = resource;

    // create state
    initialize_states(context);
    initialize_transform_feedback(context);

    initialize_texture_buffers(context);

    initialize_vertex_data(context);

    context.render_context->apply();
}

////////////////////////////////////////////////////////////////////////////////
void NURBSResource::initialize_states(RenderContext const& context) const
{
    if(context.plugin_resources.count(NURBSRasterizationState::GUA_RASTERIZATION_BASE_CACHE_ID))
        return;

    std::lock_guard<std::mutex> lock(_upload_mutex);

    auto in_device = context.render_device;

    auto state_ptr = std::make_shared<NURBSRasterizationState>();
    context.plugin_resources[NURBSRasterizationState::GUA_RASTERIZATION_BASE_CACHE_ID] = state_ptr;

    state_ptr->_sstate = in_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);

    state_ptr->_wire_no_cull = in_device->create_rasterizer_state(scm::gl::FILL_WIREFRAME, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false);

    state_ptr->_solid_no_cull = in_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false);

    state_ptr->_solid_cull = in_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK, scm::gl::ORIENT_CCW, false);
}

////////////////////////////////////////////////////////////////////////////////
void NURBSResource::initialize_texture_buffers(RenderContext const& context) const
{
    std::lock_guard<std::mutex> lock(_upload_mutex);

    validate_texture_buffers();

    auto in_device = context.render_device;

    assert(context.plugin_resources.find(uuid()) != context.plugin_resources.end());
    auto resource = std::dynamic_pointer_cast<NURBSGPUResource>(context.plugin_resources.find(uuid())->second);

    // surface tesselation data
    resource->_surface_tesselation_data.parametric_texture_buffer =
        in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F, scm::gl::USAGE_STATIC_DRAW, size_in_bytes(_data->tess_parametric_data), &_data->tess_parametric_data[0]);

    resource->_surface_tesselation_data.obb_texture_buffer = in_device->create_texture_buffer(
        scm::gl::FORMAT_RGBA_32F, scm::gl::USAGE_STATIC_DRAW, size_in_bytes(_data->object->serialized_tesselation_obbs()), &_data->object->serialized_tesselation_obbs()[0]);

    resource->_surface_tesselation_data.attribute_texture_buffer =
        in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F, scm::gl::USAGE_STATIC_DRAW, size_in_bytes(_data->tess_attribute_data), &_data->tess_attribute_data[0]);

    // trimming data
    auto const& trimdata = _data->object->serialized_trimdata_as_contour_kd();
    resource->_contour_trimming_data.partition_texture_buffer =
        in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F, scm::gl::USAGE_STATIC_DRAW, size_in_bytes(trimdata->partition), &trimdata->partition[0]);
    ;
    resource->_contour_trimming_data.contourlist_texture_buffer =
        in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F, scm::gl::USAGE_STATIC_DRAW, size_in_bytes(trimdata->contourlist), &trimdata->contourlist[0]);
    ;
    resource->_contour_trimming_data.curvelist_texture_buffer =
        in_device->create_texture_buffer(scm::gl::FORMAT_RGBA_32F, scm::gl::USAGE_STATIC_DRAW, size_in_bytes(trimdata->curvelist), &trimdata->curvelist[0]);
    ;
    resource->_contour_trimming_data.curvedata_texture_buffer =
        in_device->create_texture_buffer(scm::gl::FORMAT_R_32F, scm::gl::USAGE_STATIC_DRAW, size_in_bytes(trimdata->curvedata), &trimdata->curvedata[0]);
    ;
    resource->_contour_trimming_data.pointdata_texture_buffer =
        in_device->create_texture_buffer(scm::gl::FORMAT_RGB_32F, scm::gl::USAGE_STATIC_DRAW, size_in_bytes(trimdata->pointdata), &trimdata->pointdata[0]);
    ;

    resource->_contour_trimming_data.preclassification_buffer =
        in_device->create_texture_buffer(scm::gl::FORMAT_R_8UI, scm::gl::USAGE_STATIC_DRAW, size_in_bytes(trimdata->preclassification), &trimdata->preclassification[0]);
    ;
}

////////////////////////////////////////////////////////////////////////////////
void NURBSResource::validate_texture_buffers() const
{
    if(_data->tess_parametric_data.empty())
        _data->tess_parametric_data.resize(1);
    if(_data->tess_attribute_data.empty())
        _data->tess_attribute_data.resize(1);
}

////////////////////////////////////////////////////////////////////////////////
void NURBSResource::initialize_vertex_data(RenderContext const& context) const
{
    std::lock_guard<std::mutex> lock(_upload_mutex);

    auto in_device = context.render_device;

    assert(context.plugin_resources.find(uuid()) != context.plugin_resources.end());
    auto resource = std::dynamic_pointer_cast<NURBSGPUResource>(context.plugin_resources.find(uuid())->second);

    // initialize vertex input for adaptive tesselation
    int stride = sizeof(scm::math::vec3f) + sizeof(unsigned) + sizeof(scm::math::vec4f);

    scm::gl::vertex_format v_fmt = scm::gl::vertex_format(0,
                                                          0,
                                                          scm::gl::TYPE_VEC3F,
                                                          stride); // Vertex Position (xf, yf, zf), v_size = a chunk you will pick
                                                                   // each time from buffer
    v_fmt(0, 1, scm::gl::TYPE_UINT, stride);                       // Surface Index (indexf)
    v_fmt(0, 2, scm::gl::TYPE_VEC4F, stride);                      // uv, tmp, tmp

    resource->_surface_tesselation_data.vertex_buffer =
        in_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, size_in_bytes(_data->tess_patch_data), &_data->tess_patch_data[0]);

    resource->_surface_tesselation_data.vertex_array = in_device->create_vertex_array(v_fmt, boost::assign::list_of(resource->_surface_tesselation_data.vertex_buffer));

    resource->_surface_tesselation_data.index_buffer =
        in_device->create_buffer(scm::gl::BIND_INDEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, size_in_bytes(_data->tess_index_data), &_data->tess_index_data[0]);

    gpucast::gl::hullvertexmap hvm;
    resource->_surface_tesselation_data.hullvertexmap = in_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_STATIC_DRAW, size_in_bytes(hvm.data), &hvm.data[0]);

    resource->_surface_tesselation_data.attribute_buffer =
        in_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_STATIC_DRAW, size_in_bytes(_data->tess_attribute_data), &_data->tess_attribute_data[0]);
}

////////////////////////////////////////////////////////////////////////////////
void NURBSResource::initialize_transform_feedback(RenderContext const& context) const
{
    if(context.plugin_resources.count(NURBSTransformFeedbackBuffer::GUA_TRANSFORM_FEEDBACK_BUFFER_BASE_CACHE_ID))
        return;

    std::lock_guard<std::mutex> lock(_upload_mutex);

    auto transform_feedback_buffer = std::make_shared<NURBSTransformFeedbackBuffer>();
    context.plugin_resources[NURBSTransformFeedbackBuffer::GUA_TRANSFORM_FEEDBACK_BUFFER_BASE_CACHE_ID] = transform_feedback_buffer;

    auto in_device = context.render_device;

    int stride = sizeof(scm::math::vec3f) + sizeof(unsigned) + sizeof(scm::math::vec2f) + sizeof(scm::math::vec3f);

    scm::gl::vertex_format v_fmt = scm::gl::vertex_format(0,
                                                          0,
                                                          scm::gl::TYPE_VEC3F,
                                                          stride); // Vertex Position (xf, yf, zf), v_size = a chunk you will pick
                                                                   // each time from buffer
    v_fmt(0, 1, scm::gl::TYPE_UINT, stride);                       // Surface Index (indexf)
    v_fmt(0, 2, scm::gl::TYPE_VEC2F, stride);                      // uv
    v_fmt(0, 3, scm::gl::TYPE_VEC3F, stride);                      // remaining tesselation level

    if(transform_feedback_buffer->_transform_feedback == 0)
    {
        transform_feedback_buffer->_transform_feedback_vbo =
            in_device->create_buffer(scm::gl::BIND_TRANSFORM_FEEDBACK_BUFFER, scm::gl::USAGE_DYNAMIC_COPY, NURBSRenderer::GUA_MAX_XFB_BUFFER_SIZE_IN_BYTES);

        transform_feedback_buffer->_transform_feedback = in_device->create_transform_feedback(scm::gl::stream_output_setup(transform_feedback_buffer->_transform_feedback_vbo));

        transform_feedback_buffer->_transform_feedback_vao = in_device->create_vertex_array(v_fmt, boost::assign::list_of(transform_feedback_buffer->_transform_feedback_vbo));
    }
}

////////////////////////////////////////////////////////////////////////////////
void NURBSResource::wireframe(bool enable)
{
    if(enable)
    {
        _fill_mode = scm::gl::FILL_WIREFRAME;
    }
    else
    {
        _fill_mode = scm::gl::FILL_SOLID;
    }
}

} // namespace gua
