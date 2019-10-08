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

#ifndef GUA_NURBS_RESOURCE_HPP
#define GUA_NURBS_RESOURCE_HPP

// guacamole headers
#include <gua/renderer/NURBS.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/renderer/detail/NURBSData.hpp>

// external headers
#include <string>
#include <vector>
#include <mutex>

#include <scm/core/math.h>

#include <scm/gl_core/gl_core_fwd.h>
#include <scm/gl_core/data_types.h>
#include <scm/gl_core/state_objects.h>

#include <scm/gl_util/primitives/primitives_fwd.h>
#include <scm/gl_util/primitives/geometry.h>

#include <scm/core/platform/platform.h>
#include <scm/core/utilities/platform_warning_disable.h>

namespace gua
{
class NURBSGPURessource;

namespace node
{
class NURBSNode;
};

template <typename T>
std::size_t size_in_bytes(T const& container)
{
    using value_type = typename T::value_type;
    return container.size() * sizeof(value_type);
};

class GUA_NURBS_DLL NURBSResource : public GeometryResource
{
  public: // constants
    struct texture_buffer_binding
    {
        scm::gl::texture_buffer_ptr buffer;
        unsigned texunit;
    };
    struct ssbo_binding
    {
        scm::gl::buffer_ptr buffer;
        unsigned unit;
    };

    static std::size_t const MAX_XFB_BUFFER_SIZE_IN_BYTES = 100000000; // 200MB temporary XFB Buffer

  public: // c'tor / d'tor
    NURBSResource(std::shared_ptr<gpucast::beziersurfaceobject> const& object,
                  unsigned pre_subdivision_u,
                  unsigned pre_subdivision_v,
                  unsigned trim_resolution,
                  scm::gl::fill_mode in_fill_mode = scm::gl::FILL_SOLID
                  // scm::gl::fill_mode in_fill_mode = scm::gl::FILL_WIREFRAME
    );

  public: // methods
    /*virtual*/ void predraw(RenderContext const& context) const;

    /*virtual*/ void draw(RenderContext const& context, bool pretessellation) const;

    void ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits) {}

    void wireframe(bool enable);

  private:
    /////////////////////////////////////////////////////////////////////////////////////////////
    // CPU ressources
    /////////////////////////////////////////////////////////////////////////////////////////////
    std::shared_ptr<NURBSData> _data;

    mutable std::mutex _upload_mutex;

    scm::gl::fill_mode _fill_mode;

    /////////////////////////////////////////////////////////////////////////////////////////////

  private: // helper methods
    void upload_to(RenderContext const& context) const;

    void initialize_states(RenderContext const& context) const;
    void initialize_texture_buffers(RenderContext const& context) const;
    void validate_texture_buffers() const;
    void initialize_vertex_data(RenderContext const& context) const;
    void initialize_transform_feedback(RenderContext const& context) const;
};

} // namespace gua

#endif // GUA_NURBS_RESSOURCE_HPP
