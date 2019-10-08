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
#ifndef GUA_SPOINTS_RESOURCE_HPP
#define GUA_SPOINTS_RESOURCE_HPP

// guacamole headers
#include <gua/spoints/platform.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/renderer/Texture2D.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/spoints/spoints_geometry/NetKinectArray.hpp>
// external headers
#include <scm/gl_core.h>

#include <mutex>
#include <thread>

#include <vector>
#include <string>

namespace spoints
{
class NetKinectArray;
}

namespace gua
{
struct RenderContext;

/**
 * Stores geometry data.
 *
 * A mesh can be loaded from an Assimp mesh and the draw onto multiple
 * contexts.
 * Do not use this class directly, it is just used by the Geometry class to
 * store the individual meshes of a file.
 */
class GUA_SPOINTS_DLL SPointsResource : public GeometryResource
{
  public:
    struct SPointsData
    {
        SPointsData() = default;
        SPointsData(RenderContext const& ctx, SPointsResource const& spoints);
        // gl resources
        scm::gl::rasterizer_state_ptr rstate_solid_ = nullptr;
        // cpu resources
        std::shared_ptr<spoints::NetKinectArray> nka_ = nullptr;
        // unsigned frame_counter_ = 0;
    };

    /**
     * constructor.
     *
     * Creates a new SPoints from a given spoints string.
     * \param spoints      Holds information about kinect streams.
     */
    SPointsResource(std::string const& spoints_resource_filename, unsigned flags);

    /**
     * destructor.
     */
    ~SPointsResource() {}

    bool has_calibration(RenderContext const& ctx) const;

    bool is_vertex_data_fully_encoded();

    void draw_textured_triangle_soup(RenderContext const& ctx, std::shared_ptr<gua::ShaderProgram>& shader_program);

    std::string get_socket_string() const;

    spoints::SPointsStats get_latest_spoints_stats() const
    {
        std::lock_guard<std::mutex> lock(m_push_matrix_package_mutex_);

        if(spointsdata_)
        {
            if(spointsdata_->nka_)
            {
                return spointsdata_->nka_->get_latest_spoints_stats();
            }
        }

        return spoints::SPointsStats();
    }

    void push_matrix_package(spoints::camera_matrix_package const& cam_mat_package);

    void update_buffers(RenderContext const& ctx, Pipeline& pipe);

    /**
     * Raytest for SPoints
     *
     * Not implemented yet.
     *
     */
    void ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits) override {}

    /**
     *
     */
    void init();

    std::string server_endpoint() const { return server_endpoint_; }
    std::string feedback_endpoint() const { return feedback_endpoint_; }
    bool is_pickable() const { return is_pickable_; }

  private:
    mutable std::mutex m_push_matrix_package_mutex_;
    std::shared_ptr<SPointsData> spointsdata_;

    std::string spoints_resource_filename_ = "";
    std::string server_endpoint_ = "";
    std::string feedback_endpoint_ = "";

    scm::math::vec3ui inv_xyz_vol_res_ = scm::math::vec3ui(1, 1, 1);
    scm::math::vec3ui uv_vol_res_ = scm::math::vec3ui(1, 1, 1);

    bool is_pickable_;
};

} // namespace gua

#endif // GUA_SPOINTS_RESOURCE_HPP