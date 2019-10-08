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

#ifndef GUA_TV_3_RESOURCE_HPP
#define GUA_TV_3_RESOURCE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/KDTree.hpp>

// external headers
#include <scm/core/math.h>

#include <scm/gl_core/gl_core_fwd.h>
#include <scm/gl_core/data_types.h>
#include <scm/gl_core/state_objects.h>
#include <scm/gl_core/texture_objects.h>

#include <scm/gl_util/primitives/box_volume.h>
#include <scm/gl_util/primitives/primitives_fwd.h>
#include <scm/gl_util/primitives/geometry.h>

#include <scm/core/platform/platform.h>
#include <scm/core/utilities/platform_warning_disable.h>

#include <chrono>
//#include <pbr/types.h>
//#include <pbr/ren/model_database.h>
//#include <pbr/ren/cut_database.h>
//#include <pbr/ren/cut.h>
//#include <pbr/ren/lod_point_cloud.h>

namespace gua
{
namespace node
{
class TV_3Node;
};

/**
 * Stores a point cloud model with LOD.
 *
 * This class simply a wrapper for accessing models of PBR library
 */
class TV_3Resource : public GeometryResource
{
  public:
    // https://stackoverflow.com/questions/18837857/cant-use-enum-class-as-unordered-map-key
    struct EnumClassHash
    {
        template <typename T>
        std::size_t operator()(T t) const
        {
            return static_cast<std::size_t>(t);
        }
    };

    enum CompressionMode
    {
        UNCOMPRESSED = 0,
        SW_VQ = 1,
        SW_HVQ = 2,

        COMPRESSION_MODE_COUNT
    };

    enum PlaybackMode
    {
        NONE = 0,
        FORWARD = 1,
        BACKWARD = 2,

        PLAYBACK_MODE_COUNT
    };

  public: // c'tor /d'tor
    static void tokenize_volume_name(std::string const& string_to_split, std::map<std::string, uint64_t>& tokens);

    TV_3Resource(std::string const& resource_file_string, bool is_pickable, CompressionMode compression_mode = CompressionMode::UNCOMPRESSED);

    ~TV_3Resource();

  public: // methods
    /*virtual*/ void draw(RenderContext const& context) const;

    /**
     * Draws the point cloud.
     *
     * Draws the point cloud to the given context.
     *
     * \param context  The RenderContext to draw onto.
     */

    // dummy
    void draw(RenderContext const& ctx, scm::gl::vertex_array_ptr const& vertex_array) const;

    virtual void apply_resource_dependent_uniforms(RenderContext const& ctx, std::shared_ptr<ShaderProgram> const& current_program) const;

    virtual void bind_volume_texture(RenderContext const& ctx, scm::gl::sampler_state_ptr const& sampler_state) const;
    math::mat4 const& local_transform() const;

    CompressionMode get_compression_mode() const { return compression_mode_; }

    int64_t const get_num_volume_time_steps() const { return num_time_steps_; }

    // void enable_playback(bool enable_playback) {is_playback_enabled_ = enable_playback;}
    void set_playback_mode(PlaybackMode playback_mode) { playback_mode_ = playback_mode; }
    PlaybackMode get_playback_mode() const { return playback_mode_; }
    // bool get_playback_enabled() const {return is_playback_enabled_;}

    void set_playback_fps(float playback_fps) { playback_fps_ = std::min(10000.0f, std::max(-10000.0f, playback_fps)); }
    float get_playback_fps() const { return playback_fps_; }

    void set_time_cursor_pos(float const time_cursor_pos) { time_cursor_pos_ = std::min(float(num_time_steps_ - 1), time_cursor_pos); }
    float get_time_cursor_pos() const { return time_cursor_pos_; }
    virtual void upload_to(RenderContext const& context) const;

    void ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits);

  protected:
    // std::shared_ptr<*/scm::gl::box_volume_geometry> volume_proxy_;
    bool is_pickable_;
    math::mat4 local_transform_;
    mutable float time_cursor_pos_ = 0.0f;
    std::string resource_file_name_ = "";
    mutable uint64_t frame_counter_ = 0;
    mutable int32_t num_time_steps_ = 1;

    CompressionMode compression_mode_;
    // bool                                         is_playback_enabled_ = false;
    PlaybackMode playback_mode_ = PlaybackMode::NONE;
    float playback_fps_ = 24.0;

    mutable std::chrono::high_resolution_clock::time_point last_time_point_ = std::chrono::high_resolution_clock::now();

    static std::mutex cpu_volume_loading_mutex_;
    static std::map<std::size_t, bool> are_cpu_time_steps_loaded_;
    static std::map<std::size_t, std::map<std::string, uint64_t>> volume_descriptor_tokens_;
    static std::map<std::size_t, std::vector<std::ifstream>> per_resource_file_streams_;
    static std::map<std::size_t, std::vector<std::vector<uint8_t>>> per_resource_cpu_cache_;
};

} // namespace gua

#endif // GUA_PLOD_RESSOURCE_HPP
