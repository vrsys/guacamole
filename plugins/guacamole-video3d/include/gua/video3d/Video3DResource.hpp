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
#ifndef GUA_VIDEO3D_RESOURCE_HPP
#define GUA_VIDEO3D_RESOURCE_HPP

// guacamole headers
#include <gua/video3d/platform.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/video3d/video3d_geometry/KinectCalibrationFile.hpp>
#include <gua/video3d/video3d_geometry/FileBuffer.h>
#include <gua/renderer/Texture2D.hpp>
#include <gua/renderer/ShaderProgram.hpp>

// external headers
#include <scm/gl_core.h>

#include <mutex>
#include <thread>

#include <vector>
#include <string>

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
class GUA_VIDEO3D_DLL Video3DResource : public GeometryResource
{
  public:
    /**
     * constructor.
     *
     * Creates a new Video3D from a given video3d string.
     * \param video3d      Holds information about kinect streams.
     */
    Video3DResource(std::string const& video3d, unsigned flags);

    /**
     * destructor.
     */
    ~Video3DResource() {}

    /**
     * Raytest for Video3D
     *
     * Not implemented yet.
     *
     */
    void ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits) override {}

    /**
     *
     */
    void init();

    /**
     *
     */
    inline unsigned number_of_cameras() const { return unsigned(calib_files_.size()); }
    std::vector<std::shared_ptr<KinectCalibrationFile>> const& calib_files() const { return calib_files_; }

    unsigned color_size() const { return color_size_; }
    unsigned depth_size_byte() const { return depth_size_byte_; }
    std::string server_endpoint() const { return server_endpoint_; }
    unsigned width_depthimage() const { return width_depthimage_; }
    unsigned height_depthimage() const { return height_depthimage_; }
    unsigned width_colorimage() const { return width_colorimage_; }
    unsigned height_colorimage() const { return height_colorimage_; }

    KinectCalibrationFile const& calibration_file(unsigned i) const;

    inline bool do_overwrite_normal() const { return overwrite_normal_; }
    scm::math::vec3f const& get_overwrite_normal() const { return o_normal_; }
    bool is_pickable() const { return is_pickable_; }

  private:
    std::string ks_filename_;
    std::vector<std::shared_ptr<KinectCalibrationFile>> calib_files_;
    std::string server_endpoint_;

    // cpu resources
    unsigned depth_size_;
    unsigned depth_size_byte_;
    unsigned color_size_;

    unsigned width_depthimage_;
    unsigned height_depthimage_;

    unsigned width_colorimage_;
    unsigned height_colorimage_;

    bool overwrite_normal_;
    scm::math::vec3f o_normal_;
    bool is_pickable_;
};

} // namespace gua

#endif // GUA_VIDEO3D_RESOURCE_HPP
