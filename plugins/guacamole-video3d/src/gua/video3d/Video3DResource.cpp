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
#include <gua/video3d/Video3DResource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/video3d/Video3DRenderer.hpp>
#include <gua/video3d/Video3DLoader.hpp>
#include <gua/video3d/video3d_geometry/DXTCompressor.h>
#include <gua/video3d/video3d_geometry/NetKinectArray.hpp>
#include <gua/utils/Logger.hpp>

#include <scm/gl_core/render_device/opengl/gl_core.h>

#include <boost/filesystem.hpp>
#include <boost/asio/ip/host_name.hpp>
#include <boost/algorithm/string.hpp>

// external headers
#include <iostream>
#include <fstream>
#include <gua/utils/string_utils.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

Video3DResource::Video3DResource(std::string const& video3d, unsigned flags)
    : ks_filename_(video3d), calib_files_(), server_endpoint_(), depth_size_(0), depth_size_byte_(0), color_size_(0), width_depthimage_(0), height_depthimage_(0), width_colorimage_(0),
      height_colorimage_(0), overwrite_normal_(false), o_normal_(), is_pickable_(flags & Video3DLoader::MAKE_PICKABLE)
{
    init();
}

////////////////////////////////////////////////////////////////////////////////
void Video3DResource::init()
{
    // approximately local space - can be overwritten from .ks file
    bounding_box_ = math::BoundingBox<math::vec3>(math::vec3(-1.5, -0.1, -1.0), math::vec3(1.5, 2.2, 1.5));

    std::fstream istr;
    istr.open(ks_filename_.c_str(), std::ios::in);

    boost::filesystem::path ks_filepath(ks_filename_);
    boost::filesystem::path ks_dir = ks_filepath.parent_path();

    if(istr.good())
    {
        std::string token;

        while(istr >> token)
        {
            if(token == "serverport")
            {
                istr >> server_endpoint_;
            }
            else if(token == "kinect")
            {
                istr >> token;
                std::string cf_absolute_path = ks_dir.string() + "/" + token;
// detect absolute paths on win/unix
#ifdef _WIN32
                if(token[1] == ':')
                {
                    cf_absolute_path = token;
                }
#else
                if(token[0] == '/')
                {
                    cf_absolute_path = token;
                }
#endif

                auto calib_file_ptr = std::make_shared<KinectCalibrationFile>(cf_absolute_path.c_str());

                calib_file_ptr->parse();
                calib_file_ptr->updateMatrices();

                calib_files_.push_back(calib_file_ptr);
            }
            else if(token == "bbx")
            {
                float x_min, y_min, z_min, x_max, y_max, z_max;
                istr >> x_min >> y_min >> z_min >> x_max >> y_max >> z_max;
                bounding_box_ = math::BoundingBox<math::vec3>(math::vec3(x_min, y_min, z_min), math::vec3(x_max, y_max, z_max));
            }
            else if(token == "normal")
            {
                float x, y, z;
                istr >> x >> y >> z;
                overwrite_normal_ = true;
                o_normal_ = scm::math::vec3f(x, y, z);
            }
        }

        for(auto const& calib_file : calib_files_)
        {
            const unsigned pixelcountc = calib_file->getWidthC() * calib_file->getHeightC();

            depth_size_ = calib_file->getWidth() * calib_file->getHeight(); //== pixelcount
            color_size_ = pixelcountc * 3 * sizeof(unsigned char);
            depth_size_byte_ = depth_size_ * sizeof(float);

            if(calib_file->isCompressedRGB())
            {
                mvt::DXTCompressor dxt;
                dxt.init(calib_file->getWidthC(), calib_file->getHeightC(), FORMAT_DXT1);
                color_size_ = dxt.getStorageSize();
            }
        }

        assert(calib_files_.size() > 0);

        width_depthimage_ = calib_files_[0]->getWidth();
        height_depthimage_ = calib_files_[0]->getHeight();

        width_colorimage_ = calib_files_[0]->getWidthC();
        height_colorimage_ = calib_files_[0]->getHeightC();

        istr.close();
    }
    else
    {
        throw std::runtime_error("Couldn't open calib file");
    }
}

////////////////////////////////////////////////////////////////////////////////
KinectCalibrationFile const& Video3DResource::calibration_file(unsigned i) const
{
    assert(i < calib_files_.size());
    return *calib_files_[i];
}

} // namespace gua
