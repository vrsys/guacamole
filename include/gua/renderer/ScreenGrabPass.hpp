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

#ifndef GUA_SCREENGRAB_PASS_HPP
#define GUA_SCREENGRAB_PASS_HPP

#include <gua/config.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/PipelinePass.hpp>

#include <memory>

namespace gua
{
class Pipeline;

class GUA_DLL ScreenGrabPassDescription : public PipelinePassDescription
{
  public:
    ScreenGrabPassDescription();
    std::shared_ptr<PipelinePassDescription> make_copy() const override;
    friend class Pipeline;

    void set_output_prefix(const std::string& output_prefix);
    void set_grab_next(bool grab_next);

  protected:
    PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;

  private:
    std::string output_prefix_;
    bool grab_next_;
};



class GUA_DLL ScreenGrabJPEGSaver : public Singleton<ScreenGrabJPEGSaver>
{
  public:

    ~ScreenGrabJPEGSaver();

    void save(std::string const& output_prefix, scm::math::vec2ui const& dims, std::vector<float>& rgb_32f);
    friend class Singleton<ScreenGrabJPEGSaver>;

  private:
    ScreenGrabJPEGSaver();
    bool write_JPEG_file(const char* filename, int quality) const;
    void save_();

    std::mutex save_lock_;
    std::atomic_bool should_quit_;
    std::atomic_bool should_save_;
    std::condition_variable save_cv_;
    std::thread worker_;

    std::string output_prefix_;
    scm::math::vec2ui dims_;
    std::vector<float> rgb_32f_;
    std::vector<unsigned char> rgb_8_;
};


} // namespace gua

#endif // GUA_SCREENGRAB_PASS_HPP
