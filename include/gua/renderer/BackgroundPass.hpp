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

#ifndef GUA_BACKGROUND_PASS_HPP
#define GUA_BACKGROUND_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>

#include <memory>

namespace gua
{
class Pipeline;

class GUA_DLL BackgroundPassDescription : public PipelinePassDescription
{
  public:
    enum BackgroundMode
    {
        COLOR = 0,
        SKYMAP_TEXTURE = 1,
        QUAD_TEXTURE = 2,
    };

    BackgroundPassDescription();

    BackgroundPassDescription& color(utils::Color3f const& color);
    utils::Color3f color() const;

    BackgroundPassDescription& texture(std::string const& texture);
    std::string texture() const;

    BackgroundPassDescription& mode(BackgroundMode const& mode);
    BackgroundMode mode() const;

    BackgroundPassDescription& enable_fog(bool enable_fog);
    bool enable_fog() const;

    BackgroundPassDescription& fog_start(float fog_start);
    float fog_start() const;

    BackgroundPassDescription& fog_end(float fog_end);
    float fog_end() const;

    std::shared_ptr<PipelinePassDescription> make_copy() const override;
    friend class Pipeline;

  protected:
    PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;
};

} // namespace gua

#endif // GUA_BACKGROUND_PASS_HPP
