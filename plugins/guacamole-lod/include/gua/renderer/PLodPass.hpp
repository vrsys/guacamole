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
#ifndef GUA_P_LOD_PASS_HPP
#define GUA_P_LOD_PASS_HPP

// guacamole headers
#include <gua/renderer/Lod.hpp>
#include <gua/renderer/PipelinePass.hpp>

namespace gua
{
class GUA_LOD_DLL PLodPassDescription : public PipelinePassDescription
{
  public: // typedefs, enums
    enum SurfelRenderMode
    {
        LQ_ONE_PASS = 0,
        HQ_TWO_PASS = 1
    };

    friend class Pipeline;

  public:
    PLodPassDescription(SurfelRenderMode const mode = SurfelRenderMode::HQ_TWO_PASS);
    PLodPassDescription& mode(SurfelRenderMode const mode);
    SurfelRenderMode mode() const;

    std::shared_ptr<PipelinePassDescription> make_copy() const override;
    PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;

  private:
    SurfelRenderMode surfel_render_mode_;
};

} // namespace gua

#endif // GUA_P_LOD_PASS_HPP
