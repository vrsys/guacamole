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
#ifndef GUA_TV_3_VOLUME_PASS_HPP
#define GUA_TV_3_VOLUME_PASS_HPP

// guacamole headers
#include <gua/renderer/TV_3.hpp>
#include <gua/renderer/PipelinePass.hpp>

namespace gua
{
class GUA_TV_3_DLL TV_3VolumePassDescription : public PipelinePassDescription
{
  public: // typedefs, enums
          /*
            enum VolumeRenderMode {
      
              ISOSURFACE = 0,
              MAX_INTENSITY = 1,
              COMPOSITING = 2,
              AVG_INTENSITY = 3
      
            };
          */
    friend class Pipeline;

  public:
    TV_3VolumePassDescription();
    std::shared_ptr<PipelinePassDescription> make_copy() const override;
    PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;

    /*
        TV_3VolumePassDescription& mode(VolumeRenderMode const mode);
        VolumeRenderMode mode() const;
    */
  private:
    // VolumeRenderMode volume_render_mode_;
};

} // namespace gua

#endif // GUA_TV_3_VOLUME_PASS_HPP
