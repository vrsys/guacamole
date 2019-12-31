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

#ifndef GUA_VIS_SETTINGS
#define GUA_VIS_SETTINGS

#include <scm/core/math.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace gua
{

struct vis_settings {
  int32_t width_ {1920};
  int32_t height_ {1080};
  int32_t frame_div_ {1};
  int32_t vram_ {4096};
  int32_t ram_ {16096};
  int32_t upload_ {32};
  bool provenance_ {0};
  bool create_aux_resources_ {1};
  float near_plane_ {1.0f};
  float far_plane_ {5000.0f};
  float fov_ {30.0f};
  bool splatting_ {1};
  bool gamma_correction_ {0};
  int32_t gui_ {1};
  int32_t travel_ {2};
  float travel_speed_ {100.5f};
  int32_t max_brush_size_{4096};
  bool lod_update_ {1};
  bool use_pvs_ {1};
  bool pvs_culling_ {0};
  float lod_point_scale_ {1.0f};
  float aux_point_size_ {1.0f};
  float aux_point_distance_ {0.5f};
  float aux_point_scale_ {1.0f};
  float aux_focal_length_ {1.0f};
  int32_t vis_ {0};
  int32_t show_normals_ {0};
  bool show_accuracy_ {0};
  bool show_radius_deviation_ {0};
  bool show_output_sensitivity_ {0};
  bool show_sparse_ {0};
  bool show_views_ {0};
  bool show_photos_ {0};
  bool show_octrees_ {0};
  bool show_bvhs_ {0};
  bool show_pvs_ {0};
  int32_t channel_ {0};
  int32_t fem_result_ {0};
  int32_t fem_vis_mode_ {0};
  float fem_deform_factor_ {1.0};
  std::string fem_value_mapping_file_;

  bool use_material_color_ {0};
  scm::math::vec3f material_diffuse_ {0.6f, 0.6f, 0.6f};
  scm::math::vec4f material_specular_ {0.4f, 0.4f, 0.4f, 1000.0f};
  scm::math::vec3f ambient_light_color_ {0.1f, 0.1f, 0.1f};
  scm::math::vec4f point_light_color_ {1.0f, 1.0f, 1.0f, 1.2f};
  scm::math::mat4f fem_to_pcl_transform_ {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  bool heatmap_ {0};
  float heatmap_min_ {0.0f};
  float heatmap_max_ {1.0f};

  scm::math::vec3f heatmap_color_min_ {68.0f/255.0f, 0.0f, 84.0f/255.0f};
  scm::math::vec3f heatmap_color_max_ {251.f/255.f, 231.f/255.f, 35.f/255.f};
  std::string atlas_file_ {""};
  std::string json_ {""};
  std::string pvs_ {""};
  std::string background_image_ {""};
  int32_t use_view_tf_ {0};
  scm::math::mat4d view_tf_ {scm::math::mat4d::identity()};
  std::vector<std::string> models_;
  std::map<uint32_t, scm::math::mat4d> transforms_;
  std::map<uint32_t, int> min_lod_depths_;
  std::string selection_ {""};
  float max_radius_ {std::numeric_limits<float>::max()};
  int color_rendering_mode {0};
};


void load_vis_settings(std::string const& vis_file_name, vis_settings& settings);

}

#endif //GUA_VIS_SETTINGS
