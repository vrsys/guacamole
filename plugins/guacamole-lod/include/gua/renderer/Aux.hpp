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

#ifndef GUA_AUX_HPP
#define GUA_AUX_HPP

// guacamole headers
#include <gua/renderer/Lod.hpp>
#include <gua/scenegraph/PickResult.hpp>
// external headers
#include <set>
#include <unordered_set>
#include <memory>
#include <vector>

namespace lamure {
namespace prov {
  class aux;
  // namespace aux{
  struct feature;
  // }
}
}

// namespace lamure {
// namespace prov {

//   struct feature;

// }
// }




namespace gua {


class GUA_LOD_DLL Aux {
  public:

    struct feature {
      uint32_t camera_id_;
      uint32_t using_count_;
      scm::math::vec2f coords_;
      scm::math::vec2f error_;

      feature()
      : camera_id_(0)
      , using_count_(0)
      , coords_(scm::math::vec2f())
      , error_(scm::math::vec2f())
      {};

      feature(uint32_t camera_id, uint32_t using_count, 
              scm::math::vec2f coords, scm::math::vec2f error)
      : camera_id_(camera_id)
      , using_count_(using_count)
      , coords_(coords)
      , error_(error)
      {};

    };

    struct sparse_point {
      scm::math::vec3f pos_;
      uint8_t r_;
      uint8_t g_;
      uint8_t b_;
      uint8_t a_;
      std::vector<Aux::feature> features_;

      sparse_point()
        : pos_(scm::math::vec3f())
        , r_(0)
        , g_(0)
        , b_(0)
        , a_(0)
        , features_(std::vector<Aux::feature>())
      {};

      sparse_point(scm::math::vec3f pos, uint8_t r, uint8_t g,
                 uint8_t b, uint8_t a, 
                 std::vector<Aux::feature> features)
        : pos_(pos)
        , r_(r)
        , g_(g)
        , b_(b)
        , a_(a)
        , features_(features)
      {};

      std::shared_ptr<Aux::feature> getFeatureById(uint32_t id){
        if (id < features_.size() && id >= 0){
          return std::make_shared<Aux::feature>(features_.at(id));
        } else {
          return std::make_shared<Aux::feature>(Aux::feature());
        }
      }
    };

    struct view {
      uint32_t camera_id_;
      scm::math::vec3f position_;
      scm::math::mat4f transform_; //trans + rot
      float focal_length_;
      float distortion_;
      uint32_t image_width_;
      uint32_t image_height_;
      uint32_t atlas_tile_id_;
      std::string image_file_;

      view() 
      : camera_id_(0)
      , position_(scm::math::vec3f())
      , transform_(scm::math::mat4f()) //trans + rot
      , focal_length_(0.0f)
      , distortion_(0.0f)
      , image_width_(0)
      , image_height_(0)
      , atlas_tile_id_(0)
      , image_file_("")
      {};

      view(uint32_t camera_id, scm::math::vec3f position, scm::math::mat4f transform, 
           float focal_length, float distortion, uint32_t image_width, uint32_t image_height,
           uint32_t atlas_tile_id, std::string image_file) 
      : camera_id_(camera_id)
      , position_(position)
      , transform_(transform)
      , focal_length_(focal_length)
      , distortion_(distortion)
      , image_width_(image_width)
      , image_height_(image_height)
      , atlas_tile_id_(atlas_tile_id)
      , image_file_(image_file)
      {};
    };

 	  struct atlas {
      uint32_t num_atlas_tiles_{0};
      uint32_t atlas_width_{0};
      uint32_t atlas_height_{0};
      uint32_t rotated_{0};

      atlas() 
      :	num_atlas_tiles_(0)
			, atlas_width_(0)
			, atlas_height_(0)
			, rotated_(0)
			{};

      // atlas constructor
      atlas(uint32_t num_atlas_tiles, uint32_t atlas_width,
      			uint32_t atlas_height, uint32_t rotated) 
      :	num_atlas_tiles_(num_atlas_tiles)
			, atlas_width_(atlas_width)
			, atlas_height_(atlas_height)
			, rotated_(rotated)
			{};
	  };

    struct atlas_tile {
      uint32_t atlas_tile_id_{0};
      uint32_t x_{0};
      uint32_t y_{0};
      uint32_t width_{0};
      uint32_t height_{0};

      atlas_tile()
      : atlas_tile_id_(0) 
      , x_(0)
      , y_(0)
      , width_(0)
      , height_(0)
      {};

      atlas_tile(uint32_t atlas_tile_id, uint32_t x, uint32_t y,
                 uint32_t width, uint32_t height)
      : atlas_tile_id_(atlas_tile_id)
      , x_(x)
      , y_(y)
      , width_(width)
      , height_(height)
      {};
    };


  Aux();

public:

  void load_aux_file(std::string const& filename);

  void test_wrapping() const;

  const std::string   get_filename() const;   //done

  const uint32_t      get_num_views() const; // done 
  const uint64_t      get_num_sparse_points() const; // done
  const uint32_t      get_num_atlas_tiles() const; //done

  std::shared_ptr<view> get_view(uint32_t id) const;
  std::shared_ptr<atlas_tile> get_atlas_tile(uint32_t id) const; // done
  std::shared_ptr<sparse_point> get_sparse_point(uint64_t id) const;  // difficult 
  std::shared_ptr<atlas> get_atlas() const; // done 


  // Aux::atlas get_atlas() const;
  // const Aux::atlas get_atlas() const;
  // const std::shared_ptr< Aux::atlas> get_atlas() const;

private: // methods

private: // member
  std::shared_ptr<lamure::prov::aux> _aux;

};

}

#endif  // GUA_AUX_HPP
