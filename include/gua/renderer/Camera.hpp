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

#ifndef GUA_CAMERA_HPP
#define GUA_CAMERA_HPP

#include <gua/platform.hpp>
#include <gua/utils/TagRegister.hpp>

// external headers
#include <string>

namespace gua {

/**
 *  This struct describes a user's view on the scene.
 *
 *  It is defined by a screen, a view point a a render mask.
 */

class Camera {

  public:
    enum ProjectionMode {
      PERSPECTIVE,
      ORTHOGRAPHIC
    };

    Camera(std::string const& eye_l =     "unknown_left_eye",
           std::string const& eye_r =     "unknown_right_eye",
           std::string const& screen_l =  "unknown_left_screen",
           std::string const& screen_r =  "unknown_right_screen",
           std::string const& g =         "scene_graph", std::string const& m = "",
           ProjectionMode     p =         PERSPECTIVE)
        : eye_l(eye_l), eye_r(eye_r), screen_l(screen_l), screen_r(screen_r),
          scene_graph(g), render_mask(m), mode(p) {}

    std::string eye_l;
    std::string eye_r;
    std::string screen_l;
    std::string screen_r;
    std::string scene_graph;
    std::string render_mask;
    ProjectionMode mode;

    void add_tag_to_whitelist(std::string const& tag) {
      add_tag(tag, whitelist_);
    }

    void add_tags_to_whitelist(std::vector<std::string> const& tags) {
      for (auto const& tag : tags) {
        add_tag_to_whitelist(tag);
      }
    }

    void remove_tag_from_whitelist(std::string const& tag) {
      remove_tag(tag, whitelist_);
    }

    void remove_tags_from_whitelist(std::vector<std::string> const& tags) {
      for (auto const& tag : tags) {
        remove_tag_from_whitelist(tag);
      }
    }

    std::vector<std::string> const get_whitelist_tags() const {
      if (whitelist_.any()) {
        return gua::utils::TagRegister::instance()->get_tag_strings(whitelist_);
      }

      return std::vector<std::string>();
    }

    std::bitset<GUA_MAX_TAG_COUNT> const& get_whitelist_tag_set() const {
      return whitelist_;
    }



    void add_tag_to_blacklist(std::string const& tag) {
      add_tag(tag, blacklist_);
    }

    void add_tags_to_blacklist(std::vector<std::string> const& tags) {
      for (auto const& tag : tags) {
        add_tag_to_blacklist(tag);
      }
    }

    void remove_tag_from_blacklist(std::string const& tag) {
      remove_tag(tag, blacklist_);
    }

    void remove_tags_from_blacklist(std::vector<std::string> const& tags) {
      for (auto const& tag : tags) {
        remove_tag_from_blacklist(tag);
      }
    }

    std::vector<std::string> const get_blacklist_tags() const {
      if (blacklist_.any()) {
        return gua::utils::TagRegister::instance()->get_tag_strings(blacklist_);
      }

      return std::vector<std::string>();
    }

    std::bitset<GUA_MAX_TAG_COUNT> const& get_blacklist_tag_set() const {
      return blacklist_;
    }


  private:
    std::bitset<GUA_MAX_TAG_COUNT> whitelist_;
    std::bitset<GUA_MAX_TAG_COUNT> blacklist_;

    void add_tag(std::string const& tag, std::bitset<GUA_MAX_TAG_COUNT>& list) {
      auto new_tag(gua::utils::TagRegister::instance()->get_tag(tag));
      if (new_tag.any()) {
        list |= new_tag;
      }
    }

    void remove_tag(std::string const& tag, std::bitset<GUA_MAX_TAG_COUNT>& list) {
      if (list.any()) {
        auto tag_to_remove(gua::utils::TagRegister::instance()->get_tag(tag));
        if (tag_to_remove.any()) {
          list &= tag_to_remove.flip();
        }
      }
    }
};

}

#endif  // GUA_CAMERA_HPP
