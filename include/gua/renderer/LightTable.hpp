#ifndef GUA_LIGHT_TABLE_HPP
#define GUA_LIGHT_TABLE_HPP

#include <gua/math/math.hpp>

#include <gua/renderer/RenderContext.hpp>
#include <scm/gl_core/buffer_objects/uniform_buffer_adaptor.h>

#include <gua/renderer/Texture3D.hpp>

namespace gua {

class LightTable
{
public:
  struct LightBlock {
    math::vec4  position_and_radius; // xyz - position (or direction for sun light), w - radius
    math::vec4  beam_direction_and_half_angle; //  xyz - direction, w - half angle
    math::vec4  color;
    float       falloff;
    float       brightness;
    float       softness;
    unsigned    type;            // 0 - point, 1 - spot, 2 - sun
    unsigned    diffuse_enable;  // bool
    unsigned    specular_enable; // bool
    unsigned    casts_shadow;    // bool
    //unsigned    pad;

    //math::vec2ui shadow_map;
    //float        shadow_offset;
    //float        light_shadow_map_portion;
    //math::vec2ui light_shadow_map;

    bool operator==(const LightBlock& rhs) const { 
      return    position_and_radius == rhs.position_and_radius
             && beam_direction_and_half_angle == rhs.beam_direction_and_half_angle
             && color == rhs.color
             && falloff == rhs.falloff
             && brightness == rhs.brightness
             && softness == rhs.softness
             && type == rhs.type
             && diffuse_enable == rhs.diffuse_enable
             && specular_enable == rhs.specular_enable
             && casts_shadow == rhs.casts_shadow;
    }

    bool operator!=(const LightBlock& rhs) const {
      return !(operator==(rhs));
    }

  };

  typedef scm::gl::uniform_block_array<LightBlock> uniform_array_type;
  typedef std::vector<LightBlock> array_type;


  virtual ~LightTable() {}
  
  void remove_buffers(RenderContext const& ctx);

  void invalidate(RenderContext const& ctx, 
                  math::vec2ui const& resolution,
                  array_type const& lights);

  std::shared_ptr<Texture3D> const& get_light_bitset() const { return light_bitset_; }
  unsigned get_lights_num() const { return lights_num_; }

  inline const uniform_array_type&   light_uniform_block() const { return uniform_block_; }

private:
  uniform_array_type  uniform_block_;
  unsigned            lights_num_ = 0;

  std::shared_ptr<Texture3D> light_bitset_;
  unsigned light_bitset_words_ = 0;
};

} // namespace gua {

#endif // #ifndef GUA_LIGHT_TABLE_HPP
