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

#ifndef GUA_OCULUSSDK2_WINDOW_HPP
#define GUA_OCULUSSDK2_WINDOW_HPP

#if defined (_MSC_VER)
  #if defined (GUA_OCULUSSDK2_LIBRARY)
    #define GUA_OCULUSSDK2_DLL __declspec( dllexport )
  #else
#define GUA_OCULUSSDK2_DLL __declspec( dllimport )
  #endif
#else
  #define GUA_OCULUSSDK2_DLL
#endif // #if defined(_MSC_VER)

// guacamole headers
#include <gua/renderer/Window.hpp>

//for internal distortion mesh handling
#include <OVR.h>

namespace gua {

struct GUA_DLL OculusSDK2DistortionMesh {

  /* the vertex contains three different texture coordinates
     since it has to correct chromatic abberations for each
     wavelength (~color channel) independently
  */
  struct DistortionVertex {
    scm::math::vec2f ndc_2d_pos;
    scm::math::vec2f tex_r;
    scm::math::vec2f tex_g;
    scm::math::vec2f tex_b;
  };

  OculusSDK2DistortionMesh() : ndc_2d_positions(),
                               tex_coords_r(),
                               tex_coords_g(),
                               tex_coords_b(),
                               indices(),
                               num_vertices(0),
                               num_indices(0),
                               index_buffer_component_offset(0) {
  }

  ~OculusSDK2DistortionMesh() {
  }

  void add_distortion_mesh_component(ovrDistortionMesh const& mesh_component, 
                                     ovrVector2f* UVScaleOffset,
                                     bool isLeftEye) {
    num_vertices  += mesh_component.VertexCount;
    num_indices += mesh_component.IndexCount;

    ndc_2d_positions.reserve(num_vertices);
    tex_coords_r.reserve(num_vertices);
    tex_coords_g.reserve(num_vertices);
    tex_coords_b.reserve(num_vertices);
    indices.reserve(num_indices);

    //populate vertex buffer components
    for( unsigned int vertex_idx = 0; 
         vertex_idx < mesh_component.VertexCount; 
         ++vertex_idx ) {

      //get vertex in oculus sdk format
      ovrDistortionVertex const& distortion_vertex = mesh_component.pVertexData[vertex_idx];

      float corrected_x_pos = 0.0;

      if(isLeftEye) {
        corrected_x_pos = (distortion_vertex.ScreenPosNDC.x + 1.0) * 2.0 - 1.0;
      } else {
        corrected_x_pos = (distortion_vertex.ScreenPosNDC.x - 1.0) * 2.0 + 1.0;        
      }

      scm::math::vec2f vPos( corrected_x_pos ,
                            distortion_vertex.ScreenPosNDC.y);

      std::cout << "VPos: " << vPos.x << " " << vPos.y << "\n";

      scm::math::vec2f vTexCoordsRed(
        (distortion_vertex.TanEyeAnglesR.x 
          * UVScaleOffset[0].x + UVScaleOffset[1].x)/2.0,
        distortion_vertex.TanEyeAnglesR.y 
          * UVScaleOffset[0].y + UVScaleOffset[1].y);

      scm::math::vec2f vTexCoordsGreen(
        (distortion_vertex.TanEyeAnglesG.x 
          * UVScaleOffset[0].x + UVScaleOffset[1].x)/2.0,
        distortion_vertex.TanEyeAnglesG.y 
          * UVScaleOffset[0].y + UVScaleOffset[1].y);

      scm::math::vec2f vTexCoordsBlue(
        (distortion_vertex.TanEyeAnglesB.x 
          * UVScaleOffset[0].x + UVScaleOffset[1].x)/2.0,
        distortion_vertex.TanEyeAnglesB.y 
          * UVScaleOffset[0].y + UVScaleOffset[1].y);


      ndc_2d_positions.push_back(vPos);
      tex_coords_r.push_back(vTexCoordsRed);
      tex_coords_g.push_back(vTexCoordsGreen);
      tex_coords_b.push_back(vTexCoordsBlue);
    }
  
    //populate index buffer
    /*
      NOTE: since all of the distortion mesh components start
            at index 0, we have to keep track of the index
            offsets on our own
    */
    unsigned highest_index = index_buffer_component_offset;
    for( unsigned int i = 0; i < mesh_component.IndexCount; ++i )
    {
      //manual->index( mesh_component.pIndexData[i] );
      unsigned current_index = index_buffer_component_offset + mesh_component.pIndexData[i];
      if( current_index + 1 > highest_index ) {
        highest_index = current_index + 1;
      }

      indices.push_back(current_index);

      std::cout << current_index << "\n";
    }

    index_buffer_component_offset = highest_index;
  }


  void copy_to_buffer(DistortionVertex* d_vertex_buffer) const {
    for (unsigned v(0); v < num_vertices; ++v) {

      d_vertex_buffer[v].ndc_2d_pos = ndc_2d_positions[v];

      d_vertex_buffer[v].tex_r = tex_coords_r[v];

      d_vertex_buffer[v].tex_g = tex_coords_g[v];

      d_vertex_buffer[v].tex_b = tex_coords_b[v];
    }
  }

  virtual scm::gl::vertex_format get_vertex_format() const {
    return scm::gl::vertex_format(
      0, 0, scm::gl::TYPE_VEC2F, sizeof(DistortionVertex))(
      0, 1, scm::gl::TYPE_VEC2F, sizeof(DistortionVertex))(
      0, 2, scm::gl::TYPE_VEC2F, sizeof(DistortionVertex))(
      0, 3, scm::gl::TYPE_VEC2F, sizeof(DistortionVertex));
  }

  std::vector<scm::math::vec2f> ndc_2d_positions;
  std::vector<scm::math::vec2f> tex_coords_r;
  std::vector<scm::math::vec2f> tex_coords_g;
  std::vector<scm::math::vec2f> tex_coords_b;
  std::vector<unsigned> indices;

  unsigned int num_vertices;
  unsigned int num_indices;

  unsigned int index_buffer_component_offset;
};

class GUA_OCULUSSDK2_DLL OculusSDK2Window : public Window {
 public:

  OculusSDK2Window(std::string const& display, ovrHmd const& hmd);
  virtual ~OculusSDK2Window();

  void init_context() override;

  void set_distortion(math::vec4 const& distortion);
  void set_distortion(float distortion0, float distortion1, float distortion2, float distortion3);

  // virtual
  void display(std::shared_ptr<Texture> const& texture, bool is_left);

  private:
    void initialize_distortion_meshes(ovrHmd const& hmd, RenderContext const& ctx);
    //void create_distortion_mesh();
    scm::gl::buffer_ptr distortion_mesh_vertices_[2];
    scm::gl::buffer_ptr distortion_mesh_indices_[2];
    scm::gl::vertex_array_ptr distortion_mesh_vertex_array_[2];

    unsigned num_distortion_mesh_indices[2];
    math::vec4 distortion_;

    ovrHmd registeredHMD;
};

}

#endif  // GUA_OCULUSSDK2_WINDOW_HPP
