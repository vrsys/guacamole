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

#ifndef GUA_MATERIAL_LOADER_HPP
#define GUA_MATERIAL_LOADER_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/TriMeshRessource.hpp>

// external headers
#include <string>
#include <list>
#include <memory>

#if ASSIMP_VERSION == 3
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#else
#include <assimp/assimp.hpp>
#include <assimp/aiPostProcess.h>
#include <assimp/aiScene.h>
#endif

namespace Assimp { class Importer; }

namespace gua {

class Node;
class GeometryNode;

/**
 * Loads and draws meshes.
 *
 * This class can load mesh data from files and display them in multiple
 * contexts. A MaterialLoader object is made of several Mesh objects.
 */
class GUA_DLL MaterialLoader {
 public:

  enum ShadingCapabilities {
    DIFFUSE_MAP = 1,
    DIFFUSE_COLOR = 2,
    SPECULAR_MAP = 4,
    SPECULAR_COLOR = 8,
    EMIT_MAP = 16,
    EMIT_COLOR = 32,
    AMBIENT_MAP = 64,
    AMBIENT_COLOR = 128,
    SHININESS_MAP = 256,
    SHININESS_COLOR = 512,
    NORMAL_MAP = 1024,
    REFLECTION_MAP = 2048,
    OPACITY_MAP = 4096
  };

  std::string load_material(aiMaterial const* material,
                            std::string const& file_name) const;

 private:
  std::string load_shading_model(unsigned capabilities) const;

};

}

#endif  // GUA_MATERIAL_LOADER_HPP
