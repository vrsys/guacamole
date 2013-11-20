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

// class header
#include <gua/renderer/VolumeLoader.hpp>

// guacamole headers
#include <gua/utils/TextFile.hpp>
#include <gua/utils/logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/scenegraph/GeometryNode.hpp>
#include <gua/scenegraph/TransformNode.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/GeometryLoader.hpp>
#include <gua/renderer/Volume.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/ShadingModelDatabase.hpp>

namespace gua {

	////////////////////////////////////////////////////////////////////////////////
	VolumeLoader::VolumeLoader() : LoaderBase(), _supported_file_extensions() {
		_supported_file_extensions.insert("raw");
		_supported_file_extensions.insert("vol");
	}

	//VolumeLoader::VolumeLoader()
	//	{}

	std::shared_ptr<Node> VolumeLoader::load(std::string const& file_name,
		unsigned flags) {

		try {
			GeometryDatabase::instance()->add(
				file_name, std::make_shared<Volume>(file_name));

			auto result = std::make_shared<GeometryNode>("unnamed_volume");
			result->data.set_geometry(file_name);
			result->data.set_material("");

			return result;

		}
		catch (std::exception & e) {
			WARNING("Warning: \"%s\" \n", e.what());
			WARNING("Failed to load Volume object \"%s\": ", file_name.c_str());
			return nullptr;
		}
	}

	//std::vector<Mesh*> const VolumeLoader::load_from_buffer(char const* buffer_name,
	//	unsigned buffer_size,
	//	bool build_kd_tree) {

	//	auto importer = std::make_shared<Assimp::Importer>();

	//	aiScene const* scene(importer->ReadFileFromMemory(
	//		buffer_name,
	//		buffer_size,
	//		aiProcessPreset_TargetRealtime_Quality | aiProcess_CalcTangentSpace));

	//	std::vector<Mesh*> meshes;

	//	for (unsigned int n = 0; n < scene->mNumMeshes; ++n) {
	//		meshes.push_back(new Mesh(scene->mMeshes[n], importer, build_kd_tree));
	//	}

	//	return meshes;

	//}

	bool VolumeLoader::is_supported(std::string const& file_name) const {
		std::vector<std::string> filename_decomposition =
			gua::string_utils::split(file_name, '.');
		return filename_decomposition.empty()
			? false
			: _supported_file_extensions.count(filename_decomposition.back()) >
			0;
	}

	
}
