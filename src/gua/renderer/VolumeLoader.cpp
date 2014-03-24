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
#include "gua/renderer/VolumeLoader.hpp"

// guacamole headers
#include <gua/platform.hpp>
#include <gua/scenegraph/TransformNode.hpp>
#include <gua/renderer/MeshLoader.hpp>
#include <gua/renderer/NURBSLoader.hpp>
#include <gua/scenegraph/VolumeNode.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/renderer/Volume.hpp>


#include <gua/utils/TextFile.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>


namespace gua {

	////////////////////////////////////////////////////////////////////////////////

	std::unordered_map<std::string, std::shared_ptr<Node>>
		VolumeLoader::loaded_files_ =
		std::unordered_map<std::string, std::shared_ptr<Node>>();

	////////////////////////////////////////////////////////////////////////////////
	VolumeLoader::VolumeLoader() : LoaderBase(), _supported_file_extensions() {
		_supported_file_extensions.insert("raw");
		_supported_file_extensions.insert("vol");
	}

	//VolumeLoader::VolumeLoader()
	//	{}

	std::shared_ptr<Node> VolumeLoader::create_volume_from_file(std::string const& node_name,
																std::string const& file_name,
																unsigned flags)
	{

		std::shared_ptr<Node> cached_node;
		std::string key(file_name + "_" + string_utils::to_string(flags));

		auto searched(loaded_files_.find(key));
		if (searched != loaded_files_.end()) {

			cached_node = searched->second;

		}
		else {

			if (is_supported(file_name)) {
				cached_node = load(file_name, flags);
				cached_node->update_cache();
				loaded_files_.insert(std::make_pair(key, cached_node));

				// normalize volume position and rotation
				if (flags & VolumeLoader::NORMALIZE_POSITION || flags & VolumeLoader::NORMALIZE_SCALE) {
					auto bbox = cached_node->get_bounding_box();

					if (flags & VolumeLoader::NORMALIZE_POSITION) {
						auto center((bbox.min + bbox.max)*0.5);
						cached_node->translate(-center);
					}

					if (flags & VolumeLoader::NORMALIZE_SCALE) {
						auto size(bbox.max - bbox.min);
						auto max_size(std::max(std::max(size.x, size.y), size.z));
						cached_node->scale(1.f / max_size);
					}

				}
			}

			if (!cached_node) {

				Logger::LOG_WARNING << "Unable to load " << file_name << ": Volume Type is not supported!" << std::endl;
			}
		}

		if (cached_node) {
			auto copy(cached_node->deep_copy());

			copy->set_name(node_name);
			return copy;
		}

		return std::make_shared<TransformNode>(node_name);
	}

	std::shared_ptr<Node> VolumeLoader::load(std::string const& file_name,
											 unsigned flags)
	{
		try {
			GeometryDatabase::instance()->add(
				file_name, std::make_shared<Volume>(file_name));

			auto result = std::make_shared<VolumeNode>("unnamed_volume");
			result->data.set_volume(file_name);

			return result;

		}
		catch (std::exception &e) {
			Logger::LOG_WARNING << "Failed to load Volume object \"" << file_name << "\": " << e.what() << std::endl;
			return nullptr;
		}
	}

	bool VolumeLoader::is_supported(std::string const& file_name) const {
		std::vector<std::string> filename_decomposition =
			gua::string_utils::split(file_name, '.');
		return filename_decomposition.empty()
			? false
			: _supported_file_extensions.count(filename_decomposition.back()) >	0;
	}

}
