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

#ifndef GUA_VOLUME_LOADER_HPP
#define GUA_VOLUME_LOADER_HPP

// guacamole headers
#include <gua/volume/platform.hpp>
#include <gua/utils/Singleton.hpp>
#include <gua/databases/Database.hpp>

// external headers
#include <string>
#include <list>
#include <memory>

namespace gua
{
class Node;
class InnerNode;
class VolumeNode;

/**
 * Loads and draws meshes.
 *
 * This class can load mesh data from files and display them in multiple
 * contexts. A VolumeLoader object is made of several Mesh objects.
 */
class GUA_VOLUME_DLL VolumeLoader
{
  public:
    enum Flags
    {
        DEFAULTS = 0,
        MAKE_PICKABLE = 1 << 0,
        NORMALIZE_POSITION = 1 << 1,
        NORMALIZE_SCALE = 1 << 2
    };

    /**
     * Default constructor.
     *
     * Constructs a new and empty VolumeLoader.
     */
    VolumeLoader();

    /**
     * Constructor from a file.
     *
     * Creates a new VolumeLoader from a given file.
     *
     * \param file_name        The file to load the meh's data from.
     */
    /**
     *
     */
    std::shared_ptr<node::Node> create_volume_from_file(std::string const& node_name, std::string const& file_name, unsigned flags);

    std::shared_ptr<node::Node> load(std::string const& file_name, unsigned flags);

    /**
     * Constructor from memory buffer.
     *
     * Creates a new VolumeLoader from a existing memory buffer.
     *
     * \param buffer_name      The buffer to load the meh's data from.
     * \param buffer_size      The buffer's size.
     */
    // std::vector<Volume*> const load_from_buffer(char const* buffer_name,
    //  unsigned buffer_size);

    bool is_supported(std::string const& file_name) const;

  private:
    // static std::unordered_map<std::string, std::shared_ptr<::gua::node::Node>> loaded_files_;

    boost::unordered_set<std::string> _supported_file_extensions;
};

} // namespace gua

#endif // GUA_VOLUME_LOADER_HPP
