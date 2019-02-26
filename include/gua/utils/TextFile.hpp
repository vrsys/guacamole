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

#ifndef GUA_TEXTFILE_HPP
#define GUA_TEXTFILE_HPP

#include <gua/platform.hpp>

#include <string>

/**
 * This class is used to read text files.
 */

namespace gua
{
class GUA_DLL TextFile
{
  public:
    /**
     * Constructor.
     *
     * This constructs a TextFile without a file.
     */
    TextFile();

    /**
     * Constructor.
     *
     * This constructs a TextFile with a file.
     *
     * \param file_name The file to be read.
     */
    TextFile(std::string const& file_name);

    /**
     * Returns if the given file is valid.
     *
     * \return The validity of the file.
     */
    bool is_valid() const;

    /**
     * Returns the given file's content.
     *
     * \return The given file's content.
     */
    std::string const& get_content() const;

    /**
     * Sets the given file's content.
     *
     * \param The new content.
     */
    void set_content(std::string const& content);

    /**
     * Saves the file
     *
     */
    bool save(bool create_subdirs = false) const;

    /**
     * Deletes the file from the file system
     *
     */
    void remove();

    /**
     * Returns the given file's name.
     *
     * \return The given file's name.
     */
    std::string const& get_file_name() const;

  private:
    std::string file_name_;
    mutable std::string content_;

    mutable bool is_loaded_;
};

} // namespace gua

#endif // GUA_TEXTFILE_HPP
