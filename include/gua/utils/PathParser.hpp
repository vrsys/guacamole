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

#ifndef GUA_PATH_PARSER_HPP
#define GUA_PATH_PARSER_HPP

#include <vector>
#include <string>

/**
 * This class may be used to parse a path.
 *
 */

namespace gua
{
class PathParser
{
  public:
    /**
     * Constructor.
     *
     * This constructs a PathParser
     */
    PathParser();

    /**
     * Parses a path.
     *
     * This function parses a path and stores each substring of the path
     * which is separated by "/" into a vector cell.
     * Special attention is payed to a leading "/", which is considered
     * as part of the path and therefore also added to the vector.
     * Furthermore a finishing "/" sets a trigger.
     *
     * \param path       The path to be parsed.
     */
    void parse(std::string const& path);

    std::string get_path(bool ignore_last_entry = false) const;

    /**
     * Returns a parsed path.
     *
     * This function returns the vector containing the parsed path's data.
     *
     * \return path_data The parsed path's data.
     */
    std::vector<std::string> const& get_parsed_path() const;

    /**
     * Returns if the pars has a "/" at its end.
     *
     * This function returns true if the trigger on a finishing "/" is set.
     *
     * \return finished_by_slash The value of the trigger.
     */
    bool path_is_finished_by_slash() const;

    void make_absolute(std::string const& path_from_cwd);

  private:
    std::vector<std::string> parsed_path_;
    bool finished_by_slash_;
};

} // namespace gua

#endif // PATH_PARSER_HPP
