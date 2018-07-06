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

#ifndef GUA_NAMED_SHARED_MEMORY_CONTROLLER_H
#define GUA_NAMED_SHARED_MEMORY_CONTROLLER_H

// guacamole headers
#include <gua/utils/Singleton.hpp>
#include <gua/utils/NamedSharedMemorySegment.hpp>
#include <gua/databases/Database.hpp>

// external headers
#include <boost/noncopyable.hpp>
#include <cstddef>


namespace gua
{

  /**
   * Represents a shared memory segment.
   *
   * \ingroup av_daemon
   */
  class NamedSharedMemoryController : public Singleton<NamedSharedMemoryController> {

  public:

    void add_memory_segment(std::string const& segment_name, uint64_t size_in_byte);
    NamedSharedMemorySegment* get_memory_segment(std::string const& segment_name);

    friend class Singleton<NamedSharedMemoryController>;

  private:

    std::map<std::string, NamedSharedMemorySegment*> mNamedMemorySegments;

    /**
     * Constructor.
     */
    NamedSharedMemoryController(){};

    /**
     * Destructor.
     */
    ~NamedSharedMemoryController(){};

  };
  
}

#endif // GUA_NAMED_SHARED_MEMORY_CONTROLLER_H