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

#include <gua/utils/Logger.hpp>
#include <gua/utils/NamedSharedMemoryController.hpp>


namespace gua
{
/*
  NamedSharedMemoryController::
  NamedSharedMemoryController() 
                            : mNamedMemorySegments()
                          {}

  NamedSharedMemoryController::
  ~NamedSharedMemoryController() {
    for (auto named_shared_memory_ptr : mNamedMemorySegments) {
      delete named_shared_memory_ptr.second;
    }
  }
*/
  void NamedSharedMemoryController::
  add_memory_segment(std::string const& segment_name, uint64_t size_in_byte) {
    auto memory_segment_it = mNamedMemorySegments.find(segment_name);

    if (mNamedMemorySegments.end() == memory_segment_it) {
      auto new_named_memory_segment = std::make_shared<NamedSharedMemorySegment>(segment_name, size_in_byte);
      new_named_memory_segment->create_writeable();
      mNamedMemorySegments[segment_name] = new_named_memory_segment;
    } else {
      Logger::LOG_WARNING << "Named Shared Memory \"" << segment_name << "\" already exists!" << std::endl;
    }
  }
/*
  std::shared_ptr<NamedSharedMemorySegment> NamedSharedMemoryController::
  get_memory_segment(std::string const& segment_name) {
    auto memory_segment_it = mNamedMemorySegments.find(segment_name);

    if (mNamedMemorySegments.end() != memory_segment_it) {
      return memory_segment_it->second;
    } else {
      Logger::LOG_MESSAGE << "Requested Named Shared Memory \"" << segment_name << "\" does not exists!" << std::endl;
      Logger::LOG_MESSAGE << "Creating Read-Only Memory Segment \"" << segment_name << "\"." << std::endl;

      auto new_ro_named_memory_segment = std::make_shared<NamedSharedMemorySegment>(segment_name, 0);
      new_ro_named_memory_segment->create_readable();
      mNamedMemorySegments[segment_name] = new_ro_named_memory_segment;           
      return nullptr;
    }
  }
*/
}


