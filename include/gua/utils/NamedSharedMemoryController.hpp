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
#include <gua/utils/SharedPtrSingleton.hpp>
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
  class NamedSharedMemoryController : public SharedPtrSingleton<NamedSharedMemoryController> {

  public:

    void add_memory_segment(std::string const& segment_name, uint64_t size_in_byte);
    //std::shared_ptr<NamedSharedMemorySegment> get_memory_segment(std::string const& segment_name);


    void add_read_only_memory_segment(std::string const& segment_name);

    void write_to_segment(std::string const& segment_name, 
                          char* const data, std::size_t byte_length, std::size_t byte_offset = 0x0) {
      // for now without checking whether the segment actually exists
      mNamedMemorySegments[segment_name]->write(data, byte_length, byte_offset);
    }

    void read_from_segment(std::string const& segment_name, 
                          char* data, std::size_t byte_length, std::size_t byte_offset = 0x0) {
      // for now without checking whether the segment actually exists
      mNamedMemorySegments[segment_name]->read(data, byte_length, byte_offset);
    }

    template <typename INTERNAL_TYPE>
    void construct_named_object_on_segment(std::string const& segment_name, std::string const& object_name) {
      mNamedObjects[object_name] = mNamedMemorySegments[segment_name];
      mNamedObjects[object_name]->construct_named_object<INTERNAL_TYPE>(object_name);
    }

    template <typename INTERNAL_TYPE, typename EXTERNAL_TYPE>
    void set_value_for_named_object(std::string const& object_name, EXTERNAL_TYPE const& value){
      mNamedObjects[object_name]->set_value_on_named_object<INTERNAL_TYPE, EXTERNAL_TYPE>(object_name, value);
    }

    template <typename INTERNAL_TYPE, typename EXTERNAL_TYPE>
    EXTERNAL_TYPE get_value_from_named_object(std::string const& object_name){
      return mNamedObjects[object_name]->get_value_from_named_object<INTERNAL_TYPE, EXTERNAL_TYPE>(object_name);
    }

    void register_remotely_constructed_object_on_segment(std::string const& segment_name, std::string const& object_name) {
      mNamedObjects[object_name] = mNamedMemorySegments[segment_name];
    }


    template <typename INTERNAL_TYPE>
    void destroy_named_object(std::string const& object_name) {
      mNamedObjects[object_name]->destroy_named_object<INTERNAL_TYPE>(object_name);
    }

    friend class SharedPtrSingleton<NamedSharedMemoryController>;

  private:
    // maps segment name to actual segment
    std::map<std::string, std::shared_ptr<NamedSharedMemorySegment> > mNamedMemorySegments;
    
    std::map<std::string, std::shared_ptr<NamedSharedMemorySegment> > mNamedObjects;
    /**
     * Constructor.
     */
    NamedSharedMemoryController(){};

    /**
     * Destructor.
     */
    //~NamedSharedMemoryController(){};

  };
  
}

#endif // GUA_NAMED_SHARED_MEMORY_CONTROLLER_H