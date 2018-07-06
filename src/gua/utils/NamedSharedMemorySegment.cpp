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

#include <gua/utils/NamedSharedMemorySegment.hpp>


namespace gua
{

  NamedSharedMemorySegment::
  NamedSharedMemorySegment(std::string const& shared_memory_segment_name,
                          uint64_t shared_memory_size_in_byte) 
                            : mName(shared_memory_segment_name),
                              mSize(shared_memory_size_in_byte),
                              mSharedMemoryObjectPtr(nullptr),
                              mMappedRegionPtr(nullptr),
                              mWasCreated(false),
                              mWasMapped(false),
                              mInitRead(false)
                          {}

  NamedSharedMemorySegment::
  ~NamedSharedMemorySegment() {
    if (mWasCreated) {
      delete mSharedMemoryObjectPtr;
      delete mMappedRegionPtr;
    }

    if(mWasMapped) {
      delete mMappedRegionPtr;
    }
  }



  void NamedSharedMemorySegment::create_writeable() {
    boost::interprocess::shared_memory_object::remove(mName.c_str());

    mSharedMemoryObjectPtr = new boost::interprocess::shared_memory_object(::boost::interprocess::create_only, mName.c_str(), ::boost::interprocess::read_write);
    mWasCreated            = true;

    mSharedMemoryObjectPtr->truncate(mSize);
    mMappedRegionPtr = new boost::interprocess::mapped_region (*mSharedMemoryObjectPtr, ::boost::interprocess::read_write);
    mWasMapped = true;
  }

  void NamedSharedMemorySegment::create_readable() {
    boost::interprocess::shared_memory_object::remove(mName.c_str());
    mSharedMemoryObjectPtr = new boost::interprocess::shared_memory_object(::boost::interprocess::open_only, mName.c_str(), ::boost::interprocess::read_only);
    mWasCreated            = true;
    mInitRead              = true;

    mMappedRegionPtr = new boost::interprocess::mapped_region (*mSharedMemoryObjectPtr, ::boost::interprocess::read_only);
    mWasMapped = true;
  }


  void NamedSharedMemorySegment::write(char* const data, std::size_t byte_length, std::size_t byte_offset) {
    std::memcpy( (char*)(mMappedRegionPtr->get_address()) + byte_offset, data, byte_length);
  }

  void NamedSharedMemorySegment::read(char* data, std::size_t byte_length, std::size_t byte_offset) {
    std::memcpy( data, (char*)(mMappedRegionPtr->get_address()) + byte_offset, byte_length);
  }

}


