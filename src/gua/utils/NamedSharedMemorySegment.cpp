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
NamedSharedMemorySegment::NamedSharedMemorySegment(std::string const& shared_memory_segment_name, uint64_t shared_memory_size_in_byte)
    : mName(shared_memory_segment_name), mSize(shared_memory_size_in_byte), mManagedSharedMemoryObjectPtr(nullptr), mWasCreated(false), mInitRead(false)
{
}

NamedSharedMemorySegment::~NamedSharedMemorySegment()
{
    if(mWasCreated)
    {
        boost::interprocess::shared_memory_object::remove(mName.c_str());
        delete mManagedSharedMemoryObjectPtr;
    }
}

void NamedSharedMemorySegment::create_writeable()
{
    boost::interprocess::shared_memory_object::remove(mName.c_str());

    mManagedSharedMemoryObjectPtr = new boost::interprocess::managed_shared_memory(::boost::interprocess::create_only, mName.c_str(), mSize);
}

void NamedSharedMemorySegment::create_readable() { mManagedSharedMemoryObjectPtr = new boost::interprocess::managed_shared_memory(::boost::interprocess::open_only, mName.c_str()); }

void NamedSharedMemorySegment::write(char* const data, std::size_t byte_length, std::size_t byte_offset)
{
    std::memcpy((char*)(mManagedSharedMemoryObjectPtr->get_address()) + byte_offset, data, byte_length);
}

void NamedSharedMemorySegment::read(char* data, std::size_t byte_length, std::size_t byte_offset)
{
    std::memcpy(data, (char*)(mManagedSharedMemoryObjectPtr->get_address()) + byte_offset, byte_length);
}

} // namespace gua
