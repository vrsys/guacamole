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

#ifndef GUA_NAMED_SHARED_MEMORY_SEGMENT_H
#define GUA_NAMED_SHARED_MEMORY_SEGMENT_H

#include <boost/noncopyable.hpp>
#include <gua/utils/Singleton.hpp>
#include <gua/databases/Database.hpp>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <cstddef>
#include <string>

namespace gua
{
/**
 * Represents a named shared memory segment.
 *
 * \ingroup av_daemon
 */
class NamedSharedMemorySegment : private boost::noncopyable
{
  public:
    /**
     * Constructor.
     */
    NamedSharedMemorySegment(std::string const& shared_memory_segment_name, uint64_t shared_memory_size_in_byte = 0);

    void create_writeable();

    void create_readable();

    template <typename T>
    void construct_named_object(std::string const& object_name)
    {
        mManagedSharedMemoryObjectPtr->construct<T>(object_name.c_str())();
    }

    template <typename T>
    void destroy_named_object(std::string const& object_name)
    {
        mManagedSharedMemoryObjectPtr->destroy<T>(object_name.c_str());
    }

    template <typename T>
    std::pair<T*, std::size_t> retrieve_named_object(std::string const& object_name)
    {
        return mManagedSharedMemoryObjectPtr->find<T>(object_name.c_str());
    }

    void write(char* const data, std::size_t byte_length, std::size_t byte_offset = 0x0);
    void read(char* data, std::size_t byte_length, std::size_t byte_offset = 0x0);

    template <typename T, typename U>
    void set_value_on_named_object(std::string const& object_name, U const& value)
    {
        *(retrieve_named_object<T>(object_name).first) = value;
    }

    template <typename INTERNAL_TYPE, typename EXTERNAL_TYPE>
    EXTERNAL_TYPE get_value_from_named_object(std::string const& object_name)
    {
        return *(retrieve_named_object<INTERNAL_TYPE>(object_name).first);
    }

    // memcpy interface
    template <typename INTERNAL_TYPE>
    void memcpy_value_from_named_object(std::string const& object_name, char* to_write, std::size_t byte_to_write)
    {
        auto& array_to_copy_from = *(retrieve_named_object<INTERNAL_TYPE>(object_name).first);
        std::cout << "Executed innermost read call\n";
        std::cout << "Reading: " << byte_to_write << " bytes\n";
        std::memcpy(to_write, (char*)&(array_to_copy_from[0]), byte_to_write);
    }

    // memcpy interface
    template <typename INTERNAL_TYPE>
    void memcpy_value_to_named_object(std::string const& object_name, char* const to_read, std::size_t byte_to_read)
    {
        auto& array_to_copy_to = *(retrieve_named_object<INTERNAL_TYPE>(object_name).first);
        std::cout << "Executed innermost write call\n";
        std::cout << "Writing: " << byte_to_read << " bytes\n";
        std::memcpy((char*)&(array_to_copy_to[0]), to_read, byte_to_read);
    }

    /**
     * Destructor.
     */
    ~NamedSharedMemorySegment();

  private:
    void _create();
    std::string mName;
    uint64_t mSize;

    // actual shared memory instance
    boost::interprocess::managed_shared_memory* mManagedSharedMemoryObjectPtr;

    bool mWasCreated;
    bool mInitRead;
};
} // namespace gua

#endif // GUA_NAMED_SHARED_MEMORY_SEGMENT_H
