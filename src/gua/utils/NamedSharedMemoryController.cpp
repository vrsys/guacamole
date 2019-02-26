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
// check whether memory segment was created already by this process
bool NamedSharedMemoryController::check_memory_segment_exists(std::string const& segment_name) const
{
    auto memory_segment_it = mNamedMemorySegments.find(segment_name);
    return (mNamedMemorySegments.end() != memory_segment_it);
}

bool NamedSharedMemoryController::check_constructed_object_exists(std::string const& object_name) const
{
    auto named_object_it = mNamedObjects.find(object_name);
    return (mNamedObjects.end() != named_object_it);
}

#if not defined(__WIN32__) && not defined(_WIN32) && not defined(_WIN64)
void NamedSharedMemoryController::add_memory_segment(std::string const& segment_name, uint64_t size_in_byte, bool enable_warning)
{
    if(!check_memory_segment_exists(segment_name))
    {
        auto new_named_memory_segment = std::make_shared<NamedSharedMemorySegment>(segment_name, size_in_byte);
        new_named_memory_segment->create_writeable();
        mNamedMemorySegments[segment_name] = new_named_memory_segment;
    }
    else
    {
        if(enable_warning)
        {
            Logger::LOG_WARNING << "Named Shared Memory \"" << segment_name << "\" already exists!" << std::endl;
        }
    }
}

void NamedSharedMemoryController::add_read_only_memory_segment(std::string const& segment_name, bool enable_warning)
{
    if(!check_memory_segment_exists(segment_name))
    {
        auto new_named_memory_segment = std::make_shared<NamedSharedMemorySegment>(segment_name, 0);
        new_named_memory_segment->create_readable();
        mNamedMemorySegments[segment_name] = new_named_memory_segment;
    }
    else
    {
        if(enable_warning)
        {
            Logger::LOG_WARNING << "Named Shared Memory \"" << segment_name << "\" already exists!" << std::endl;
        }
    }
}
#endif

} // namespace gua
