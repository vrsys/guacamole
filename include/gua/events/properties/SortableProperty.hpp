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

#ifndef SORTABLE_PROPERTY_HPP_
#define SORTABLE_PROPERTY_HPP_

#include <gua/events/properties/Property.hpp>

namespace gua
{
namespace events
{
template <typename T>
class SortableProperty : public virtual Property<T>
{
  public:
    SortableProperty() : Property<T>() {}

    SortableProperty(T const& value) : Property<T>(value) {}

    SortableProperty(SortableProperty<T> const& to_copy) : Property<T>(to_copy) {}

    virtual ~SortableProperty() {}

    bool operator>(SortableProperty<T> const& rhs) const { return Property<T>::get() > rhs.get(); }

    bool operator>(T const& rhs) const { return Property<T>::get() > rhs; }

    bool operator>=(SortableProperty<T> const& rhs) const { return Property<T>::get() >= rhs.get(); }

    bool operator>=(T const& rhs) const { return Property<T>::get() >= rhs; }

    bool operator<(SortableProperty<T> const& rhs) const { return Property<T>::get() < rhs.get(); }

    bool operator<(T const& rhs) const { return Property<T>::get() < rhs; }

    bool operator<=(SortableProperty<T> const& rhs) const { return Property<T>::get() <= rhs.get(); }

    bool operator<=(T const& rhs) const { return Property<T>::get() <= rhs; }
};

} // namespace events
} // namespace gua

#endif /* SORTABLE_PROPERTY_HPP_ */
