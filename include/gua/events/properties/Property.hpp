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

#ifndef PROPERTY_HPP_
#define PROPERTY_HPP_

#include <gua/events/Signal.hpp>

namespace gua
{
namespace events
{
template <typename T>
class Property
{
  public:
    Property() {}

    Property(T const& value) : value_(value) {}

    Property(Property<T> const& to_copy) : value_(to_copy.value_) {}

    virtual ~Property() {}

    virtual Signal<T> const& on_change() const { return on_change_; }

    virtual void set(T const& value)
    {
        if(value != value_)
        {
            value_ = value;
            on_change_.emit(value_);
        }
    }

    virtual T const& get() const { return value_; }

    Property<T>& operator=(T const& rhs)
    {
        set(rhs);
        return *this;
    }

    Property<T>& operator=(Property<T> const& rhs)
    {
        set(rhs.value_);
        return *this;
    }

    bool operator==(Property<T> const& rhs) const { return Property<T>::get() == rhs.get(); }

    bool operator==(T const& rhs) const { return Property<T>::get() == rhs; }

    bool operator!=(Property<T> const& rhs) const { return Property<T>::get() != rhs.get(); }

    bool operator!=(T const& rhs) const { return Property<T>::get() != rhs; }

  private:
    T value_;
    Signal<T> on_change_;
};

} // namespace events
} // namespace gua

#endif /* PROPERTY_HPP_ */
