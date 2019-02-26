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

#ifndef NUMERIC_PROPERTY_HPP_
#define NUMERIC_PROPERTY_HPP_

#include <gua/events/properties/SortableProperty.hpp>
#include <gua/events/properties/StreamableProperty.hpp>

namespace gua
{
namespace events
{
template <typename T>
class NumericProperty : public virtual SortableProperty<T>, public virtual StreamableProperty<T>
{
  public:
    NumericProperty() : Property<T>() {}

    NumericProperty(T const& value) : Property<T>(value) {}

    NumericProperty(NumericProperty<T> const& to_copy) : Property<T>(to_copy) {}

    virtual ~NumericProperty() {}

    NumericProperty<T> operator+(NumericProperty<T> const& rhs) const { return NumericProperty(Property<T>::get() + rhs.get()); }

    NumericProperty<T> operator+(T const& rhs) const { return NumericProperty(Property<T>::get() + rhs); }

    NumericProperty<T> operator-(NumericProperty<T> const& rhs) const { return NumericProperty(Property<T>::get() - rhs.get()); }

    NumericProperty<T> operator-(T const& rhs) const { return NumericProperty(Property<T>::get() - rhs); }

    NumericProperty<T> operator*(NumericProperty<T> const& rhs) const { return NumericProperty(Property<T>::get() * rhs.get()); }

    NumericProperty<T> operator*(T const& rhs) const { return NumericProperty(Property<T>::get() * rhs); }

    NumericProperty<T> operator/(NumericProperty<T> const& rhs) const { return NumericProperty(Property<T>::get() / rhs.get()); }

    NumericProperty<T> operator/(T const& rhs) const { return NumericProperty(Property<T>::get() / rhs); }

    NumericProperty<T>& operator+=(NumericProperty<T> const& rhs)
    {
        set(Property<T>::get() + rhs.get());
        return *this;
    }

    NumericProperty<T>& operator+=(T const& rhs)
    {
        set(Property<T>::get() + rhs);
        return *this;
    }

    NumericProperty<T>& operator-=(NumericProperty<T> const& rhs)
    {
        set(Property<T>::get() - rhs.get());
        return *this;
    }

    NumericProperty<T>& operator-=(T const& rhs)
    {
        set(Property<T>::get() - rhs);
        return *this;
    }

    NumericProperty<T>& operator*=(NumericProperty<T> const& rhs)
    {
        set(Property<T>::get() * rhs.get());
        return *this;
    }

    NumericProperty<T>& operator*=(T const& rhs)
    {
        set(Property<T>::get() * rhs);
        return *this;
    }

    NumericProperty<T>& operator/=(NumericProperty<T> const& rhs)
    {
        set(Property<T>::get() / rhs.get());
        return *this;
    }

    NumericProperty<T>& operator/=(T const& rhs)
    {
        set(Property<T>::get() / rhs);
        return *this;
    }

    NumericProperty<T>& operator++()
    {
        T tmp(Property<T>::get());
        set(++tmp);
        return *this;
    }

    NumericProperty<T> operator++(int dummy)
    {
        T tmp(Property<T>::get());
        set(tmp++);
        return NumericProperty<T>(tmp);
    }

    NumericProperty<T>& operator--()
    {
        T tmp(Property<T>::get());
        set(--tmp);
        return *this;
    }

    NumericProperty<T> operator--(int dummy)
    {
        T tmp(Property<T>::get());
        set(tmp--);
        return NumericProperty<T>(tmp);
    }
};

} // namespace events
} // namespace gua

#endif /* NUMERIC_PROPERTY_HPP_ */
