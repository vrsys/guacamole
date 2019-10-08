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

#ifndef GUA_ADD_PROPERTY_HPP
#define GUA_ADD_PROPERTY_HPP

#define GUA_ADD_PROPERTY(TYPE, NAME, VALUE)                                                                                                                                                            \
    /** \cond */                                                                                                                                                                                       \
    struct NAME##_struct                                                                                                                                                                               \
    {                                                                                                                                                                                                  \
      public:                                                                                                                                                                                          \
        NAME##_struct() : val_(VALUE) {}                                                                                                                                                               \
        NAME##_struct(TYPE const& val) : val_(val) {}                                                                                                                                                  \
                                                                                                                                                                                                       \
        TYPE& operator()() { return val_; }                                                                                                                                                            \
        TYPE const& operator()() const { return val_; }                                                                                                                                                \
                                                                                                                                                                                                       \
      private:                                                                                                                                                                                         \
        TYPE val_;                                                                                                                                                                                     \
    } NAME;                                                                                                                                                                                            \
    /** \endcond */                                                                                                                                                                                    \
    Configuration& set_##NAME(TYPE const& val)                                                                                                                                                         \
    {                                                                                                                                                                                                  \
        NAME() = val;                                                                                                                                                                                  \
        return *this;                                                                                                                                                                                  \
    }                                                                                                                                                                                                  \
    /**                                                                                                                                                                                                \
    The default value is VALUE.                                                                                                                                                                        \
     */                                                                                                                                                                                                \
    TYPE const& get_##NAME() const { return NAME(); }

#endif // GUA_ADD_PROPERTY_HPP
