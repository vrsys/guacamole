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

#ifndef GUA_SINGLETON_HPP
#define GUA_SINGLETON_HPP

#include <cstddef>

/**
 * This is base class for singletons.
 *
 * Singletons are classes, which are only instanciated once.
 */

namespace gua
{
template <typename T>
class Singleton
{
  public:
    /**
     * Gets the instance.
     *
     * Singletons are classes, which are only instanciated once. This
     * method will create this instance if necessary and return a pointer
     * to it.
     *
     * \return The instance of this singleton.
     */
    static T* instance()
    {
        static T instance;
        return &instance;
    };

  protected:
    /**
     * Constructor.
     *
     * Has to be private in derived classe.
     */
    Singleton(){};

    /**
     * Destructor.
     *
     * Has to be private in derived classe.
     */
    virtual ~Singleton(){};

  private:
    Singleton(Singleton const& copy) = delete;
    Singleton& operator=(Singleton const&) = delete;
};

} // namespace gua

#endif // SINGLETON_HPP
