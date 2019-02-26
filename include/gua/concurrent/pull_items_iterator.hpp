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

#ifndef GUA_PULL_ITEMS_RANGE_HPP
#define GUA_PULL_ITEMS_RANGE_HPP

#include <boost/optional.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/range/iterator_range.hpp>

namespace gua
{
namespace concurrent
{
// Adapted from:
// http://ericniebler.com/2013/11/07/input-iterators-vs-input-ranges/
template <class T, class Container>
class pull_items_range
{
    Container& sin_;
    mutable T obj_;

    bool next() const
    {
        boost::optional<T> v = sin_->read();
        if(v)
            obj_ = *v;
        return bool(v);
    }

  public:
    struct iterator : boost::iterator_facade<iterator, T const, std::input_iterator_tag>
    {
        iterator() : rng_{} {}

      private:
        friend class pull_items_range;
        friend class boost::iterator_core_access;

        explicit iterator(pull_items_range const& rng) : rng_(rng ? &rng : nullptr) {}

        void increment()
        {
            // Don't advance a singular iterator
            BOOST_ASSERT(rng_);
            // Fetch the next element, null out the
            // iterator if it fails
            if(!rng_->next())
                rng_ = nullptr;
        }
        bool equal(iterator that) const { return rng_ == that.rng_; }

        T const& dereference() const
        {
            // Don't deref a singular iterator
            BOOST_ASSERT(rng_);
            return rng_->obj_;
        }

        pull_items_range const* rng_;
    };
    // Define const_iterator and iterator together:
    using const_iterator = iterator;

    explicit pull_items_range(Container& sin) : sin_(sin), obj_{}
    {
        next(); // prime the pump
    }

    iterator begin() const { return iterator{*this}; }

    iterator end() const { return iterator{}; }

    explicit operator bool() const { return !sin_->closed(); }

    bool operator!() const { return !sin_; }
};

} // namespace concurrent

} // namespace gua

#endif // GUA_PULL_ITEMS_RANGE_HPP
