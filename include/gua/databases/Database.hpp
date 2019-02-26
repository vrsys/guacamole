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

#ifndef GUA_DATABASE_HPP
#define GUA_DATABASE_HPP

// guacamole headers
#include <gua/utils.hpp>

// external headers
#include <mutex>
#include <boost/optional.hpp>
#include <boost/none.hpp>
#include <thread>
#include <memory>
#include <string>
#include <set>
#include <unordered_map>

namespace gua
{
/**
 * A database for accessing data.
 *
 * It can store any type of Data. The data is mapped on strings,
 * which then can be used to access this data.
 *
 * \ingroup gua_databases
 */
template <typename T, typename K = std::string>
class Database
{
  public:
    using key_type = K;
    using mapped_type = std::shared_ptr<T>;

    /**
     * Adds a new entry to the data base.
     *
     * It can be accessed later with the lookup() method.
     *
     * \param k    The unique key of this entry.
     * \param date The newly added entry.
     */
    void add(key_type const& k, mapped_type const& date)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_[k] = date;
        keys_.insert(k);
    }

    /**
     * Adds a new entry to the data base.
     *
     * It can be accessed later with the lookup() method.
     *
     * \param k    The unique key of this entry.
     * \param date The newly added entry.
     */
    void add_if_not_element(key_type const& k, mapped_type const& date)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto result = data_.find(k);
        if(result == data_.end())
        {
            data_[k] = date;
            keys_.insert(k);
        }
    }

    /**
     * Remove entry to the data base.
     */
    void remove(key_type const& k)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        auto entry = data_.find(k);
        if(entry != data_.end())
        {
            data_.erase(entry);
        }

        auto key = keys_.find(k);
        if(key != keys_.end())
        {
            keys_.erase(key);
        }
    }

    /**
     * Check for existance of a key.
     *
     * Returns true, if an entry with the given key exists in the Database.
     *
     * \param k    The key to check for.
     * \return     Whether the given key is stored in the Database.
     */
    bool contains(key_type const& k) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return keys_.find(k) != keys_.end();
    }

    /**
     * Gets an entry from the Database
     *
     * Returns a entry from the Database. It will return nullptr if
     * the entry in question does not exist.
     *
     * \param  k   The key of the entry.
     * \return     A shared pointer to the data of the requested
     *             entry. nullptr if the entry does not exist.
     */
    mapped_type lookup(key_type const& k) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto result = data_.find(k);

        if(result == data_.end())
        {
            return std::shared_ptr<T>();
        }
        else
        {
            return result->second;
        }
    }

    /**
     * Lists all supported keys.
     *
     * \return A set containing all keys.
     */
    inline std::set<key_type> const& list_all() const { return keys_; }

  protected:
    std::unordered_map<key_type, mapped_type> data_;
    std::set<key_type> keys_;

  private:
    mutable std::mutex mutex_;
};

template <typename K, typename T>
auto lookup(Database<T>& db, typename Database<T>::key_type const& k) -> decltype(boost::make_optional(db.lookup(k)))
{
    if(db.contains(k))
        return boost::make_optional(db.lookup(k));
    else
        return boost::none;
}

} // namespace gua

#endif // GUA_DATABASE_HPP
