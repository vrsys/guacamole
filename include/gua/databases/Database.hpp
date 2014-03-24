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

//guacamole headers
#include <gua/utils.hpp>

// external headers
#include <boost/thread.hpp>
#include <thread>
#include <memory>
#include <string>
#include <set>
#include <unordered_map>

namespace gua {

/**
 * A database for accessing data.
 *
 * It can store any type of Data. The data is mapped on strings,
 * which then can be used to access this data.
 *
 * \ingroup gua_databases
 */
template <typename T> class Database {
 public:

  typedef std::string key_type;
  typedef std::shared_ptr<T> mapped_type;

  /**
   * Adds a new entry to the data base.
   *
   * It can be accessed later with the lookup() method.
   *
   * \param k    The unique key of this entry.
   * \param date The newly added entry.
   */
  void add(key_type const& k, mapped_type const& date) {
    boost::upgrade_lock<boost::shared_mutex> lock(mutex_);
    boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
    data_[k] = date;
    keys_.insert(k);
  }

  /**
   * Check for existance of a key.
   *
   * Returns true, if an entry with the given key exists in the Database.
   *
   * \param k    The key to check for.
   * \return     Whether the given key is stored in the Database.
   */
  bool is_supported(key_type const& k) const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);
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
  mapped_type lookup(key_type const& k) {
    auto result(data_.end());

    {
      boost::shared_lock<boost::shared_mutex> lock(mutex_);
      result = data_.find(k);
    }

    if (result == data_.end()) {
      load(k);

      {
        boost::shared_lock<boost::shared_mutex> lock(mutex_);
        result = data_.find(k);
      }
    }

    if (result == data_.end()) {
      Logger::LOG_WARNING << "There is no entry \" << k << \" in the database!" << std::endl;
      return std::shared_ptr<T>();
    }

    return result->second;
  }

  /**
   * Lists all supported keys.
   *
   * \return A set containing all keys.
   */
  inline std::set<key_type> const& list_all() const { return keys_; }

  /**
   * This function gets called when a not-stored item is requested.
   *
   * Derived classes may overload this method.
   *
   * \return A set containing all keys.
   */
  virtual void load(key_type const& k) {};

 protected:
  std::unordered_map<key_type,mapped_type> data_;
  std::set<key_type> keys_;

 private:
  mutable boost::shared_mutex mutex_;

};

}

#endif  // GUA_DATABASE_HPP
