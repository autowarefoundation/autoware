// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef AUTOWARE__UNIVERSE_UTILS__SYSTEM__LRU_CACHE_HPP_
#define AUTOWARE__UNIVERSE_UTILS__SYSTEM__LRU_CACHE_HPP_

#include <cstddef>
#include <list>
#include <optional>
#include <unordered_map>
#include <utility>

namespace autoware::universe_utils
{

/**
 * @brief A template class for LRU (Least Recently Used) Cache.
 *
 * This class implements a simple LRU cache using a combination of a list and a hash map.
 *
 * @tparam Key The type of keys.
 * @tparam Value The type of values.
 * @tparam Map The type of underlying map, defaulted to std::unordered_map.
 */
template <typename Key, typename Value, template <typename...> class Map = std::unordered_map>
class LRUCache
{
private:
  size_t capacity_;                              ///< The maximum capacity of the cache.
  std::list<std::pair<Key, Value>> cache_list_;  ///< List to maintain the order of elements.
  Map<Key, typename std::list<std::pair<Key, Value>>::iterator>
    cache_map_;  ///< Map for fast access to elements.

public:
  /**
   * @brief Construct a new LRUCache object.
   *
   * @param size The capacity of the cache.
   */
  explicit LRUCache(size_t size) : capacity_(size) {}

  /**
   * @brief Get the capacity of the cache.
   *
   * @return The capacity of the cache.
   */
  [[nodiscard]] size_t capacity() const { return capacity_; }

  /**
   * @brief Insert a key-value pair into the cache.
   *
   * If the key already exists, its value is updated and it is moved to the front.
   * If the cache exceeds its capacity, the least recently used element is removed.
   *
   * @param key The key to insert.
   * @param value The value to insert.
   */
  void put(const Key & key, const Value & value)
  {
    auto it = cache_map_.find(key);
    if (it != cache_map_.end()) {
      cache_list_.erase(it->second);
    }
    cache_list_.push_front({key, value});
    cache_map_[key] = cache_list_.begin();

    if (cache_map_.size() > capacity_) {
      auto last = cache_list_.back();
      cache_map_.erase(last.first);
      cache_list_.pop_back();
    }
  }

  /**
   * @brief Retrieve a value from the cache.
   *
   * If the key does not exist in the cache, std::nullopt is returned.
   * If the key exists, the value is returned and the element is moved to the front.
   *
   * @param key The key to retrieve.
   * @return The value associated with the key, or std::nullopt if the key does not exist.
   */
  std::optional<Value> get(const Key & key)
  {
    auto it = cache_map_.find(key);
    if (it == cache_map_.end()) {
      return std::nullopt;
    }
    cache_list_.splice(cache_list_.begin(), cache_list_, it->second);
    return it->second->second;
  }

  /**
   * @brief Clear the cache.
   *
   * This removes all elements from the cache.
   */
  void clear()
  {
    cache_list_.clear();
    cache_map_.clear();
  }

  /**
   * @brief Get the current size of the cache.
   *
   * @return The number of elements in the cache.
   */
  [[nodiscard]] size_t size() const { return cache_map_.size(); }

  /**
   * @brief Check if the cache is empty.
   *
   * @return True if the cache is empty, false otherwise.
   */
  [[nodiscard]] bool empty() const { return cache_map_.empty(); }

  /**
   * @brief Check if a key exists in the cache.
   *
   * @param key The key to check.
   * @return True if the key exists, false otherwise.
   */
  [[nodiscard]] bool contains(const Key & key) const
  {
    return cache_map_.find(key) != cache_map_.end();
  }
};

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__SYSTEM__LRU_CACHE_HPP_
