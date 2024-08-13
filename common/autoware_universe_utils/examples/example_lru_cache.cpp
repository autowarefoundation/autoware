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

#include "autoware/universe_utils/system/lru_cache.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>

using autoware::universe_utils::LRUCache;

// Fibonacci calculation with LRU cache
int64_t fibonacci_with_cache(int n, LRUCache<int, int64_t> * cache)
{
  if (n <= 1) return n;

  if (cache->contains(n)) {
    return *cache->get(n);
  }
  int64_t result = fibonacci_with_cache(n - 1, cache) + fibonacci_with_cache(n - 2, cache);
  cache->put(n, result);
  return result;
}

// Fibonacci calculation without cache
int64_t fibonacci_no_cache(int n)
{
  if (n <= 1) return n;
  return fibonacci_no_cache(n - 1) + fibonacci_no_cache(n - 2);
}

// Helper function to measure execution time
template <typename Func, typename... Args>
int64_t measure_time(Func func, Args &&... args)
{
  auto start = std::chrono::high_resolution_clock::now();
  func(std::forward<Args>(args)...);
  auto end = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

int main()
{
  const int max_n = 40;                 // Increased for more noticeable time difference
  LRUCache<int, int64_t> cache(max_n);  // Cache size set to MAX_N

  std::cout << "Comparing Fibonacci calculation time with and without LRU cache:\n\n";
  std::cout << "n\tWith Cache (μs)\tWithout Cache (μs)\n";
  std::cout << std::string(43, '-') << "\n";

  for (int i = 30; i <= max_n; ++i) {  // Starting from 30 for more significant differences
    int64_t time_with_cache = measure_time([i, &cache]() { fibonacci_with_cache(i, &cache); });
    int64_t time_without_cache = measure_time([i]() { fibonacci_no_cache(i); });

    std::cout << i << "\t" << time_with_cache << "\t\t" << time_without_cache << "\n";

    // Clear the cache after each iteration to ensure fair comparison
    cache.clear();
  }

  return 0;
}
