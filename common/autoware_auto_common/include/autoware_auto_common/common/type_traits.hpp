// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Developed by Apex.AI, Inc.

#include "autoware_auto_common/common/types.hpp"
#include "autoware_auto_common/common/visibility_control.hpp"

#include <cstdint>
#include <tuple>
#include <type_traits>

#ifndef AUTOWARE_AUTO_COMMON__COMMON__TYPE_TRAITS_HPP_
#define AUTOWARE_AUTO_COMMON__COMMON__TYPE_TRAITS_HPP_

namespace autoware
{
namespace common
{
namespace type_traits
{

///
/// @brief      A helper function to be used in static_assert to indicate an impossible branch.
///
/// @details    Typically used when a static_assert is used to guard a certain default
///             implementation to never be executed and to show a helpful message to the user.
///
/// @tparam     T     Any type needed to delay the compilation of this function until it is used.
///
/// @return     A boolean that should be false for any type passed into this function.
///
template <typename T>
constexpr inline autoware::common::types::bool8_t COMMON_PUBLIC impossible_branch() noexcept
{
  return sizeof(T) == 0;
}

/// Find an index of a type in a tuple
template <class QueryT, class TupleT>
struct COMMON_PUBLIC index
{
  static_assert(!std::is_same<TupleT, std::tuple<>>::value, "Could not find QueryT in given tuple");
};

/// Specialization for a tuple that starts with the HeadT type. End of recursion.
template <class HeadT, class... Tail>
struct COMMON_PUBLIC index<HeadT, std::tuple<HeadT, Tail...>>
: std::integral_constant<std::int32_t, 0>
{
};

/// Specialization for a tuple with a type different to QueryT that calls the recursive step.
template <class QueryT, class HeadT, class... Tail>
struct COMMON_PUBLIC index<QueryT, std::tuple<HeadT, Tail...>>
: std::integral_constant<std::int32_t, 1 + index<QueryT, std::tuple<Tail...>>::value>
{
};

///
/// @brief      Visit every element in a tuple.
///
///             This specialization indicates the end of the recursive tuple traversal.
///
/// @tparam     I         Current index.
/// @tparam     Callable  Callable type, usually a lambda with one auto input parameter.
/// @tparam     TypesT    Types in the tuple.
///
/// @return     Does not return anything. Capture variables in a lambda to return any values.
///
template <std::size_t I = 0UL, typename Callable, typename... TypesT>
COMMON_PUBLIC inline constexpr typename std::enable_if_t<I == sizeof...(TypesT)> visit(
  std::tuple<TypesT...> &, Callable) noexcept
{
}
/// @brief      Same as the previous specialization but for const tuple.
template <std::size_t I = 0UL, typename Callable, typename... TypesT>
COMMON_PUBLIC inline constexpr typename std::enable_if_t<I == sizeof...(TypesT)> visit(
  const std::tuple<TypesT...> &, Callable) noexcept
{
}

///
/// @brief      Visit every element in a tuple.
///
///             This specialization is used to apply the callable to an element of a tuple and
///             recursively call this function on the next one.
///
/// @param      tuple     The tuple instance
/// @param[in]  callable  A callable, usually a lambda with one auto input parameter.
///
/// @tparam     I         Current index.
/// @tparam     Callable  Callable type, usually a lambda with one auto input parameter.
/// @tparam     TypesT    Types in the tuple.
///
/// @return     Does not return anything. Capture variables in a lambda to return any values.
///
template <std::size_t I = 0UL, typename Callable, typename... TypesT>
COMMON_PUBLIC inline constexpr typename std::enable_if_t<I != sizeof...(TypesT)> visit(
  std::tuple<TypesT...> & tuple, Callable callable) noexcept
{
  callable(std::get<I>(tuple));
  visit<I + 1UL, Callable, TypesT...>(tuple, callable);
}
/// @brief      Same as the previous specialization but for const tuple.
template <std::size_t I = 0UL, typename Callable, typename... TypesT>
COMMON_PUBLIC inline constexpr typename std::enable_if_t<I != sizeof...(TypesT)> visit(
  const std::tuple<TypesT...> & tuple, Callable callable) noexcept
{
  callable(std::get<I>(tuple));
  visit<I + 1UL, Callable, TypesT...>(tuple, callable);
}

/// @brief      A class to compute a conjunction over given traits.
template <class...>
struct COMMON_PUBLIC conjunction : std::true_type
{
};
/// @brief      A conjunction of another type shall derive from that type.
template <class TraitT>
struct COMMON_PUBLIC conjunction<TraitT> : TraitT
{
};
template <class TraitT, class... TraitsTs>
struct COMMON_PUBLIC conjunction<TraitT, TraitsTs...>
: std::conditional_t<static_cast<bool>(TraitT::value), conjunction<TraitsTs...>, TraitT>
{
};

///
/// @brief      A trait to check if a tuple has a type.
///
/// @details    Taken from https://stackoverflow.com/a/25958302/678093
///
/// @tparam     QueryT  A query type.
/// @tparam     TupleT  A tuple to search the type in.
///
template <typename QueryT, typename TupleT>
struct has_type;

///
/// @brief      An overload of the general trait that signifies that nothing can be found in an
///             empty tuple.
///
/// @tparam     QueryT     Any type.
///
template <typename QueryT>
struct has_type<QueryT, std::tuple<>> : std::false_type
{
};

///
/// @brief      Recursive override of the main trait.
///
/// @tparam     QueryT  Query type.
/// @tparam     HeadT   Head type in the tuple.
/// @tparam     TailTs  Rest of the tuple types.
///
template <typename QueryT, typename HeadT, typename... TailTs>
struct has_type<QueryT, std::tuple<HeadT, TailTs...>> : has_type<QueryT, std::tuple<TailTs...>>
{
};

///
/// @brief      End of recursion for the main `has_type` trait. Becomes a `true_type` when the first
///             type in the tuple matches the query type.
///
/// @tparam     QueryT  Query type.
/// @tparam     TailTs  Other types in the tuple.
///
template <typename QueryT, typename... TailTs>
struct has_type<QueryT, std::tuple<QueryT, TailTs...>> : std::true_type
{
};

///
/// @brief      A trait used to intersect types stored in tuples at compile time. The resulting
///             typedef `type` will hold a tuple with the intersection of the types provided in the
///             input tuples.
///
/// @details    Taken from https://stackoverflow.com/a/41200732/1763680
///
/// @tparam     TupleT1  Tuple 1
/// @tparam     TupleT2  Tuple 2
///
template <typename TupleT1, typename TupleT2>
struct intersect
{
  ///
  /// @brief      Intersect the types.
  ///
  /// @details    This function "iterates" over the types in TupleT1 and checks if those are in
  ///             TupleT2. If this is true, these types are concatenated into a new tuple.
  ///
  template <std::size_t... Indices>
  static constexpr auto make_intersection(std::index_sequence<Indices...>)
  {
    return std::tuple_cat(std::conditional_t<
                          has_type<std::tuple_element_t<Indices, TupleT1>, TupleT2>::value,
                          std::tuple<std::tuple_element_t<Indices, TupleT1>>, std::tuple<>>{}...);
  }
  /// The resulting tuple type.
  using type =
    decltype(make_intersection(std::make_index_sequence<std::tuple_size<TupleT1>::value>{}));
};

}  // namespace type_traits
}  // namespace common
}  // namespace autoware

#endif  // AUTOWARE_AUTO_COMMON__COMMON__TYPE_TRAITS_HPP_
