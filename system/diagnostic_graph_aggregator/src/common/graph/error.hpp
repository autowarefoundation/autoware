// Copyright 2023 The Autoware Contributors
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

#ifndef COMMON__GRAPH__ERROR_HPP_
#define COMMON__GRAPH__ERROR_HPP_

#include <stdexcept>

namespace diagnostic_graph_aggregator
{

struct Exception : public std::runtime_error
{
  using runtime_error::runtime_error;
};

class FileNotFound : public Exception
{
  using Exception::Exception;
};

class UnknownType : public Exception
{
  using Exception::Exception;
};

class InvalidType : public Exception
{
  using Exception::Exception;
};

class InvalidValue : public Exception
{
  using Exception::Exception;
};

class FieldNotFound : public Exception
{
  using Exception::Exception;
};

class PathConflict : public Exception
{
  using Exception::Exception;
};

class PathNotFound : public Exception
{
  using Exception::Exception;
};

class GraphStructure : public Exception
{
  using Exception::Exception;
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__ERROR_HPP_
