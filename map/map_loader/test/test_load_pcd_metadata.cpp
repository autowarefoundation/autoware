// Copyright 2022 The Autoware Contributors
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

#include "../src/pointcloud_map_loader/utils.hpp"

#include <gmock/gmock.h>

#include <filesystem>
#include <fstream>

using ::testing::ContainerEq;

std::string createYAMLFile()
{
  std::filesystem::path tmp_path = std::filesystem::temp_directory_path() / "temp_metadata.yaml";

  std::ofstream ofs(tmp_path);
  ofs << "file1.pcd: [1, 2]\n";
  ofs << "file2.pcd: [3, 4]\n";
  ofs << "x_resolution: 5\n";
  ofs << "y_resolution: 6\n";
  ofs.close();

  return tmp_path.string();
}

TEST(LoadPCDMetadataTest, BasicFunctionality)
{
  std::string yaml_file_path = createYAMLFile();

  std::map<std::string, PCDFileMetadata> expected = {
    {"file1.pcd", {{1, 2, 0}, {6, 8, 0}}},
    {"file2.pcd", {{3, 4, 0}, {8, 10, 0}}},
  };

  auto result = loadPCDMetadata(yaml_file_path);
  ASSERT_THAT(result, ContainerEq(expected));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
