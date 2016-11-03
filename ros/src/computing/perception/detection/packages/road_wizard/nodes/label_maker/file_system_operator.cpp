#include "file_system_operator.h"

#include <sys/stat.h>
#include <dirent.h>

#include <fstream>
#include <sstream>
#include <vector>

FileSystemOperator::FileSystemOperator():
  file_path_("") {
}


FileSystemOperator::~FileSystemOperator() {
}


// The function to return std::map structure which contains
// image ID and its filename in the specified directory path
std::map<int, std::string> FileSystemOperator::GetImageList(const std::string directory_path) {
  std::map<int, std::string> list;

  struct dirent *entry;
  DIR *directory_handler = opendir(directory_path.c_str());

  // Check all contents in the specified directory
  while ((entry = readdir(directory_handler)) != NULL) {
    // Get entry's status (file name, permission...etc)
    struct stat status;
    std::string absolute_path = directory_path + std::string(entry->d_name);
    if (stat(absolute_path.c_str(), &status) == 0 &&
        S_ISREG(status.st_mode)) { // This entry is surely nomal file

      // Get file ID by erasing file extension
      int file_id = GetFileIDFromFilePath(entry->d_name);

      // Insert file ID and file name into std::map structure
      list[file_id] = entry->d_name;
    }
  }

  closedir(directory_handler);

  return list;
}  // std::map<int, std::string> FileSystemOperation::GetImageList()


void FileSystemOperator::CheckFileExistance(std::string file_name) {
  file_path_ = file_name;

  // Check whether specified file already exist
  struct stat status;
  if (stat(file_path_.c_str(), &status) == 0 &&
      S_ISREG(status.st_mode)) { // specified file already exist
    LoadFileContents();
  }
}


void FileSystemOperator::LoadFileContents() {
  // Open the file for read
  std::ifstream file_handler_for_read;
  file_handler_for_read.open(file_path_);

  // Load file contents line by line
  std::string line_string;
  while (getline(file_handler_for_read, line_string)) {
    if (line_string.at(0) == '#') { // The line started from '#' is comment line
      continue;
    }

    // Divide each element by ','
    std::string token;
    std::vector<std::string> elements;
    std::istringstream string_stream(line_string);
    while (getline(string_stream, token, ',')) {
      elements.push_back(token);
    }

    LabelData line_data = {elements[0],
                           static_cast<LightState>(std::atoi(elements[1].c_str()))};

    // Get image ID from file name
    int image_id = GetFileIDFromFilePath(line_data.file_name);

    // Insert data into class member
    label_data_list_[image_id] = line_data;
  }

  // Close the file handler
  file_handler_for_read.close();
}


void FileSystemOperator::WriteStateToFile(std::string file_name, LightState state) {
  int image_id = GetFileIDFromFilePath(file_name);
  LabelData label_data{file_name, state};

  // Insert specified data into data list (if this ID's data already exist, it will be overwritten)
  label_data_list_[image_id] = label_data;

  // Open the file handler with overwritten mode
  std::ofstream file_handler_for_write;
  file_handler_for_write.open(file_path_, std::ios::trunc);

  // Write file format information as comment
  file_handler_for_write << "# format: \"<file_name>,<state: GREEN=0 YELLOW=1 RED=2 UNKNOWN=3>\"" << std::endl;

  // Rewrite all of the file contents
  for (const auto& data : label_data_list_) {
    file_handler_for_write << data.second.file_name << "," << data.second.state << std::endl;
  }

  file_handler_for_write.close();

}


// The utility function to get image ID from image file name
// This function assumes that image files are named by its ID like "0.png, Images/1.png,..."
int FileSystemOperator::GetFileIDFromFilePath(std::string path) {
  // Extract only file name from path
  std::string file_name = path.substr(path.find_last_of("/") + 1);

  // Remove extention
  std::string id_string = file_name.substr(0, file_name.find_last_of("."));

  // Convert string to integer and return the value
  return std::atoi(id_string.c_str());
}

