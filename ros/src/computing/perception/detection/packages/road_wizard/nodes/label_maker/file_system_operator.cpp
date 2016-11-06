#include "file_system_operator.h"

#include <sys/stat.h>
#include <dirent.h>

#include <fstream>
#include <sstream>
#include <vector>

#include <tinyxml.h>

FileSystemOperator::FileSystemOperator():
  target_directory_path_("") {
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


void FileSystemOperator::CheckPreSavedData(std::string target_dir_name) {
  target_directory_path_ = target_dir_name;

  // Check whether specified directory already exist
  struct stat directory_status;
  if (stat(target_directory_path_.c_str(), &directory_status) == 0) {
    // Specified directory already exist
    LoadPreSavedContents();
  } else {
    // Newly create directory
    mkdir(target_directory_path_.c_str(), 0755);
  }
}


void FileSystemOperator::LoadPreSavedContents() {
  struct dirent *entry;
  DIR *directory_handler = opendir(target_directory_path_.c_str());

  // Load all annotation file data in the specified directory
  while ((entry = readdir(directory_handler)) != NULL) {
    // Get entry's status (file name, permission...etc)
    struct stat status;
    std::string absolute_path = target_directory_path_ + std::string(entry->d_name);

    if (stat(absolute_path.c_str(), &status) == 0 &&
        S_ISREG(status.st_mode)) { // This entry is surely nomal file
      LabelData loaded_data;

      // Open this xml file
      TiXmlDocument xml_document(absolute_path);
      xml_document.LoadFile();

      // Parse its contents and insert into the structure
      TiXmlElement* root = xml_document.FirstChildElement("annotation");

      TiXmlElement* folder = root->FirstChildElement("folder");
      loaded_data.folder_name = std::string(folder->GetText());

      TiXmlElement* file = root->FirstChildElement("filename");
      loaded_data.file_name = std::string(file->GetText());

      TiXmlElement* object = root->FirstChildElement("object");
      TiXmlElement* name = object->FirstChildElement("name");
      loaded_data.state = static_cast<LightState>(std::atoi(name->GetText()));

      TiXmlElement* bounding_box = object->FirstChildElement("bndbox");
      TiXmlElement* x_min = bounding_box->FirstChildElement("xmin");
      TiXmlElement* y_min = bounding_box->FirstChildElement("ymin");
      TiXmlElement* x_max = bounding_box->FirstChildElement("xmax");
      TiXmlElement* y_max = bounding_box->FirstChildElement("ymax");
      loaded_data.x_start = std::atoi(x_min->GetText());
      loaded_data.y_start = std::atoi(y_min->GetText());
      loaded_data.x_end = std::atoi(x_max->GetText());
      loaded_data.y_end = std::atoi(y_max->GetText());

      // Insert loaded data into list
      int file_id = GetFileIDFromFilePath(entry->d_name);
      label_data_list_[file_id] = loaded_data;
    }
  }
}


void FileSystemOperator::WriteStateToFile(std::string folder_name,
                                          std::string file_name,
                                          LightState state,
                                          int x_start,
                                          int y_start,
                                          int x_end,
                                          int y_end) {
  int image_id = GetFileIDFromFilePath(file_name);
  LabelData label_data{folder_name,
        file_name,
        state,
        x_start,
        y_start,
        x_end,
        y_end};

  // Insert specified data into data list (if this ID's data already exist, it will be overwritten)
  label_data_list_[image_id] = label_data;

  // Create XML data
  TiXmlDocument xml_data;

  TiXmlElement* root = new TiXmlElement("annotation");
  xml_data.LinkEndChild(root);

  TiXmlElement* folder = new TiXmlElement("folder");
  folder->LinkEndChild(new TiXmlText(label_data_list_[image_id].folder_name));
  root->LinkEndChild(folder);

  TiXmlElement* file = new TiXmlElement("filename");
  file->LinkEndChild(new TiXmlText(label_data_list_[image_id].file_name));
  root->LinkEndChild(file);

  TiXmlElement* object = new TiXmlElement("object");
  TiXmlElement* name = new TiXmlElement("name");
  name->LinkEndChild(new TiXmlText(std::to_string(label_data_list_[image_id].state)));
  object->LinkEndChild(name);

  TiXmlElement* bounding_box = new TiXmlElement("bndbox");
  TiXmlElement* x_min = new TiXmlElement("xmin");
  x_min->LinkEndChild(new TiXmlText(std::to_string(label_data_list_[image_id].x_start)));
  TiXmlElement* y_min = new TiXmlElement("ymin");
  y_min->LinkEndChild(new TiXmlText(std::to_string(label_data_list_[image_id].y_start)));
  TiXmlElement* x_max = new TiXmlElement("xmax");
  x_max->LinkEndChild(new TiXmlText(std::to_string(label_data_list_[image_id].x_end)));
  TiXmlElement* y_max = new TiXmlElement("ymax");
  y_max->LinkEndChild(new TiXmlText(std::to_string(label_data_list_[image_id].y_end)));
  bounding_box->LinkEndChild(x_min);
  bounding_box->LinkEndChild(y_min);
  bounding_box->LinkEndChild(x_max);
  bounding_box->LinkEndChild(y_max);

  object->LinkEndChild(bounding_box);

  root->LinkEndChild(object);

  // Save XML data
  std::string xml_file_name = target_directory_path_ + std::to_string(image_id) + ".xml";
  xml_data.SaveFile(xml_file_name);
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

