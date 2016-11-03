#ifndef FILESYSTEMOPERATOR_H
#define FILESYSTEMOPERATOR_H

#include <map>
#include <string>


class FileSystemOperator
{
public:
  // The status of traffic light color.
  // These are defined in the same order used in recognition part
  enum LightState {GREEN, YELLOW, RED, UNKNOWN};

  FileSystemOperator();
  ~FileSystemOperator();

  // The function to get all image file name in the specified directory
  std::map<int, std::string> GetImageList(const std::string diectory_path);

  // The function to check the existance of specified file
  void CheckFileExistance(std::string file_path);

  // The function to write specified state into file
  void WriteStateToFile(std::string file_name, LightState state);

private:
  // The data structure to hold label data
  struct LabelData {
    std::string file_name;
    LightState state;
  };

  // The function to load the contents of already exist file
  void LoadFileContents();

  // The utility function to get image ID from image file name
  int GetFileIDFromFilePath(std::string path);

  // The path to the target file
  std::string file_path_;

  // The data list to be written into the target file
  std::map<int, LabelData> label_data_list_;
};

#endif // FILESYSTEMOPERATOR_H
