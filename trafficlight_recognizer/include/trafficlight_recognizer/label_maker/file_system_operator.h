/*
 * Copyright 2019 Autoware Foundation
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TRAFFICLIGHT_RECOGNIZER_LABEL_MAKER_FILE_SYSTEM_OPERATOR_H
#define TRAFFICLIGHT_RECOGNIZER_LABEL_MAKER_FILE_SYSTEM_OPERATOR_H

#include <map>
#include <string>

class FileSystemOperator
{
public:
  // The status of traffic light color.
  // These are defined in the same order used in recognition part
  enum LightState
  {
    GREEN,
    YELLOW,
    RED,
    UNKNOWN
  };

  FileSystemOperator();
  ~FileSystemOperator();

  // The function to get all image file name in the specified directory
  std::map<int, std::string> GetImageList(const std::string diectory_path);

  // The function to check the presaved annotation files
  void CheckPreSavedData(std::string target_dir_name);

  // The function to write specified state into file
  void WriteStateToFile(std::string folder_name, std::string file_name, LightState state, int image_height,
                        int image_width, int image_depth, int x_start, int y_start, int x_end, int y_end);

private:
  // The data structure to hold label data
  struct LabelData
  {
    std::string folder_name;
    std::string file_name;
    LightState state;
    int height;
    int width;
    int depth;
    int x_start;
    int y_start;
    int x_end;
    int y_end;
  };

  // The function to load the contents of already exist annotation file
  void LoadPreSavedContents();

  // The utility function to get image ID from image file name
  int GetFileIDFromFilePath(std::string path);

  // The path for a directory that annotation files to be saved
  std::string target_directory_path_;

  // The data list to be written into the target file
  std::map<int, LabelData> label_data_list_;
};

#endif  // TRAFFICLIGHT_RECOGNIZER_LABEL_MAKER_FILE_SYSTEM_OPERATOR_H
