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

#ifndef TRAFFICLIGHT_RECOGNIZER_LABEL_MAKER_LABEL_MAKER_GUI_H
#define TRAFFICLIGHT_RECOGNIZER_LABEL_MAKER_LABEL_MAKER_GUI_H

#include "trafficlight_recognizer/label_maker/file_system_operator.h"

#include <map>
#include <string>

#include <QMainWindow>
#include <QString>
#include <QLabel>
#include <QAbstractButton>

namespace Ui
{
class LabelMakerGui;
}

class LabelMakerGui : public QMainWindow
{
  Q_OBJECT

public:
  explicit LabelMakerGui(QWidget* parent = 0);
  ~LabelMakerGui();

private slots:
  // Show current ID's image
  void ShowImage();

  // Change background color to corresponding one
  void SetRadioButtonsColor(QAbstractButton* selected_button);

  // The behavior of "Next" and "Previous" button
  void SaveAndGoNext();
  void SaveAndGoPrevious();

  // The behavior of "Reset Selection" button
  void ResetSelection();

private:
  // The utility function to get directory path
  QString GetTargetDirectoryPath();

  // Reset radio buttons status
  void ResetRadioButtonsBackGround();

  // Save the current status
  bool SaveCurrentState();

  // The GUI handelr
  Ui::LabelMakerGui* ui_;

  // Path to the target dataset directory
  QString dataset_path_;

  // The list of path and ID contained in "Images" directory
  std::map<int, std::string> image_list_;

  // The class instance to operate file system
  FileSystemOperator file_system_operator_;
};

#endif  // TRAFFICLIGHT_RECOGNIZER_LABEL_MAKER_LABEL_MAKER_GUI_H
