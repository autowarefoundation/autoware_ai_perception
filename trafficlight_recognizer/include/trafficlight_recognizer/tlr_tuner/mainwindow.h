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

#ifndef TRAFFICLIGHT_RECOGNIZER_TLR_TUNER_MAINWINDOW_H
#define TRAFFICLIGHT_RECOGNIZER_TLR_TUNER_MAINWINDOW_H

#include <QMainWindow>

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget* parent = 0);
  ~MainWindow();

private:
  Ui::MainWindow* ui;

private slots:
  void on_radioButton_green_clicked();
  void on_radioButton_yellow_clicked();
  void on_radioButton_red_clicked();
  void on_pushButton_reloadImage_clicked();
  void on_pushButton_save_clicked();
  void on_pushButton_loadSetting_clicked();
  void on_pushButton_exit_clicked();
};

#endif  // TRAFFICLIGHT_RECOGNIZER_TLR_TUNER_MAINWINDOW_H
