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

#include "trafficlight_recognizer/tlr_tuner/mainwindow.h"
#include "trafficlight_recognizer/tlr_tuner/tuner_body.h"

#include <ros/ros.h>

#include <QApplication>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tlr_tuner");

  QApplication a(argc, argv);
  MainWindow w;
  TunerBody tuner;

  w.show();
  tuner.launch();

  return a.exec();
}
