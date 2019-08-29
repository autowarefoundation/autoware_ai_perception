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

#include "trafficlight_recognizer/tlr_tuner/tuner_body.h"
#include "trafficlight_recognizer/tlr_tuner/mainwindow.h"
#include "ui_mainwindow.h"  // NOLINT(build/include)

#include <string>

#include <QtCore/QString>
#include <QFileDialog>
#include <QMessageBox>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
  /* tuner controller setup */
  ui->setupUi(this);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_radioButton_green_clicked()
{
  TunerBody::setColor(TunerBody::GREEN);
  return;
}

void MainWindow::on_radioButton_yellow_clicked()
{
  TunerBody::setColor(TunerBody::YELLOW);
  return;
}

void MainWindow::on_radioButton_red_clicked()
{
  TunerBody::setColor(TunerBody::RED);
  return;
}

void MainWindow::on_pushButton_reloadImage_clicked()
{
  TunerBody::setUpdateImage();
  return;
}

void MainWindow::on_pushButton_save_clicked()
{
  QString fileName =
      QFileDialog::getSaveFileName(this, tr("Save tuning result"), "", tr("yaml file (*.yaml);;All Files(*)"));
  std::string save_filePath(fileName.toLatin1());

  if (!save_filePath.empty())
  {
    TunerBody::saveResult(save_filePath);

    /* show notification */
    QMessageBox msgBox(this);
    QString message = tr("<font color = white> <p>tuning result was saved into </p> <p>\"%1\"</p> "
                         "<p>successfully</p></font>")
                          .arg(fileName);
    msgBox.setWindowTitle(tr("Message"));
    msgBox.setText(message);
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.exec();
  }
}

void MainWindow::on_pushButton_loadSetting_clicked()
{
  QString fileName =
      QFileDialog::getOpenFileName(this, tr("Open setting file"), "", tr("yaml file (*.yaml);;All Files(*)"));
  std::string open_filePath(fileName.toLatin1());

  if (!open_filePath.empty())
  {
    TunerBody::openSetting(open_filePath);
  }
}

void MainWindow::on_pushButton_exit_clicked()
{
  exit(EXIT_SUCCESS);
}
