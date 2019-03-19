/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 *  Created on: Oct 4, 2018
 *      Author: sujiwo
 */

#include "FileProperty.h"
#include <QFileDialog>
#include <iostream>
#include <rviz/properties/line_edit_with_button.h>
#include <sstream>

using namespace std;

FileProperty::FileProperty(const QString &name, const QString &description,
                           Property *parent, const char *changed_slot,
                           QObject *receiver)
    :

      rviz::Property(name, QVariant(), description, parent, changed_slot,
                     receiver) {}

QWidget *FileProperty::createEditor(QWidget *parent,
                                    const QStyleOptionViewItem &option) {
  cout << "Test\n";
  rviz::LineEditWithButton *wEdit = new rviz::LineEditWithButton(parent);
  wEdit->setFrame(false);
  QPushButton *b = wEdit->button();
  //	connect(b, SIGNAL())
  return wEdit;
}

void FileProperty::onButtonClick() {}
