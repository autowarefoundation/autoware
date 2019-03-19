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

#ifndef _RVIZ_PLUGINS_FILEPROPERTY_H_
#define _RVIZ_PLUGINS_FILEPROPERTY_H_

#include <QWidget>
#include <rviz/properties/property.h>

class FileProperty : public rviz::Property {
public:
  FileProperty(const QString &name = QString(),
               const QString &description = QString(), Property *parent = 0,
               const char *changed_slot = 0, QObject *receiver = 0);

  virtual QWidget *createEditor(QWidget *parent,
                                const QStyleOptionViewItem &option);

public Q_SLOTS:
  virtual void onButtonClick();

private:
};

#endif /* _RVIZ_PLUGINS_FILEPROPERTY_H_ */
