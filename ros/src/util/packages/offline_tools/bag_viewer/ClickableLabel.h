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
 *  Created on: Dec 25, 2018
 *      Author: sujiwo
 */

#ifndef _CLICKABLELABEL_H_
#define _CLICKABLELABEL_H_

#include <QLabel>
#include <QWidget>
#include <Qt>

class ClickableLabel : public QLabel {
  Q_OBJECT

public:
  explicit ClickableLabel(QWidget *parent = Q_NULLPTR,
                          Qt::WindowFlags f = Qt::WindowFlags());
  explicit ClickableLabel(const QString &text, QWidget *parent = nullptr,
                          Qt::WindowFlags f = Qt::WindowFlags());
  ~ClickableLabel();

signals:
  void clicked();

protected:
  void mousePressEvent(QMouseEvent *event);
};

#endif /* _CLICKABLELABEL_H_ */
