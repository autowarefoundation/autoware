// Copyright 2024 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef CUSTOM_CONTAINER_HPP_
#define CUSTOM_CONTAINER_HPP_

#include "material_colors.hpp"

#include <QFrame>
#include <QGridLayout>
#include <QPainter>
#include <QPainterPath>

class CustomContainer : public QFrame
{
  Q_OBJECT

public:
  explicit CustomContainer(QWidget * parent = nullptr);

  // Methods to set dimensions and corner radius
  void setContainerHeight(int height);
  void setContainerWidth(int width);

  // Getters
  int getContainerHeight() const;
  int getContainerWidth() const;
  QGridLayout * getLayout() const;  // Add a method to access the layout

protected:
  void paintEvent(QPaintEvent * event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

private:
  QGridLayout * layout;
  int cornerRadius;
};

#endif  // CUSTOM_CONTAINER_HPP_
