// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "facing_visualizer.h"
#include <rviz/uniform_string_stream.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <rviz/properties/parse_color.h>
#include <QPainter>

namespace jsk_rviz_plugins
{

  const double minimum_font_size = 0.2;
  const float arrow_animation_duration = 1.0;
  
  SquareObject::SquareObject(Ogre::SceneManager* manager,
                             double outer_radius,
                             double inner_radius,
                             std::string name):
    manager_(manager), outer_radius_(outer_radius),
    inner_radius_(inner_radius), name_(name), polygon_type_(CIRCLE)
  {
    manual_ = manager->createManualObject();
    rebuildPolygon();
  }

  SquareObject::~SquareObject()
  {
    manual_->detachFromParent();
    manager_->destroyManualObject(manual_);
  }

  Ogre::ManualObject* SquareObject::getManualObject()
  {
    return manual_;
  }

  void SquareObject::setOuterRadius(double outer_radius)
  {
    outer_radius_ = outer_radius;
  }

  void SquareObject::setInnerRadius(double inner_radius)
  {
    inner_radius_ = inner_radius;
  }

  void SquareObject::rebuildPolygon()
  {
    manual_->clear();
    manual_->begin(name_,
                   Ogre::RenderOperation::OT_TRIANGLE_STRIP);
    if (polygon_type_ == CIRCLE) {
      const size_t resolution = 100;
      const double radius_ratio = inner_radius_ / outer_radius_;
      const double inner_offset = - outer_radius_ * 0.0;
      int counter = 0;
      for (size_t i = 0; i < resolution; i++) {
        double theta = 2.0 * M_PI / resolution * i;
        double next_theta = 2.0 * M_PI / resolution * (i + 1);
      
        manual_->position(inner_radius_ * cos(theta) + inner_offset,
                          inner_radius_ * sin(theta) + inner_offset,
                          0.0f);
        manual_->textureCoord((1 + radius_ratio *  cos(theta)) / 2.0,
                              (1.0 - radius_ratio * sin(theta)) / 2.0);
        manual_->index(counter++);
        manual_->position(outer_radius_ * cos(theta),
                          outer_radius_ * sin(theta),
                          0.0f);
        manual_->textureCoord((1 + cos(theta)) / 2.0,
                              (1.0 -sin(theta)) / 2.0);
        manual_->index(counter++);
        manual_->position(inner_radius_ * cos(next_theta) + inner_offset,
                          inner_radius_ * sin(next_theta) + inner_offset,
                          0.0f);
        manual_->textureCoord((1 + radius_ratio *  cos(next_theta)) / 2.0,
                              (1.0 - radius_ratio * sin(next_theta)) / 2.0);
        manual_->index(counter++);
        manual_->position(outer_radius_ * cos(next_theta),
                          outer_radius_ * sin(next_theta),
                          0.0f);
        manual_->textureCoord((1 + cos(next_theta)) / 2.0,
                              (1.0 -sin(next_theta)) / 2.0);
        manual_->index(counter++);
      
      }
    }
    else if (polygon_type_ == SQUARE) {
      manual_->position(outer_radius_, outer_radius_,
                        0.0f);     // 1
      manual_->textureCoord(0, 0); // 4
      manual_->index(0);
      
      manual_->position(-outer_radius_, outer_radius_,
                        0.0f);  // 2
      manual_->textureCoord(0, 1); // 3
      manual_->index(1);
      
      manual_->position(-outer_radius_, -outer_radius_,
                        0.0f);  // 3
      manual_->textureCoord(1, 1); // 2
      manual_->index(2);
      
      manual_->position(outer_radius_, -outer_radius_,
                        0.0f);  // 4
      manual_->textureCoord(1, 0); // 1
      manual_->index(3);
      
      manual_->position(outer_radius_, outer_radius_,
                        0.0f);  // 1
      manual_->textureCoord(0, 0); // 4
      manual_->index(4);
    }
    // for (size_t i = 0; i < resolution; i++) {
    // }
    // manual_->index(0);
    manual_->end();
  }

  void SquareObject::setPolygonType(PolygonType type)
  {
    polygon_type_ = type;
  }
  
  TextureObject::TextureObject(const int width, const int height,
                               const std::string name):
    width_(width), height_(height), name_(name)
  {
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
       width, height,
      0,
      Ogre::PF_A8R8G8B8,
      Ogre::TU_DEFAULT
      );
    material_ = Ogre::MaterialManager::getSingleton().create(
      getMaterialName(), // name
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
 
    material_->getTechnique(0)->getPass(0)->createTextureUnitState(
      texture_->getName());
    material_->setReceiveShadows(false);
    material_->getTechnique(0)->setLightingEnabled(true);
    material_->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
    material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    material_->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);
      
    material_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_->getName());
    material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

    material_->getTechnique(0)->getPass(0)
      ->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
     // material_->getTechnique(0)->getPass(0)
     //   ->setSceneBlending(Ogre::SBT_MODULATE);

  }

  TextureObject::~TextureObject()
  {
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_->getName());
  }
  
  ScopedPixelBuffer TextureObject::getBuffer()
  {
    return ScopedPixelBuffer(texture_->getBuffer());
  }

  std::string TextureObject::getMaterialName()
  {
    return name_ + "Material";
  }

  FacingObject::FacingObject(Ogre::SceneManager* manager,
                             Ogre::SceneNode* parent,
                             double size):
    scene_manager_(manager), size_(size), enable_(true), text_("")
  {
    node_ = parent->createChildSceneNode();
  }

  FacingObject::~FacingObject()
  {
    node_->detachAllObjects();
    scene_manager_->destroySceneNode(node_);
  }
  
  void FacingObject::setPosition(Ogre::Vector3& pos)
  {
    node_->setPosition(pos);
  }
  
  void FacingObject::setOrientation(rviz::DisplayContext* context)
  {
    rviz::ViewManager* manager = context->getViewManager();
    rviz::RenderPanel* panel = manager->getRenderPanel();
    Ogre::Camera* camera = panel->getCamera();
    Ogre::Quaternion q = camera->getDerivedOrientation();
    setOrientation(q);
  }

  void FacingObject::setOrientation(Ogre::Quaternion& rot)
  {
    node_->setOrientation(rot);
  }

  void FacingObject::setSize(double size)
  {
    size_ = size;
  }

  void FacingObject::setEnable(bool enable)
  {
    enable_ = enable;
    node_->setVisible(enable_);
  }

  void FacingObject::setText(std::string text)
  {
    text_ = text;
    //updateTextUnderLine();
    updateText();
  }

  void FacingObject::setAlpha(double alpha)
  {
    color_.a = alpha;
    updateColor();
  }

  void FacingObject::setColor(QColor color)
  {
    color_.r = color.red() / 255.0;
    color_.g = color.green() / 255.0;
    color_.b = color.blue() / 255.0;
    updateColor();
  }

  void FacingObject::setColor(Ogre::ColourValue color)
  {
    color_ = color;
    updateColor();
  }
  
  SimpleCircleFacingVisualizer::SimpleCircleFacingVisualizer(
    Ogre::SceneManager* manager,
    Ogre::SceneNode* parent,
    rviz::DisplayContext* context,
    double size,
    std::string text):
    FacingObject(manager, parent, size)
  {
    line_ = new rviz::BillboardLine(
      context->getSceneManager(),
      node_);
    text_under_line_ = new rviz::BillboardLine(
      context->getSceneManager(),
      node_);
    target_text_node_ = node_->createChildSceneNode();
    msg_ = new rviz::MovableText("not initialized", "Arial", 0.05);
    msg_->setVisible(false);
    msg_->setTextAlignment(rviz::MovableText::H_LEFT,
                           rviz::MovableText::V_ABOVE);
    target_text_node_->attachObject(msg_);
    createArrows(context);
    updateLine();
    updateTextUnderLine();
    updateText();
    setEnable(false);
  }

  SimpleCircleFacingVisualizer::~SimpleCircleFacingVisualizer()
  {
    delete line_;
    delete text_under_line_;
    delete msg_;
    scene_manager_->destroyManualObject(upper_arrow_);
    scene_manager_->destroyManualObject(lower_arrow_);
    scene_manager_->destroyManualObject(left_arrow_);
    scene_manager_->destroyManualObject(right_arrow_);
    upper_material_->unload();
    lower_material_->unload();
    left_material_->unload();
    right_material_->unload();
    Ogre::MaterialManager::getSingleton().remove(upper_material_->getName());
    Ogre::MaterialManager::getSingleton().remove(lower_material_->getName());
    Ogre::MaterialManager::getSingleton().remove(left_material_->getName());
    Ogre::MaterialManager::getSingleton().remove(right_material_->getName());
  }

  void SimpleCircleFacingVisualizer::update(float wall_dt, float ros_dt)
  {
    double t_ = ros::WallTime::now().toSec();
    double t_rate
      = fmod(t_, arrow_animation_duration) / arrow_animation_duration;
    upper_arrow_node_->setPosition(0, (1.3 - 0.3 * t_rate) * size_, 0);
    lower_arrow_node_->setPosition(0, (-1.3 + 0.3 * t_rate) * size_, 0);
    left_arrow_node_->setPosition((1.3 - 0.3 * t_rate) * size_, 0, 0);
    right_arrow_node_->setPosition((-1.3 + 0.3 * t_rate) * size_, 0, 0);
  }
  
  void SimpleCircleFacingVisualizer::reset()
  {
    line_->clear();
    text_under_line_->clear();
    msg_->setVisible(false);
  }

  void SimpleCircleFacingVisualizer::updateArrowsObjects(Ogre::ColourValue color)
  {
    const double size_factor = 0.15;
    upper_arrow_node_->setPosition(Ogre::Vector3(0, size_ * 1.0, 0.0));
    upper_arrow_->clear();
    upper_arrow_->estimateVertexCount(3);
    upper_arrow_->begin(upper_material_name_,
                        Ogre::RenderOperation::OT_TRIANGLE_LIST);
    
    upper_arrow_->colour(color);
    upper_arrow_->position(Ogre::Vector3(0, size_ * size_factor, 0));
    upper_arrow_->colour(color);
    upper_arrow_->position(Ogre::Vector3(size_ * size_factor,
                                         size_ * size_factor * 2,
                                         0));
    upper_arrow_->colour(color);
    upper_arrow_->position(Ogre::Vector3(-size_ * size_factor,
                                         size_ * size_factor * 2,
                                         0));
    upper_arrow_->end();
    
    lower_arrow_node_->setPosition(Ogre::Vector3(0, -size_ * 1.0, 0.0));
    lower_arrow_->clear();
    lower_arrow_->estimateVertexCount(3);
    lower_arrow_->begin(lower_material_name_,
                        Ogre::RenderOperation::OT_TRIANGLE_LIST);
    
    lower_arrow_->colour(color);
    lower_arrow_->position(Ogre::Vector3(0,
                                         -size_ * size_factor,
                                         0));
    lower_arrow_->colour(color);
    lower_arrow_->position(Ogre::Vector3(size_ * size_factor,
                                         -size_ * size_factor * 2,
                                         0));
    lower_arrow_->colour(color);
    lower_arrow_->position(Ogre::Vector3(-size_ * size_factor,
                                         -size_ * size_factor * 2,
                                         0));
    lower_arrow_->end();
    left_arrow_node_->setPosition(Ogre::Vector3(size_ * 1.0, 0.0, 0.0));
    left_arrow_->clear();
    left_arrow_->estimateVertexCount(3);
    left_arrow_->begin(left_material_name_,
                       Ogre::RenderOperation::OT_TRIANGLE_LIST);
    
    left_arrow_->colour(color);
    left_arrow_->position(Ogre::Vector3(size_ * size_factor,
                                        0.0,
                                        0));
    left_arrow_->colour(color);
    left_arrow_->position(Ogre::Vector3(size_ * size_factor * 2,
                                        size_ * size_factor,
                                        0));
    left_arrow_->colour(color);
    left_arrow_->position(Ogre::Vector3(size_ * size_factor * 2,
                                        - size_ * size_factor,
                                        0));
    left_arrow_->end();
    
    right_arrow_node_->setPosition(Ogre::Vector3(-size_ * 1.0, 0.0, 0.0));
    right_arrow_->clear();
    right_arrow_->estimateVertexCount(3);
    right_arrow_->begin(right_material_name_,
                        Ogre::RenderOperation::OT_TRIANGLE_LIST);
    
    right_arrow_->colour(color);
    right_arrow_->position(Ogre::Vector3(-size_ * size_factor,
                                         0.0,
                                         0));
    right_arrow_->colour(color);
    right_arrow_->position(Ogre::Vector3(-size_ * size_factor * 2,
                                         size_ * size_factor,
                                         0));
    right_arrow_->colour(color);
    right_arrow_->position(Ogre::Vector3(-size_ * size_factor * 2,
                                         - size_ * size_factor,
                                         0));
    right_arrow_->end();
    
    
    upper_material_->getTechnique(0)->setLightingEnabled(false);
    upper_material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    upper_material_->getTechnique(0)->setDepthWriteEnabled( false );
    lower_material_->getTechnique(0)->setLightingEnabled(false);
    lower_material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    lower_material_->getTechnique(0)->setDepthWriteEnabled( false );
    left_material_->getTechnique(0)->setLightingEnabled(false);
    left_material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    left_material_->getTechnique(0)->setDepthWriteEnabled( false );
    right_material_->getTechnique(0)->setLightingEnabled(false);
    right_material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    right_material_->getTechnique(0)->setDepthWriteEnabled( false );
  }
  
  // allocate material and node for arrrows
  void SimpleCircleFacingVisualizer::createArrows(
    rviz::DisplayContext* context)
  {
    static uint32_t count = 0;
    rviz::UniformStringStream ss;
    ss << "TargetVisualizerDisplayTriangle" << count++;
    ss << "Material";
    ss << "0";
    upper_material_name_ = std::string(ss.str());
    ss << "1";
    lower_material_name_ = std::string(ss.str());
    ss << "2";
    left_material_name_ = std::string(ss.str());
    ss << "3";
    right_material_name_ = std::string(ss.str());
    upper_material_ = Ogre::MaterialManager::getSingleton().create(
      upper_material_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    lower_material_ = Ogre::MaterialManager::getSingleton().create(
      lower_material_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    left_material_ = Ogre::MaterialManager::getSingleton().create(
      left_material_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    right_material_ = Ogre::MaterialManager::getSingleton().create(
      right_material_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    
    upper_material_->setReceiveShadows(false);
    upper_material_->getTechnique(0)->setLightingEnabled(true);
    upper_material_->setCullingMode(Ogre::CULL_NONE);
    lower_material_->setReceiveShadows(false);
    lower_material_->getTechnique(0)->setLightingEnabled(true);
    lower_material_->setCullingMode(Ogre::CULL_NONE);
    left_material_->setReceiveShadows(false);
    left_material_->getTechnique(0)->setLightingEnabled(true);
    left_material_->setCullingMode(Ogre::CULL_NONE);
    right_material_->setReceiveShadows(false);
    right_material_->getTechnique(0)->setLightingEnabled(true);
    right_material_->setCullingMode(Ogre::CULL_NONE);

    upper_arrow_ = context->getSceneManager()->createManualObject(
      upper_material_name_);
    upper_arrow_node_ = node_->createChildSceneNode();
    upper_arrow_node_->attachObject(upper_arrow_);
    lower_arrow_ = context->getSceneManager()->createManualObject(
      lower_material_name_);
    lower_arrow_node_ = node_->createChildSceneNode();
    lower_arrow_node_->attachObject(lower_arrow_);
    left_arrow_ = context->getSceneManager()->createManualObject(
      left_material_name_);
    left_arrow_node_ = node_->createChildSceneNode();
    left_arrow_node_->attachObject(left_arrow_);
    right_arrow_ = context->getSceneManager()->createManualObject(
      right_material_name_);
    right_arrow_node_ = node_->createChildSceneNode();
    right_arrow_node_->attachObject(right_arrow_);
    updateArrowsObjects(color_);
  }

  void SimpleCircleFacingVisualizer::updateLine()
  {
    const int resolution = 100;
    line_->clear();
    line_->setColor(color_.r, color_.g, color_.b, color_.a);
    line_->setLineWidth(0.1 * size_);
    line_->setNumLines(1);
    line_->setMaxPointsPerLine(1024);
    for (size_t i = 0; i < resolution + 1; i++) {
      double x = size_ * cos(i * 2 * M_PI / resolution);
      double y = size_ * sin(i * 2 * M_PI / resolution);
      double z = 0;
      Ogre::Vector3 p;
      p[0] = x;
      p[1] = y;
      p[2] = z;
      line_->addPoint(p);
    }
  }
  
  // need msg to be initialized beforehand
  void SimpleCircleFacingVisualizer::updateTextUnderLine()
  {
    Ogre::Vector3 text_position(size_ * cos(45.0 / 180.0 * M_PI)
                                + size_ / 2.0,
                                size_ * sin(45.0 / 180.0 * M_PI)
                                + size_ / 2.0,
                                0);
    target_text_node_->setPosition(text_position);
    Ogre::Vector3 msg_size = msg_->GetAABB().getSize();
    text_under_line_->clear();
    text_under_line_->setColor(color_.r, color_.g, color_.b, color_.a);
    
    text_under_line_->setLineWidth(0.01);
    text_under_line_->setNumLines(1);
    text_under_line_->setMaxPointsPerLine(1024);
    Ogre::Vector3 A(size_ * cos(45.0 / 180.0 * M_PI),
                    size_ * sin(45.0 / 180.0 * M_PI),
                    0);
    Ogre::Vector3 B(text_position + Ogre::Vector3(- size_ / 4.0, 0, 0));
    Ogre::Vector3 C(text_position + Ogre::Vector3(msg_size[0], 0, 0));
    text_under_line_->addPoint(A);
    text_under_line_->addPoint(B);
    text_under_line_->addPoint(C);
  }

  void SimpleCircleFacingVisualizer::setSize(double size)
  {
    FacingObject::setSize(size);
    updateLine();
    updateText();
    updateTextUnderLine();
  }

  void SimpleCircleFacingVisualizer::setEnable(bool enable)
  {
    FacingObject::setEnable(enable);
    msg_->setVisible(enable);
    line_->getSceneNode()->setVisible(enable);
    text_under_line_->getSceneNode()->setVisible(enable);
  }
  
  void SimpleCircleFacingVisualizer::updateText()
  {
    msg_->setCaption(text_);
    msg_->setCharacterHeight(std::max(0.15 * size_, minimum_font_size));
  }

  void SimpleCircleFacingVisualizer::setText(std::string text)
  {
    text_ = text;
    updateTextUnderLine();
    updateText();
  }

  void SimpleCircleFacingVisualizer::updateColor()
  {
    msg_->setColor(color_);
    line_->setColor(color_.r, color_.g, color_.b, color_.a);
    text_under_line_->setColor(color_.r, color_.g, color_.b, color_.a);
    updateArrowsObjects(color_);
  }
  
  FacingTexturedObject::FacingTexturedObject(Ogre::SceneManager* manager,
                                             Ogre::SceneNode* parent,
                                             double size):
    FacingObject(manager, parent, size)
  {
    rviz::UniformStringStream ss;
    static int count = 0;
    ss << "FacingVisualizer" << count++;
    texture_object_.reset(new TextureObject(128, 128, ss.str()));
    square_object_.reset(new SquareObject(manager, size, 0,
                                          texture_object_->getMaterialName()));
    node_->attachObject(square_object_->getManualObject());
  }
  

  void FacingTexturedObject::setSize(double size)
  {
    FacingObject::setSize(size);
    square_object_->setOuterRadius(size_);
    square_object_->rebuildPolygon();
  }

  GISCircleVisualizer::GISCircleVisualizer(Ogre::SceneManager* manager,
                                           Ogre::SceneNode* parent,
                                           double size,
                                           std::string text):
    FacingTexturedObject(manager, parent, size), text_(text)
  {

  }
  
  void GISCircleVisualizer::update(float wall_dt, float ros_dt)
  {
    ros::WallTime now = ros::WallTime::now();
    std::string text = text_ + " ";
    {
      ScopedPixelBuffer buffer = texture_object_->getBuffer();
      QColor transparent(0, 0, 0, 0);
      QColor foreground = rviz::ogreToQt(color_);
      QColor white(255, 255, 255, color_.a * 255);
      QImage Hud = buffer.getQImage(128, 128, transparent);
      double line_width = 5;
      double inner_line_width = 10;
      double l = 128;
      //double cx = l / 2 - line_width / 4.0;
      double cx = l / 2;
      //double cy = l / 2 - line_width / 4.0;
      double cy = l / 2;
      double r = 48;
      double inner_r = 40;
      double mouse_r = 30;
      double mouse_cy_offset = 5;
      
      QPainter painter( &Hud );
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setPen(QPen(foreground, line_width, Qt::SolidLine));
      painter.setBrush(white);
      painter.drawEllipse(line_width / 2.0, line_width / 2.0,
                          l - line_width, l - line_width);
      double offset_rate = fmod(now.toSec(), 10) / 10.0;
      double theta_offset = offset_rate * M_PI * 2.0;
      for (size_t ci = 0; ci < text.length(); ci++) {
        double theta = M_PI * 2.0 / text.length() * ci + theta_offset;
        painter.save();
        QFont font("DejaVu Sans Mono");
        font.setPointSize(8);
        font.setBold(false);
        painter.setFont(font);
        painter.translate(cx + r * cos(theta),
                          cy + r * sin(theta));
        painter.rotate(theta / M_PI * 180 + 90);
        painter.drawText(0, 0, text.substr(ci, 1).c_str());
        painter.restore();
      }
      painter.setPen(QPen(foreground, inner_line_width, Qt::SolidLine));
      painter.setBrush(transparent);
      painter.drawEllipse(cx - inner_r, cy - inner_r,
                          inner_r * 2, inner_r * 2);
      double mouse_c_x = cx;
      double mouse_c_y = cy - mouse_cy_offset;
      double start_angle = -25 * M_PI / 180;
      double end_angle = -125 * M_PI / 180;
      painter.setPen(QPen(foreground, line_width, Qt::SolidLine));
      painter.drawChord(mouse_c_x - mouse_r, mouse_c_y - mouse_r,
                        2.0 * mouse_r, 2.0 * mouse_r,
                        start_angle * 180 / M_PI * 16,
                        end_angle * 180 / M_PI * 16);
      painter.end();
    }
  }

  void GISCircleVisualizer::setAnonymous(bool anonymous)
  {
    anonymous_ = anonymous;
    if (!anonymous_) {
      square_object_->setInnerRadius(size_ * 0.6);
    }
    else {
      square_object_->setInnerRadius(0.0);
      
    }
    square_object_->rebuildPolygon();
  }

  void GISCircleVisualizer::setSize(double size)
  {
    FacingObject::setSize(size);
    square_object_->setOuterRadius(size_);
    if (!anonymous_) {
      square_object_->setInnerRadius(size_ * 0.6);
    }
    else {
      square_object_->setInnerRadius(0.0);
    }
    square_object_->rebuildPolygon();
  }

  
}
