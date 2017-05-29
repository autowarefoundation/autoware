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

#include "pictogram_display.h"
#include <QPainter>
#include <QFontDatabase>
#include <ros/package.h>

////////////////////////////////////////////////////////
// read Entypo fonts
// http://mempko.wordpress.com/2014/11/28/using-entypo-fonts-as-icons-in-your-qt-application/
////////////////////////////////////////////////////////

#include "Entypo.dat"
#include "Entypo_Social.dat"
#include "fontawesome.dat"
#include "pictogram_font_mapping.h"

namespace jsk_rviz_plugins
{

  int addFont(unsigned char* data, unsigned int data_len)
  {
    // register font
    QByteArray entypo =
      QByteArray::fromRawData(
        reinterpret_cast<const char*>(data), data_len);
    int id =
      QFontDatabase::addApplicationFontFromData(entypo);
    if (id == -1) {
      ROS_WARN("failed to load font");
    }
    else {
      return id;
    }
  }  
  
  bool epsEqual(double a, double b)
  {
    return (std::abs(a - b) < 0.01);
  }
  
  bool isCharacterSupported(std::string character)
  {
    return ((entypo_social_character_map.find(character)
             != entypo_social_character_map.end()) ||
            (entypo_character_map.find(character)
             != entypo_character_map.end()) ||
            (fontawesome_character_map.find(character)
             != fontawesome_character_map.end()));
  }
  
  QFont getFont(std::string character)
  {
    if (entypo_social_character_map.find(character)
        != entypo_social_character_map.end()) {
      return QFont("Entypo Social");
    }
    else if (entypo_character_map.find(character)
             != entypo_character_map.end()) {
      return QFont("Entypo");
    }
    else {
      return QFont("FontAwesome");
    }
  }
  
  QString lookupPictogramText(std::string character)
  {
    if (entypo_social_character_map.find(character)
        != entypo_social_character_map.end()) {
      return entypo_social_character_map[character];
    }
    else if (entypo_character_map.find(character)
             != entypo_character_map.end()){
      return entypo_character_map[character];
    }
    else {
      return fontawesome_character_map[character];
    }
  }

  bool isEntypo(std::string text) {
    return ((entypo_social_character_map.find(text)
             != entypo_social_character_map.end()) ||
            (entypo_character_map.find(text)
             != entypo_character_map.end()));
  }

  bool isFontAwesome(std::string text) {
    return (fontawesome_character_map.find(text)
            != fontawesome_character_map.end());
  }

  
  
  PictogramObject::PictogramObject(Ogre::SceneManager* manager,
                                   Ogre::SceneNode* parent,
                                   double size):
    FacingTexturedObject(manager, parent, size),
    need_to_update_(false),
    action_(jsk_rviz_plugins::Pictogram::ADD)
  {
    square_object_->setPolygonType(SquareObject::SQUARE);
    square_object_->rebuildPolygon();
    
    // for (std::map<std::string, QString>::iterator it = fontawesome_character_map.begin();
    //      it != fontawesome_character_map.end();
    //      ++it) {
    //   ROS_INFO("%s", it->first.c_str());
    // }
  }

  void PictogramObject::setEnable(bool enable)
  {
    if (enable && !enable_) {
      need_to_update_ = true;
    }
    FacingTexturedObject::setEnable(enable);
  }

  void PictogramObject::start()
  {
    time_ = ros::WallTime::now();
  }

  void PictogramObject::setSize(double size)
  {
    if (size_ != size) {
      need_to_update_ = true;
      FacingTexturedObject::setSize(size);
    }
  }

  void PictogramObject::setSpeed(double speed)
  {
    speed_ = speed;
  }

  void PictogramObject::setPose(const geometry_msgs::Pose& pose,
                                const std::string& frame_id)
  {
    pose_ = pose;
    frame_id_ = frame_id;
  }
  
  void PictogramObject::setContext(rviz::DisplayContext* context)
  {
    context_ = context;
  }

  void PictogramObject::setMode(uint8_t mode)
  {
    mode_ = mode;
  }

  void PictogramObject::setTTL(double ttl)
  {
    ttl_ = ttl;
  }

  void PictogramObject::setAction(uint8_t type)
  {
    action_ = type;
    if (action_ == jsk_rviz_plugins::Pictogram::DELETE) {
      setEnable(false);
    }
    else{
      start();
    }
  }

  void PictogramObject::updatePose(float wall_dt)
  {
    Ogre::Vector3 position;
    Ogre::Quaternion quaternion;
    std_msgs::Header header;
    header.frame_id = frame_id_;
    if(!context_->getFrameManager()->transform(header,
                                               pose_,
                                               position,
                                               quaternion)) {
      ROS_ERROR( "Error transforming pose from frame '%s'",
                 frame_id_.c_str());
      return;
    }

    if (action_ == jsk_rviz_plugins::Pictogram::ADD) {
      setPosition(position);
      setOrientation(quaternion);
    }
    else if (action_ == jsk_rviz_plugins::Pictogram::ROTATE_Z ||
             action_ == jsk_rviz_plugins::Pictogram::ROTATE_X ||
             action_ == jsk_rviz_plugins::Pictogram::ROTATE_Y) {
      Ogre::Vector3 axis;
      if (action_ == jsk_rviz_plugins::Pictogram::ROTATE_Z) {
        axis = Ogre::Vector3(0, 0, 1);
      }
      else if (action_ == jsk_rviz_plugins::Pictogram::ROTATE_X) {
        axis = Ogre::Vector3(1, 0, 0);
      }
      else if (action_ == jsk_rviz_plugins::Pictogram::ROTATE_Y) {
        axis = Ogre::Vector3(0, 1, 0);
      }
      time_ = time_ + ros::WallDuration(wall_dt);
      // time_ -> theta
      Ogre::Radian theta(M_PI * 2 * fmod(time_.toSec() * speed_, 1.0));
      
      Ogre::Quaternion offset;
      offset.FromAngleAxis(theta, axis);
      Ogre::Quaternion final_rot = quaternion * offset;
      setPosition(position);
      setOrientation(final_rot);
    }
    else if (action_ == jsk_rviz_plugins::Pictogram::JUMP ||
             action_ == jsk_rviz_plugins::Pictogram::JUMP_ONCE) {
      bool jumpingp = false;
      if (action_ == jsk_rviz_plugins::Pictogram::JUMP) {
        jumpingp = true;
      }
      else if (action_ == jsk_rviz_plugins::Pictogram::JUMP_ONCE &&
               (ros::WallTime::now() - time_).toSec() < 2) {
        jumpingp = true;
      }
      
      if (!jumpingp) {
        setPosition(position);
      }
      else {
        // t(2-t) * size
        double t = fmod((ros::WallTime::now() - time_).toSec(), 2.0);
        double height = size_ * t * (2 - t);
        Ogre::Vector3 new_pos = position + quaternion * Ogre::Vector3(height, 0, 0);
        setPosition(new_pos);
      }
      setOrientation(quaternion);
    }

    double exceeded_time;
    if( ttl_ && (exceeded_time = (ros::WallTime::now() - time_).toSec()) > ttl_) {
      setAlpha( std::max(1.0 - 1.0 * (ros::WallTime::now() - (time_ + ros::WallDuration(ttl_))).toSec() / 5.0, 0.0) );
      if( 1.0 - 1.0 * (ros::WallTime::now() - (time_ + ros::WallDuration(ttl_))).toSec() / 3.0 < 0)
	setAction(jsk_rviz_plugins::Pictogram::DELETE);
    }
  }
  
  void PictogramObject::update(float wall_dt, float ros_dt)
  {
    if (text_.empty()) {
      // not yet setted
      return;
    }
    // update position and orientation
    if (!context_) {
      return;
    }
    updatePose(wall_dt);
    if (!need_to_update_) {
      return;
    }
    need_to_update_ = false;
    ScopedPixelBuffer buffer = texture_object_->getBuffer();
    QColor transparent(255, 255, 255, 0);
    QImage Hud = buffer.getQImage(128, 128, transparent); // should change according to size
    QPainter painter( &Hud );
    painter.setRenderHint(QPainter::Antialiasing, true);
    QColor foreground = rviz::ogreToQt(color_);
    painter.setPen(QPen(foreground, 5, Qt::SolidLine));
    
    if (isCharacterSupported(text_) && mode_ == jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE) {
      QFont font = getFont(text_);
      QString pictogram_text = lookupPictogramText(text_);
      if (isEntypo(text_)) {
        font.setPointSize(100);
      }
      else if (isFontAwesome(text_)) {
        font.setPointSize(45);
      }
      painter.setFont(font);
      painter.drawText(0, 0, 128, 128,
                       Qt::AlignHCenter | Qt::AlignVCenter,
                       pictogram_text);
      painter.end();
    }else if( mode_ == jsk_rviz_plugins::Pictogram::STRING_MODE){
      QFont font("Arial");
      font.setPointSize(32);
      font.setBold(true);
      painter.setFont(font);
      painter.drawText(0, 0, 128, 128,
		       Qt::TextWordWrap | Qt::AlignHCenter | Qt::AlignVCenter,
		       text_.c_str());
      painter.end();
    }
    else {
      ROS_WARN("%s is not supported", text_.c_str());
    }
  }

  void PictogramObject::updateColor()
  {
  }
  
  void PictogramObject::updateText()
  {
  }

  void PictogramObject::setColor(QColor color)
  {
    if (!epsEqual(color_.r * 255.0, color.red()) ||
        !epsEqual(color_.g * 255.0, color.green()) ||
        !epsEqual(color_.b * 255.0, color.blue())) {
      FacingTexturedObject::setColor(color);
      need_to_update_ = true;
    }
  }

  void PictogramObject::setText(std::string text)
  {
    if (text_ != text) {
      FacingTexturedObject::setText(text);
      need_to_update_ = true;
    }
  }
  
  void PictogramObject::setAlpha(double alpha)
  {
    if (!epsEqual(color_.a, alpha)) {
      need_to_update_ = true;
      FacingTexturedObject::setAlpha(alpha);
    }
  }
  
  PictogramDisplay::PictogramDisplay()
  {
    setupFont();
  }

  PictogramDisplay::~PictogramDisplay()
  {
    
  }
  
  void PictogramDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    pictogram_.reset(new PictogramObject(scene_manager_,
                                         scene_node_,
                                         1.0));

    pictogram_->setContext(context_);
    pictogram_->setEnable(false);
    pictogram_->start();
    // initial setting
    pictogram_->setColor(QColor(25, 255, 240));
    pictogram_->setAlpha(1.0);
    pictogram_->setSpeed(1.0);
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  }

  void PictogramDisplay::reset()
  {
    MFDClass::reset();
    pictogram_->setEnable(false);
  }

  void PictogramDisplay::onEnable()
  {
    subscribe();
    if (pictogram_) {
      // keep false, it will be true
      // in side of processMessae callback.
      pictogram_->setEnable(false);
    }
  }

  void PictogramDisplay::processMessage(const jsk_rviz_plugins::Pictogram::ConstPtr& msg)
  {
    boost::mutex::scoped_lock (mutex_);

    pictogram_->setEnable(isEnabled());
    if (!isEnabled()) {
      return;
    }
    pictogram_->setAction(msg->action);
    if (msg->action == jsk_rviz_plugins::Pictogram::DELETE) {
      return;
    }
    
    if (msg->size <= 0.0) {
      pictogram_->setSize(0.5);
    }
    else {
      pictogram_->setSize(msg->size / 2.0);
    }
    pictogram_->setColor(QColor(msg->color.r * 255,
                                msg->color.g * 255,
                                msg->color.b * 255));
    pictogram_->setAlpha(msg->color.a);
    pictogram_->setPose(msg->pose, msg->header.frame_id);
    pictogram_->setText(msg->character);
    pictogram_->setMode(msg->mode);
    pictogram_->setTTL(msg->ttl);
    if (msg->speed)
      pictogram_->setSpeed(msg->speed);
  }

  void PictogramDisplay::update(float wall_dt, float ros_dt)
  {
    boost::mutex::scoped_lock (mutex_);
    if (pictogram_) {
      pictogram_->update(wall_dt, ros_dt);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_rviz_plugins::PictogramDisplay, rviz::Display);
