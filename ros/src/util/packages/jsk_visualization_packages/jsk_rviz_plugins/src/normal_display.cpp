// -*- mode: C++ -*-
#include "normal_display.h"

using namespace rviz;

namespace jsk_rviz_plugins
{

  NormalDisplay::NormalDisplay():skip_rate_(1),scale_(0.3),alpha_(1.0)
  {
    skip_rate_property_
      = new FloatProperty("Display Rate (%)", 1,
                                            "Skip the display normals for speed up. Around 1% is recommended",
                                            this, SLOT( updateSkipRate() ));
    skip_rate_property_->setMax(100.0);
    skip_rate_property_->setMin(  0.0);

    scale_property_
      = new rviz::FloatProperty("Scale", 0.3,
                                "set the scale of arrow",
                                this, SLOT(updateScale()));

    scale_property_->setMin(0.0);

    alpha_property_
      = new rviz::FloatProperty("Alpha", 1,
                                "set the alpha of arrow",
                                this, SLOT(updateAlpha()));

    alpha_property_->setMax(1.0);
    alpha_property_->setMin(0.0);

    style_property_ = new EnumProperty( "Style", "PointsColor",
                                        "Rendering mode to use, in order of computational complexity.",
                                        this, SLOT( updateStyle() ), this);
    style_property_->addOption( "PointsColor", NormalDisplay::POINTS_COLOR );
    style_property_->addOption( "FlatColor", NormalDisplay::FLAT_COLOR );
    style_property_->addOption( "DirectionColor", NormalDisplay::DIRECTION_COLOR );
    style_property_->addOption( "CurvatureColor", NormalDisplay::CURVATURE_COLOR );

    color_property_ = new ColorProperty( "Color", Qt::white,
                                         "Color to assign to every point.",this);
    color_property_->hide();

    rainbow_property_ = new BoolProperty( "Use Rainbow", true, "Set rainbow range", this, SLOT( updateRainbow() ), this);
    rainbow_property_->hide();

    min_color_property_ = new ColorProperty( "MinColor", Qt::green,
                                         "Min color.",this);
    min_color_property_->hide();
    max_color_property_ = new ColorProperty( "Max Color", Qt::red,
                                             "Max color.",this);
    max_color_property_->hide();
  }

  void NormalDisplay::getRainbow(float value , float& rf, float& gf, float& bf){
    value = std::min(value, 1.0f);
    value = std::max(value, 0.0f);
    float h = value * 5.0f + 1.0f;
    int i = floor(h);
    float f = h - i;
    if ( !(i&1) ) f = 1 - f;
    float n = 1 - f;
    if      (i <= 1) rf = n, gf = 0, bf = 1;
    else if (i == 2) rf = 0, gf = n, bf = 1;
    else if (i == 3) rf = 0, gf = 1, bf = n;
    else if (i == 4) rf = n, gf = 1, bf = 0;
    else if (i >= 5) rf = 1, gf = n, bf = 0;
  }

  void NormalDisplay::updateRainbow(){
    if(rainbow_property_->getBool()){
      min_color_property_->hide();
      max_color_property_->hide();
    }else{
      min_color_property_->show();
      max_color_property_->show();
    }
  }

  void NormalDisplay::updateScale(){
    scale_ = scale_property_->getFloat();
  };

  void NormalDisplay::updateAlpha(){
    alpha_ = alpha_property_->getFloat();
  };

  void NormalDisplay::updateSkipRate(){
    skip_rate_ = skip_rate_property_->getFloat();
  }

  void NormalDisplay::updateStyle()
  {
    NormalDisplay::ColorTypes mode = (NormalDisplay::ColorTypes) style_property_->getOptionInt();
    if( mode != NormalDisplay::FLAT_COLOR )
      {
        color_property_->hide();
      }
    else
      {
        color_property_->show();
      }

    if( mode != NormalDisplay::CURVATURE_COLOR)
      {
        min_color_property_->hide();
        max_color_property_->hide();
        rainbow_property_->hide();
      }
    else
      {
        rainbow_property_->show();
        if(rainbow_property_->getBool()){
          min_color_property_->hide();
          max_color_property_->hide();
        }else{
          min_color_property_->show();
          max_color_property_->show();
        }
      }
  }

  void NormalDisplay::onInitialize()
  {
    MFDClass::onInitialize();
  }

  NormalDisplay::~NormalDisplay()
  {
    delete style_property_;
    delete color_property_;
    visuals_.clear();
  }

  void NormalDisplay::reset()
  {
    MFDClass::reset();
    visuals_.clear();
  }


  void NormalDisplay::processMessage( const sensor_msgs::PointCloud2::ConstPtr& msg )
  {
    //check x,y,z
    int32_t xi = findChannelIndex(msg, "x");
    int32_t yi = findChannelIndex(msg, "y");
    int32_t zi = findChannelIndex(msg, "z");

    if (xi == -1 || yi == -1 || zi == -1)
      {
        ROS_ERROR("doesn't have x, y, z");
        return;
      }

    const uint32_t xoff = msg->fields[xi].offset;
    const uint32_t yoff = msg->fields[yi].offset;
    const uint32_t zoff = msg->fields[zi].offset;

    //check normals x,y,z
    int32_t normal_xi = findChannelIndex(msg, "normal_x");
    int32_t normal_yi = findChannelIndex(msg, "normal_y");
    int32_t normal_zi = findChannelIndex(msg, "normal_z");
    int32_t curvature_i = findChannelIndex(msg, "curvature");

    if (normal_xi == -1 || normal_yi == -1 || normal_zi == -1 || curvature_i == -1)
      {
        ROS_ERROR("doesn't have normal_x, normal_y, normal_z, curvature");
        return;
      }

    const uint32_t normal_xoff = msg->fields[normal_xi].offset;
    const uint32_t normal_yoff = msg->fields[normal_yi].offset;
    const uint32_t normal_zoff = msg->fields[normal_zi].offset;
    const uint32_t curvature_off = msg->fields[curvature_i].offset;

    //check rgba color
    int32_t rgbai = findChannelIndex(msg, "rgb");
    uint32_t rgbaoff = -1;
    if(rgbai != -1)
      rgbaoff = msg->fields[rgbai].offset;

    //check other option values
    const uint32_t point_step = msg->point_step;
    const size_t point_count = msg->width * msg->height;

    if (point_count == 0)
      {
        ROS_ERROR("doesn't have point_count > 0");
        return;
      }

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                    msg->header.stamp,
                                                    position, orientation ))
      {
        ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                   msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
        return;
      }

    int skip_time = int( 100 / skip_rate_ );
    skip_time = std::max(skip_time, 1);
    skip_time = std::min(skip_time, int(point_count / 2));
    visuals_.rset_capacity(int(point_count/skip_time));
    const uint8_t* ptr = &msg->data.front();

    //Use Prev Curvature max, min
    static float prev_max_curvature = 0.0;
    static float prev_min_curvature = 1.0;
    float max_curvature = 0.0;
    float min_curvature = 1.0;
    bool use_rainbow = rainbow_property_->getBool();
    Ogre::ColourValue max_color = max_color_property_->getOgreColor();
    Ogre::ColourValue min_color = min_color_property_->getOgreColor();
    for (size_t i = 0; i < point_count; ++i)
      {
        if(i % skip_time != 0){
          ptr += point_step;
          continue;
        }
        float x = *reinterpret_cast<const float*>(ptr + xoff);
        float y = *reinterpret_cast<const float*>(ptr + yoff);
        float z = *reinterpret_cast<const float*>(ptr + zoff);
        float normal_x = *reinterpret_cast<const float*>(ptr + normal_xoff);
        float normal_y = *reinterpret_cast<const float*>(ptr + normal_yoff);
        float normal_z = *reinterpret_cast<const float*>(ptr + normal_zoff);
        float curvature = *reinterpret_cast<const float*>(ptr + curvature_off);
        int r=1,g=0,b=0;

        if (validateFloats(Ogre::Vector3(x, y, z)) && validateFloats(Ogre::Vector3(normal_x, normal_y, normal_z)))
          {
            boost::shared_ptr<NormalVisual> visual;
            if(visuals_.full()){
              visual = visuals_.front();
            }else{
              visual.reset(new NormalVisual( context_->getSceneManager(), scene_node_ ));
            }
            visual->setValues( x, y, z, normal_x, normal_y, normal_z );
            visual->setFramePosition( position );
            visual->setFrameOrientation( orientation );
            visual->setScale( scale_ );

            QColor color = color_property_->getColor();
            Ogre::Vector3 dir_vec(normal_x, normal_y, normal_z);
            switch((NormalDisplay::ColorTypes) style_property_->getOptionInt()){
            case (NormalDisplay::POINTS_COLOR):
              {
                int r=1,g=0,b=0;
                if(rgbai != -1){
                  b = *reinterpret_cast<const uint8_t*>(ptr + rgbaoff);
                  g = *reinterpret_cast<const uint8_t*>(ptr + rgbaoff + 1*sizeof(uint8_t));
                  r = *reinterpret_cast<const uint8_t*>(ptr + rgbaoff + 2*sizeof(uint8_t));
                }
                visual->setColor( r/256.0, g/256.0, b/256.0, alpha_ );
              }
              break;
            case (NormalDisplay::FLAT_COLOR):
              visual->setColor(color.redF(), color.greenF(), color.blueF(), alpha_);
              break;
            case (NormalDisplay::DIRECTION_COLOR):
              visual->setColor( dir_vec.dotProduct(Ogre::Vector3(-1,0,0)), dir_vec.dotProduct(Ogre::Vector3(0,1,0)), dir_vec.dotProduct(Ogre::Vector3(0,0,-1)), alpha_ );
              break;
            case (NormalDisplay::CURVATURE_COLOR):
              if(use_rainbow){
                float prev_diff = prev_max_curvature - prev_min_curvature;
                float value = 1 - (curvature - prev_min_curvature)/prev_diff;
                float rf,gf,bf;
                getRainbow(value ,rf, gf, bf);
                visual->setColor( rf, gf, bf, alpha_);
              }else{
                float value  = curvature/(prev_max_curvature - prev_min_curvature);
                value = std::min(value, 1.0f);
                value = std::max(value, 0.0f);

                float rf = max_color.r * value + min_color.r * ( 1 - value );
                float gf = max_color.g * value + min_color.g * ( 1 - value );
                float bf = max_color.b * value + min_color.b * ( 1 - value );
                visual->setColor( rf, gf, bf, alpha_);
              }
              max_curvature = std::max(max_curvature, curvature);
              min_curvature = std::min(min_curvature, curvature);

              break;
            }
            visuals_.push_back(visual);
          }

        ptr += point_step;
      }
    prev_min_curvature = min_curvature;
    prev_max_curvature = max_curvature;

  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::NormalDisplay,rviz::Display )
