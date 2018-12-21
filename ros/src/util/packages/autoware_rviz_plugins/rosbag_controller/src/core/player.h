/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, OMRON AUTOMOTIVE ELECTRONICS
*  Copyright (c) 2008, Willow Garage, Inc.
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
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
********************************************************************/

#ifndef ROSBAG_CONTROL_PLAYER_H
#define ROSBAG_CONTROL_PLAYER_H

#include <sys/stat.h>
#if !defined(_MSC_VER)
  #include <termios.h>
  #include <unistd.h>
#endif
#include <time.h>

#include <queue>
#include <string>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "time_translator.h"
#include "rosbag/macros.h"

using namespace rosbag;

#define ROSBAG_PLAYER_S_OK               (0)
#define ROSBAG_PLAYER_E_PLAY_FINISHED    (-1000)
#define ROSBAG_PLAYER_E_PLAY_ABORTED     (-1001)

namespace rosbag_control {

//! Helper function to create AdvertiseOptions from a MessageInstance
/*!
 *  param msg         The Message instance for which to generate adveritse options
 *  param queue_size  The size of the outgoing queue
 */
ros::AdvertiseOptions createAdvertiseOptions(MessageInstance const& msg, uint32_t queue_size);

ROSBAG_DECL ros::AdvertiseOptions createAdvertiseOptions(const ConnectionInfo* c, uint32_t queue_size);


struct ROSBAG_DECL PlayerOptions
{
    PlayerOptions();

    void check();

    bool     start_paused;
    bool     at_once;
    bool     bag_time;
    double   bag_time_frequency;
    double   time_scale;
    int      queue_size;
    bool     try_future;
    bool     has_time;
    bool     loop;
    float    time;
    bool     has_duration;
    float    duration;
    ros::Duration skip_empty;

    std::vector<std::string> bags;
    std::vector<std::string> topics;
    std::vector<std::string> pause_topics;  // TODO remove soon
};


//! PRIVATE. A helper class to track relevant state for publishing time
class ROSBAG_DECL TimePublisher {
public:
    /*! Create a time publisher
     *  A publish_frequency of < 0 indicates that time shouldn't actually be published
     */
    TimePublisher();

    void resetClock();

    void setPublishFrequency(double publish_frequency);
    
    void setTimeScale(double time_scale);

    /*! Set the horizon that the clock will run to */
    void setHorizon(const ros::Time& horizon);

    /*! Set the horizon that the clock will run to */
    void setWCHorizon(const ros::WallTime& horizon);

    /*! Set the current time */
    void setTime(const ros::Time& time);

    /*! Get the current time */
    ros::Time const& getTime() const;

    /*! Run the clock for AT MOST duration
     *
     * If horizon has been reached this function returns immediately
     */
    void runClock(const ros::WallDuration& duration);

    //! Sleep as necessary, but don't let the click run 
    void runStalledClock(const ros::WallDuration& duration);

    //! Step the clock to the horizon
    void stepClock();

    bool horizonReached();

private:
    bool do_publish_;
    
    double publish_frequency_;
    double time_scale_;
    
    ros::NodeHandle node_handle_;
    ros::Publisher time_pub_;
    
    ros::WallDuration wall_step_;
    
    ros::WallTime next_pub_;

    ros::WallTime wc_horizon_;
    ros::Time horizon_;
    ros::Time current_;
};

enum PlayerState
{
    PlayerState_Unloaded,
    PlayerState_Stopped,
    PlayerState_Playing,
    PlayerState_Exiting,
};

struct ROSBAG_DECL PlayerStatus
{
    PlayerStatus();
    PlayerStatus &operator=(const PlayerStatus &status);

    PlayerState state;
    int error;
    ros::Duration totalTime;
    ros::Duration currentTime;
};

//! PRIVATE.  Player class to abstract the interface to the player
/*!
 *  This API is currently considered private, but will be released in the 
 * future after view.
 */
class ROSBAG_DECL Player
{
public:
    Player(PlayerOptions const& options);
    ~Player();

    int load();
    void play();
    void stop();
    void pause();
    void seek(const ros::Duration &offset, int balance=10);

    void getStatus(PlayerStatus &status);

private:
    void init();
    void init(rosbag::View::iterator it);
    void init(const ros::Time &time);

    void doPlay();

    void doPublish(ros::NodeHandle &nh, rosbag::MessageInstance const& m, bool seekPlay=false);

    bool doPauseIfNeed(ros::NodeHandle &nh);
    bool doSeekIfNeed();

    void printTime();

private:
    bool canPlayContinue(ros::NodeHandle &nh);

private:
    PlayerOptions options_;
    PlayerStatus status_;

    ros::NodeHandle node_handle_;
    boost::scoped_ptr<rosbag::View> view_;
    boost::scoped_ptr<boost::thread> play_thread_;

    rosbag::View::iterator msg_it_;

    std::vector<boost::shared_ptr<Bag> >  bags_;
    std::map<std::string, ros::Publisher> publishers_;

    TimeTranslator time_translator_;
    TimePublisher time_publisher_;

    ros::Time start_time_;
    ros::Duration bag_length_;

    boost::mutex                  seek_mutex_;           //!< mutex for seek
    bool seek_request_;
    int seek_balance_;
    ros::Time seek_target_time_;
};


} // namespace rosbag_control

#endif
