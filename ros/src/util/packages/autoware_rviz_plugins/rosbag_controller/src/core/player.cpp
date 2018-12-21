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

#include "player.h"
#include "rosbag/message_instance.h"
#include "rosbag/view.h"

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include "rosgraph_msgs/Clock.h"

#define foreach BOOST_FOREACH

using std::map;
using std::pair;
using std::string;
using std::vector;
using boost::shared_ptr;
using ros::Exception;
using namespace rosbag;

namespace rosbag_control {

ros::AdvertiseOptions createAdvertiseOptions(const ConnectionInfo* c, uint32_t queue_size) {
    ros::AdvertiseOptions opts(c->topic, queue_size, c->md5sum, c->datatype, c->msg_def);
    ros::M_string::const_iterator header_iter = c->header->find("latching");
    opts.latch = (header_iter != c->header->end() && header_iter->second == "1");
    return opts;
}


ros::AdvertiseOptions createAdvertiseOptions(rosbag::MessageInstance const& m, uint32_t queue_size) {
    return ros::AdvertiseOptions(m.getTopic(), queue_size, m.getMD5Sum(), m.getDataType(), m.getMessageDefinition());
}

// PlayerOptions

PlayerOptions::PlayerOptions() :
    start_paused(false),
    at_once(false),
    bag_time(false),
    bag_time_frequency(0.0),
    time_scale(1.0),
    queue_size(0),
    try_future(false),
    has_time(false),
    loop(false),
    time(0.0f),
    has_duration(false),
    duration(0.0f),
    skip_empty(ros::DURATION_MAX)
{
}

void PlayerOptions::check() {
    if (bags.size() == 0)
        throw Exception("You must specify at least one bag file to play from");
    if (has_duration && duration <= 0.0)
        throw Exception("Invalid duration, must be > 0.0");
}

// PlayerStatus

PlayerStatus::PlayerStatus() :
    state(PlayerState_Unloaded),
    error(0)
{
}

PlayerStatus &PlayerStatus::operator=(const PlayerStatus &status)
{
    state = status.state;
    error = status.error;
    totalTime = status.totalTime;
    currentTime = status.currentTime;
    return *this;
}

// Player

Player::Player(PlayerOptions const& options) :
    options_(options)
{
}

Player::~Player() {
    if (play_thread_) {
        stop();
    }

    foreach(shared_ptr<rosbag::Bag> bag, bags_)
        bag->close();
}

void Player::getStatus(PlayerStatus &status)
{
    status = status_;
}

int Player::load() {
    if (status_.state != PlayerState_Unloaded) {
       ROS_WARN("already loaded");
       return 0;
    }

    options_.check();
    view_.reset();

    // Open all the bag files
    foreach(string const& filename, options_.bags) {
        ROS_INFO("Opening %s", filename.c_str());

        try
        {
            shared_ptr<rosbag::Bag> bag(new rosbag::Bag);
            bag->open(filename, bagmode::Read);
            bags_.push_back(bag);
        }
        catch (BagUnindexedException ex) {
            ROS_ERROR_STREAM("Bag file " << filename << " is unindexed.  Run rosbag reindex.");
            return -1;
        }
    }

    // Publish all messages in the bags
    View full_view;
    foreach(shared_ptr<rosbag::Bag> bag, bags_)
        full_view.addQuery(*bag);

    ros::Time initial_time = full_view.getBeginTime();

    initial_time += ros::Duration(options_.time);

    ros::Time finish_time = ros::TIME_MAX;
    if (options_.has_duration)
    {
      finish_time = initial_time + ros::Duration(options_.duration);
    }

    boost::scoped_ptr<rosbag::View> view(new rosbag::View);
    TopicQuery topics(options_.topics);

    if (options_.topics.empty())
    {
      foreach(shared_ptr<rosbag::Bag> bag, bags_)
        view->addQuery(*bag, initial_time, finish_time);
    } else {
      foreach(shared_ptr<rosbag::Bag> bag, bags_)
        view->addQuery(*bag, topics, initial_time, finish_time);
    }

    if (view->size() == 0)
    {
      ROS_ERROR("No messages to play on specified topics.");
      return -1;
    }

    // Advertise all of our messages
    foreach(const ConnectionInfo* c, view->getConnections())
    {
        ros::M_string::const_iterator header_iter = c->header->find("callerid");
        std::string callerid = (header_iter != c->header->end() ? header_iter->second : string(""));

        string callerid_topic = callerid + c->topic;

        map<string, ros::Publisher>::iterator pub_iter = publishers_.find(callerid_topic);
        if (pub_iter == publishers_.end()) {

            ros::AdvertiseOptions opts = createAdvertiseOptions(c, options_.queue_size);

            ros::Publisher pub = node_handle_.advertise(opts);
            publishers_.insert(publishers_.begin(), pair<string, ros::Publisher>(callerid_topic, pub));

            pub_iter = publishers_.find(callerid_topic);
        }
    }

    view_.swap(view);
    msg_it_ = view_->begin();

    status_.state = PlayerState_Stopped;
    status_.error = 0;
    status_.currentTime = ros::Duration();
    status_.totalTime = view_->getEndTime() - view_->getBeginTime();

    seek_request_ = false;

    init();
    return 0;
}

void Player::stop() {
    if (status_.state == PlayerState_Unloaded) {
        return;
    }

    status_.state = PlayerState_Exiting;

    if (play_thread_) {
        play_thread_->join();
        play_thread_.reset();
    }

    status_.state = PlayerState_Stopped;
    seek_request_ = false;
    init();
}

void Player::init() {
    init(view_->begin()->getTime());
}

void Player::init(rosbag::View::iterator it) {
    init(it->getTime());
}

void Player::init(const ros::Time &time) {
    // Set up our time_translator and publishers
    time_translator_.setTimeScale(options_.time_scale);

    start_time_ = view_->begin()->getTime();
    time_translator_.setRealStartTime(start_time_);
    bag_length_ = view_->getEndTime() - view_->getBeginTime();

    // reset time
    status_.currentTime = ros::Duration();
    ros::Time currentTime = time;

    time_publisher_.setTime(currentTime);

    ros::WallTime now_wt = ros::WallTime::now();
    time_translator_.setTranslatedStartTime(ros::Time(now_wt.sec, now_wt.nsec));

    time_publisher_.setTimeScale(options_.time_scale);
    if (options_.bag_time)
        time_publisher_.setPublishFrequency(options_.bag_time_frequency);
    else
        time_publisher_.setPublishFrequency(-1.0);

    ros::Time translated = time_translator_.translate(currentTime);
    ros::WallTime horizon = ros::WallTime(translated.sec, translated.nsec);

    time_publisher_.setHorizon(currentTime);
    time_publisher_.setWCHorizon(horizon);
}

void Player::play() {
    if (status_.state == PlayerState_Unloaded) {
        ROS_ERROR("call load() before.");
        return;
    } else if (status_.state != PlayerState_Stopped) {
        ROS_ERROR("playing now.");
        return;
    }

    // Update status
    status_.state = PlayerState_Playing;

    if (!play_thread_) {
        status_.currentTime = ros::Duration();
        // Spin up a thread for writing to the file
        play_thread_.reset(new boost::thread(boost::bind(&Player::doPlay, this)));
    }
}

void Player::pause() {
    if (status_.state == PlayerState_Unloaded) {
        ROS_ERROR("call load() before.");
        return;
    }

    // Update status
    status_.state = PlayerState_Stopped;
}

void Player::seek(const ros::Duration &offset, int balance) {
    if (status_.state == PlayerState_Unloaded) {
        ROS_ERROR("call load() before.");
        return;
    }

    // target time
    const ros::Time current = time_publisher_.getTime();
    ros::Time target = current + offset;
    if (target < start_time_) {
        target = start_time_;
    }

    // set seek request
    {
        boost::mutex::scoped_lock lock(seek_mutex_);
        seek_target_time_ = target;
        seek_balance_ = balance;
        seek_request_ = true;
    }
}

bool Player::doSeekIfNeed() {
    boost::mutex::scoped_lock lock(seek_mutex_);

    if (!seek_request_)
        return false;

    // search the nearest msg
    ros::Time target = seek_target_time_;
    rosbag::View::iterator it, it2;
    int index = 0;
    it = view_->begin();
    while (it != view_->end()) {
        ros::Time const& time = it->getTime();
        if (time >= target) {
            target = time;
            break;
        }
        it++;
        index++;
    }

    if (it == view_->end()) {
        // seek to head
        it = view_->begin();
        index = 0;
    } else {
        index -= seek_balance_;
        if (index < 0) {
            index = 0;
        }
    }

    it2 = view_->begin();
    while (index-- > 0) {
        it2++;
    }

    if (it2 != view_->end()) {
        init(it2);

        while (it != it2) {
            doPublish(node_handle_, *it2, true);
            it2++;
        }
    }

    // set a new iterator
    msg_it_ = it2;

    // turn off request
    seek_request_ = false;

    ROS_INFO("rosbag::player - seek");

    return true;
}

void Player::doPlay() {
    ros::NodeHandle nh;

    ROS_INFO("rosbag::player - start playing");

    while (true) {
        init();

        // Call do-publish for each message
        msg_it_ = view_->begin();
        while (msg_it_ != view_->end()) {
            if (doSeekIfNeed())
                continue;
            if (doPauseIfNeed(nh))
                continue;
            if (!canPlayContinue(nh))
                break;

            doPublish(nh, *msg_it_);
            msg_it_++;
        }

        if (!canPlayContinue(nh)) {
            status_.error = ROSBAG_PLAYER_E_PLAY_ABORTED;
            break;
        }
        if (!options_.loop) {
            status_.error = ROSBAG_PLAYER_E_PLAY_FINISHED;
            break;
        }
    }

    // to clear TF buffers since sometimes UI is freezed
    time_publisher_.resetClock();

    ROS_INFO("rosbag::player - finish playing");
}

bool Player::canPlayContinue(ros::NodeHandle &nh)
{
    return (nh.ok() && status_.state != PlayerState_Exiting);
}

bool Player::doPauseIfNeed(ros::NodeHandle &nh)
{
    if (status_.state != PlayerState_Stopped)
        return false;

    ROS_INFO("rosbag::player - paused");

    bool paused = false;
    bool seek_in_paused = false;

    ros::WallTime paused_time = ros::WallTime::now();
    while (true) {
        if (nh.ok() && status_.state == PlayerState_Stopped) {
            paused = true;
            time_publisher_.runStalledClock(ros::WallDuration(.1));
            if (doSeekIfNeed()) {
                seek_in_paused = true;
            }
            continue;
        }
        break;
    }

    // step time if paused
    if (paused && !seek_in_paused) {
        ros::WallDuration shift = ros::WallTime::now() - paused_time;
        time_translator_.shift(ros::Duration(shift.sec, shift.nsec));

        ros::Time translated = time_translator_.translate(time_publisher_.getTime());
        ros::WallTime horizon = ros::WallTime(translated.sec, translated.nsec);

        horizon += shift;
        time_publisher_.setWCHorizon(horizon);
    }

    ROS_INFO("rosbag::player - resumed");

    // return true if time has been changed
    return seek_in_paused;
}

void Player::printTime()
{
    ros::Time current_time = time_publisher_.getTime();
    ros::Duration d = current_time - start_time_;

    // update current time
    status_.currentTime = d;
}

void Player::doPublish(ros::NodeHandle &nh, rosbag::MessageInstance const& m, bool seekPlay) {
    string const& topic   = m.getTopic();
    ros::Time const& time = m.getTime();
    string callerid       = m.getCallerId();
    
    ros::Time translated = time_translator_.translate(time);
    ros::WallTime horizon = ros::WallTime(translated.sec, translated.nsec);

    time_publisher_.setHorizon(time);
    time_publisher_.setWCHorizon(horizon);

    string callerid_topic = callerid + topic;

    map<string, ros::Publisher>::iterator pub_iter = publishers_.find(callerid_topic);
    ROS_ASSERT(pub_iter != publishers_.end());

    // If immediate specified, play immediately
    if (options_.at_once) {
        time_publisher_.stepClock();
        pub_iter->second.publish(m);
        printTime();
        return;
    }

    // If skip_empty is specified, skip this region and shift.
    if (seekPlay || time - time_publisher_.getTime() > options_.skip_empty)
    {
      time_publisher_.stepClock();

      ros::WallDuration shift = ros::WallTime::now() - horizon ;
      time_translator_.shift(ros::Duration(shift.sec, shift.nsec));
      horizon += shift;
      time_publisher_.setWCHorizon(horizon);
      (pub_iter->second).publish(m);
      printTime();
      return;
    }

    while (!time_publisher_.horizonReached() && nh.ok()) {
        printTime();
        time_publisher_.runClock(ros::WallDuration(.1));
    }

    pub_iter->second.publish(m);
}

TimePublisher::TimePublisher() : time_scale_(1.0)
{
  setPublishFrequency(-1.0);
  time_pub_ = node_handle_.advertise<rosgraph_msgs::Clock>("clock",1);
  next_pub_ = ros::WallTime::now();
}

void TimePublisher::resetClock()
{
  rosgraph_msgs::Clock pub_msg;
  pub_msg.clock = ros::Time(0,0);
  time_pub_.publish(pub_msg);
}

void TimePublisher::setPublishFrequency(double publish_frequency)
{
  publish_frequency_ = publish_frequency;
  
  do_publish_ = (publish_frequency > 0.0);

  wall_step_.fromSec(1.0 / publish_frequency);
}

void TimePublisher::setTimeScale(double time_scale)
{
    time_scale_ = time_scale;
}

void TimePublisher::setHorizon(const ros::Time& horizon)
{
    horizon_ = horizon;
}

void TimePublisher::setWCHorizon(const ros::WallTime& horizon)
{
  wc_horizon_ = horizon;
}

void TimePublisher::setTime(const ros::Time& time)
{
    current_ = time;
}

ros::Time const& TimePublisher::getTime() const
{
    return current_;
}

void TimePublisher::runClock(const ros::WallDuration& duration)
{
    if (do_publish_)
    {
        rosgraph_msgs::Clock pub_msg;

        ros::WallTime t = ros::WallTime::now();
        ros::WallTime done = t + duration;

        while (t < done && t < wc_horizon_)
        {
            ros::WallDuration leftHorizonWC = wc_horizon_ - t;

            ros::Duration d(leftHorizonWC.sec, leftHorizonWC.nsec);
            d *= time_scale_;

            current_ = horizon_ - d;

            if (current_ >= horizon_)
              current_ = horizon_;

            if (t >= next_pub_)
            {
                pub_msg.clock = current_;
                time_pub_.publish(pub_msg);
                next_pub_ = t + wall_step_;
            }

            ros::WallTime target = done;
            if (target > wc_horizon_)
              target = wc_horizon_;
            if (target > next_pub_)
              target = next_pub_;

            ros::WallTime::sleepUntil(target);

            t = ros::WallTime::now();
        }
    } else {

        ros::WallTime t = ros::WallTime::now();

        ros::WallDuration leftHorizonWC = wc_horizon_ - t;

        ros::Duration d(leftHorizonWC.sec, leftHorizonWC.nsec);
        d *= time_scale_;

        current_ = horizon_ - d;
        
        if (current_ >= horizon_)
            current_ = horizon_;

        ros::WallTime target = ros::WallTime::now() + duration;

        if (target > wc_horizon_)
            target = wc_horizon_;

        ros::WallTime::sleepUntil(target);
    }
}

void TimePublisher::stepClock()
{
    if (do_publish_)
    {
        current_ = horizon_;

        rosgraph_msgs::Clock pub_msg;

        pub_msg.clock = current_;
        time_pub_.publish(pub_msg);

        ros::WallTime t = ros::WallTime::now();
        next_pub_ = t + wall_step_;
    } else {
        current_ = horizon_;
    }
}

void TimePublisher::runStalledClock(const ros::WallDuration& duration)
{
    if (do_publish_)
    {
        rosgraph_msgs::Clock pub_msg;

        ros::WallTime t = ros::WallTime::now();
        ros::WallTime done = t + duration;

        while ( t < done )
        {
            if (t > next_pub_)
            {
                pub_msg.clock = current_;
                time_pub_.publish(pub_msg);
                next_pub_ = t + wall_step_;
            }

            ros::WallTime target = done;

            if (target > next_pub_)
              target = next_pub_;

            ros::WallTime::sleepUntil(target);

            t = ros::WallTime::now();
        }
    } else {
        duration.sleep();
    }
}

bool TimePublisher::horizonReached()
{
  return ros::WallTime::now() > wc_horizon_;
}

} // namespace rosbag_control
