// Copyright 2023 TIER IV, Inc.
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

#include "synchronized_grid_map_fusion_node.hpp"

#include "autoware/probabilistic_occupancy_grid_map/cost_value/cost_value.hpp"
#include "autoware/probabilistic_occupancy_grid_map/utils/utils.hpp"

// cspell: ignore LOBF

namespace autoware::occupancy_grid_map
{
using autoware::universe_utils::ScopedTimeTrack;
using costmap_2d::OccupancyGridMapFixedBlindSpot;
using costmap_2d::OccupancyGridMapLOBFUpdater;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;

GridMapFusionNode::GridMapFusionNode(const rclcpp::NodeOptions & node_options)
: Node("synchronized_occupancy_grid_map_fusion", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  /* load input parameters */
  {
    // get input topics
    declare_parameter("fusion_input_ogm_topics", std::vector<std::string>{});
    input_topics_ = get_parameter("fusion_input_ogm_topics").as_string_array();
    if (input_topics_.empty()) {
      throw std::runtime_error("The number of input topics must larger than 0.");
    }
    num_input_topics_ = input_topics_.size();
    if (num_input_topics_ > 12) {
      RCLCPP_WARN(
        this->get_logger(), "maximum num_input_topics_ is 12. current num_input_topics_ is %zu",
        num_input_topics_);
      num_input_topics_ = 12;
    }
    // get input topic reliabilities
    declare_parameter("input_ogm_reliabilities", std::vector<double>{});
    input_topic_weights_ = get_parameter("input_ogm_reliabilities").as_double_array();

    if (input_topic_weights_.empty()) {  // no topic weight is set
      // set equal weight
      for (std::size_t topic_i = 0; topic_i < num_input_topics_; ++topic_i) {
        input_topic_weights_map_[input_topics_.at(topic_i)] = 1.0 / num_input_topics_;
      }
    } else if (num_input_topics_ == input_topic_weights_.size()) {
      // set weight to map
      for (std::size_t topic_i = 0; topic_i < num_input_topics_; ++topic_i) {
        input_topic_weights_map_[input_topics_.at(topic_i)] = input_topic_weights_.at(topic_i);
      }
    } else {
      throw std::runtime_error("The number of weights does not match the number of topics.");
    }
  }

  // Set fusion timing parameters
  match_threshold_sec_ = declare_parameter<double>("match_threshold_sec", 0.01);
  timeout_sec_ = declare_parameter<double>("timeout_sec", 0.1);
  input_offset_sec_ = declare_parameter("input_offset_sec", std::vector<double>{});
  if (!input_offset_sec_.empty() && num_input_topics_ != input_offset_sec_.size()) {
    throw std::runtime_error("The number of offsets does not match the number of topics.");
  } else if (input_offset_sec_.empty()) {
    // if there are not input offset, set 0.0
    for (std::size_t topic_i = 0; topic_i < num_input_topics_; ++topic_i) {
      input_offset_sec_.push_back(0.0);
    }
  }
  // Set fusion map parameters
  map_frame_ = declare_parameter("map_frame_", "map");
  base_link_frame_ = declare_parameter("base_link_frame_", "base_link");
  gridmap_origin_frame_ = declare_parameter("grid_map_origin_frame_", "base_link");
  fusion_map_length_x_ = declare_parameter("fusion_map_length_x", 100.0);
  fusion_map_length_y_ = declare_parameter("fusion_map_length_y", 100.0);
  fusion_map_resolution_ = declare_parameter("fusion_map_resolution", 0.5);

  // set fusion method
  std::string fusion_method_str = declare_parameter("fusion_method", "overwrite");
  if (fusion_method_str == "overwrite") {
    fusion_method_ = fusion_policy::FusionMethod::OVERWRITE;
  } else if (fusion_method_str == "log-odds") {
    fusion_method_ = fusion_policy::FusionMethod::LOG_ODDS;
  } else if (fusion_method_str == "dempster-shafer") {
    fusion_method_ = fusion_policy::FusionMethod::DEMPSTER_SHAFER;
  } else {
    throw std::runtime_error("The fusion method is not supported.");
  }

  // sub grid_map
  grid_map_subs_.resize(num_input_topics_);
  for (std::size_t topic_i = 0; topic_i < num_input_topics_; ++topic_i) {
    std::function<void(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)> fnc = std::bind(
      &GridMapFusionNode::onGridMap, this, std::placeholders::_1, input_topics_.at(topic_i));
    grid_map_subs_.at(topic_i) = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_topics_.at(topic_i), rclcpp::QoS{1}.best_effort(), fnc);
  }

  // pub grid_map
  fused_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "~/output/occupancy_grid_map", rclcpp::QoS{1}.reliable());
  single_frame_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "~/debug/single_frame_map", rclcpp::QoS{1}.reliable());

  // updater
  occupancy_grid_map_updater_ptr_ = std::make_shared<OccupancyGridMapLOBFUpdater>(
    fusion_map_length_x_ / fusion_map_resolution_, fusion_map_length_y_ / fusion_map_resolution_,
    fusion_map_resolution_);

  // Set timer
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout_sec_));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&GridMapFusionNode::timer_callback, this));

  // set offset map
  for (size_t i = 0; i < input_offset_sec_.size(); ++i) {
    offset_map_[input_topics_[i]] = input_offset_sec_[i];
  }

  // init dict
  for (size_t i = 0; i < input_topics_.size(); ++i) {
    gridmap_dict_[input_topics_[i]] = nullptr;
    gridmap_dict_tmp_[input_topics_[i]] = nullptr;
  }

  // debug tools
  {
    using autoware::universe_utils::DebugPublisher;
    using autoware::universe_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, "synchronized_grid_map_fusion");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");

    // time keeper setup
    bool use_time_keeper = declare_parameter<bool>("publish_processing_time_detail");
    if (use_time_keeper) {
      detailed_processing_time_publisher_ =
        this->create_publisher<autoware::universe_utils::ProcessingTimeDetail>(
          "~/debug/processing_time_detail_ms", 1);
      auto time_keeper = autoware::universe_utils::TimeKeeper(detailed_processing_time_publisher_);
      time_keeper_ = std::make_shared<autoware::universe_utils::TimeKeeper>(time_keeper);
    }
  }
}

/**
 * @brief Set the period of the timer
 *
 * @param new_period
 */
void GridMapFusionNode::setPeriod(const int64_t new_period)
{
  if (!timer_) {
    return;
  }
  int64_t old_period = 0;
  rcl_ret_t ret = rcl_timer_get_period(timer_->get_timer_handle().get(), &old_period);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't get old period");
  }
  ret = rcl_timer_exchange_period(timer_->get_timer_handle().get(), new_period, &old_period);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't exchange_period");
  }
}

/**
 * @brief Callback function for the grid map message add data to buffer
 *
 * @param occupancy_grid_msg
 * @param topic_name
 */
void GridMapFusionNode::onGridMap(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & occupancy_grid_msg,
  const std::string & topic_name)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::lock_guard<std::mutex> lock(mutex_);
  const bool is_already_subscribed_this = (gridmap_dict_[topic_name] != nullptr);
  const bool is_already_subscribed_tmp = std::any_of(
    std::begin(gridmap_dict_tmp_), std::end(gridmap_dict_tmp_),
    [](const auto & e) { return e.second != nullptr; });

  // already subscribed
  if (is_already_subscribed_this) {
    gridmap_dict_tmp_[topic_name] = occupancy_grid_msg;

    if (!is_already_subscribed_tmp) {
      auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout_sec_));
      try {
        setPeriod(period.count());
      } catch (rclcpp::exceptions::RCLError & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
      }
      timer_->reset();
    }
    // first time to subscribe in this period
  } else {
    gridmap_dict_[topic_name] = occupancy_grid_msg;  // add to buffer

    // check if all topics are subscribed
    bool is_subscribed_all = std::all_of(
      std::begin(gridmap_dict_), std::end(gridmap_dict_),
      [](const auto & e) { return e.second != nullptr; });
    // Then, go to publish
    if (is_subscribed_all) {
      for (const auto & e : gridmap_dict_tmp_) {
        if (e.second != nullptr) {
          gridmap_dict_[e.first] = e.second;
        }
      }
      std::for_each(std::begin(gridmap_dict_tmp_), std::end(gridmap_dict_tmp_), [](auto & e) {
        e.second = nullptr;
      });

      timer_->cancel();
      publish();
      // if not all topics are subscribed, set timer
    } else if (offset_map_.size() > 0) {
      timer_->cancel();
      auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout_sec_ - offset_map_[topic_name]));
      try {
        setPeriod(period.count());
      } catch (rclcpp::exceptions::RCLError & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
      }
      timer_->reset();
    }
  }
}

/**
 * @brief Timer callback function
 *
 */
void GridMapFusionNode::timer_callback()
{
  using std::chrono_literals::operator""ms;
  timer_->cancel();
  if (mutex_.try_lock()) {
    publish();
    mutex_.unlock();
  } else {
    try {
      std::chrono::nanoseconds period = 10ms;
      setPeriod(period.count());
    } catch (rclcpp::exceptions::RCLError & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
    }
    timer_->reset();
  }
}

/**
 * @brief Publisher function
 *
 */
void GridMapFusionNode::publish()
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  stop_watch_ptr_->toc("processing_time", true);
  builtin_interfaces::msg::Time latest_stamp = get_clock()->now();
  double height = 0.0;

  // merge available gridmap
  std::vector<OccupancyGridMapFixedBlindSpot> subscribed_maps;
  std::vector<double> weights;
  {  // merging grid map
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<ScopedTimeTrack>("merge_grid_map", *time_keeper_);

    for (const auto & e : gridmap_dict_) {
      if (e.second != nullptr) {
        subscribed_maps.push_back(GridMapFusionNode::OccupancyGridMsgToGridMap(*e.second));
        weights.push_back(input_topic_weights_map_[e.first]);
        latest_stamp = e.second->header.stamp;
        height = e.second->info.origin.position.z;
      }
    }
  }

  // return if empty
  if (subscribed_maps.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "No grid map is subscribed. Please check the topic name and the topic type.");
    return;
  }

  // fusion map
  // single frame gridmap fusion
  auto fused_map = SingleFrameOccupancyFusion(subscribed_maps, latest_stamp, weights);
  // multi frame gridmap fusion
  occupancy_grid_map_updater_ptr_->update(fused_map);

  // publish
  fused_map_pub_->publish(
    OccupancyGridMapToMsgPtr(map_frame_, latest_stamp, height, *occupancy_grid_map_updater_ptr_));
  single_frame_pub_->publish(OccupancyGridMapToMsgPtr(map_frame_, latest_stamp, height, fused_map));

  // copy 2nd temp to temp buffer
  gridmap_dict_ = gridmap_dict_tmp_;
  std::for_each(std::begin(gridmap_dict_tmp_), std::end(gridmap_dict_tmp_), [](auto & e) {
    e.second = nullptr;
  });

  // add processing time for debug
  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - latest_stamp).nanoseconds()))
        .count();
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_ptr_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }
}

OccupancyGridMapFixedBlindSpot GridMapFusionNode::SingleFrameOccupancyFusion(
  std::vector<OccupancyGridMapFixedBlindSpot> & occupancy_grid_maps,
  const builtin_interfaces::msg::Time latest_stamp, const std::vector<double> & weights)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // if only single map
  if (occupancy_grid_maps.size() == 1) {
    return occupancy_grid_maps[0];
  }

  // get map to gridmap_origin_frame_ transform
  Pose gridmap_origin{};
  try {
    gridmap_origin = utils::getPose(latest_stamp, tf_buffer_, gridmap_origin_frame_, map_frame_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(get_logger(), ex.what());
    return occupancy_grid_maps[0];
  }

  {  // create fused occupancy grid map
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<ScopedTimeTrack>("create_fused_map", *time_keeper_);

    // init fused map with calculated origin
    OccupancyGridMapFixedBlindSpot fused_map(
      occupancy_grid_maps[0].getSizeInCellsX(), occupancy_grid_maps[0].getSizeInCellsY(),
      occupancy_grid_maps[0].getResolution());
    fused_map.updateOrigin(
      gridmap_origin.position.x - fused_map.getSizeInMetersX() / 2,
      gridmap_origin.position.y - fused_map.getSizeInMetersY() / 2);

    // fix origin of each map
    for (auto & map : occupancy_grid_maps) {
      map.updateOrigin(fused_map.getOriginX(), fused_map.getOriginY());
    }

    // assume map is same size and resolutions
    for (unsigned int x = 0; x < fused_map.getSizeInCellsX(); x++) {
      for (unsigned int y = 0; y < fused_map.getSizeInCellsY(); y++) {
        // get cost of each map
        std::vector<unsigned char> costs;
        for (auto & map : occupancy_grid_maps) {
          costs.push_back(map.getCost(x, y));
        }

        // set fusion policy
        auto fused_cost = fusion_policy::singleFrameOccupancyFusion(costs, fusion_method_, weights);

        // set max cost to fused map
        fused_map.setCost(x, y, fused_cost);
      }
    }

    return fused_map;
  }  // scope for time keeper ends
}

OccupancyGridMapFixedBlindSpot GridMapFusionNode::OccupancyGridMsgToGridMap(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid_map)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  OccupancyGridMapFixedBlindSpot gridmap(
    occupancy_grid_map.info.width, occupancy_grid_map.info.height,
    occupancy_grid_map.info.resolution);
  gridmap.updateOrigin(
    occupancy_grid_map.info.origin.position.x, occupancy_grid_map.info.origin.position.y);

  for (unsigned int i = 0; i < occupancy_grid_map.info.width; i++) {
    for (unsigned int j = 0; j < occupancy_grid_map.info.height; j++) {
      const unsigned int index = i + j * occupancy_grid_map.info.width;
      gridmap.setCost(
        i, j, cost_value::inverse_cost_translation_table[occupancy_grid_map.data[index]]);
    }
  }
  return gridmap;
}

nav_msgs::msg::OccupancyGrid::UniquePtr GridMapFusionNode::OccupancyGridMapToMsgPtr(
  const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
  const float & robot_pose_z, const nav2_costmap_2d::Costmap2D & occupancy_grid_map)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  auto msg_ptr = std::make_unique<OccupancyGrid>();

  msg_ptr->header.frame_id = frame_id;
  msg_ptr->header.stamp = stamp;
  msg_ptr->info.resolution = occupancy_grid_map.getResolution();

  msg_ptr->info.width = occupancy_grid_map.getSizeInCellsX();
  msg_ptr->info.height = occupancy_grid_map.getSizeInCellsY();

  double wx{};
  double wy{};
  occupancy_grid_map.mapToWorld(0, 0, wx, wy);
  msg_ptr->info.origin.position.x = occupancy_grid_map.getOriginX();
  msg_ptr->info.origin.position.y = occupancy_grid_map.getOriginY();
  msg_ptr->info.origin.position.z = robot_pose_z;
  msg_ptr->info.origin.orientation.w = 1.0;

  msg_ptr->data.resize(msg_ptr->info.width * msg_ptr->info.height);

  const unsigned char * data = occupancy_grid_map.getCharMap();
  for (unsigned int i = 0; i < msg_ptr->data.size(); ++i) {
    msg_ptr->data[i] = cost_value::cost_translation_table[data[i]];
  }
  return msg_ptr;
}

}  // namespace autoware::occupancy_grid_map

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::occupancy_grid_map::GridMapFusionNode)
