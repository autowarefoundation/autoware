// Copyright 2020 Autoware Foundation
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

/**
 * @file nl80211.cpp
 * @brief 802.11 netlink-based interface class
 */

#include "system_monitor/net_monitor/nl80211.hpp"

#include <linux/nl80211.h>
#include <net/if.h>
#include <netlink/genl/ctrl.h>
#include <netlink/genl/genl.h>

NL80211::NL80211() : bitrate_(0.0), initialized_(false), socket_(nullptr), id_(-1), cb_(nullptr) {}

// Attribute validation policy
static struct nla_policy stats_policy[NL80211_STA_INFO_MAX + 1];
static struct nla_policy rate_policy[NL80211_RATE_INFO_MAX + 1];

// cspell: ignore ghdr
static int callback(struct nl_msg * msg, void * arg)
{
  int ret;
  auto * rate = reinterpret_cast<float *>(arg);

  // Return actual netlink message.
  struct nlmsghdr * nlh = nlmsg_hdr(msg);
  // Return pointer to message payload.
  auto * ghdr = static_cast<genlmsghdr *>(nlmsg_data(nlh));

  struct nlattr * tb[NL80211_ATTR_MAX + 1];
  struct nlattr * sinfo[NL80211_STA_INFO_MAX + 1];
  struct nlattr * rinfo[NL80211_RATE_INFO_MAX + 1];

  // Create attribute index based on a stream of attributes.
  ret =
    nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(ghdr, 0), genlmsg_attrlen(ghdr, 0), nullptr);
  // Returns 0 on success or a negative error code.
  if (ret < 0) {
    return NL_SKIP;
  }

  // Information about a station missing
  if (!tb[NL80211_ATTR_STA_INFO]) {
    return NL_SKIP;
  }

  // Create attribute index based on nested attribute.
  ret = nla_parse_nested(sinfo, NL80211_STA_INFO_MAX, tb[NL80211_ATTR_STA_INFO], stats_policy);
  // Returns 0 on success or a negative error code.
  if (ret < 0) {
    return NL_SKIP;
  }

  // current unicast tx rate missing
  if (!sinfo[NL80211_STA_INFO_TX_BITRATE]) {
    return NL_SKIP;
  }

  // Create attribute index based on nested attribute.
  ret =
    nla_parse_nested(rinfo, NL80211_RATE_INFO_MAX, sinfo[NL80211_STA_INFO_TX_BITRATE], rate_policy);
  // Returns 0 on success or a negative error code.
  if (ret < 0) {
    return NL_SKIP;
  }

  // total bitrate exists
  if (rinfo[NL80211_RATE_INFO_BITRATE]) {
    // Return payload of 16 bit integer attribute.
    *rate = static_cast<float>(nla_get_u16(rinfo[NL80211_RATE_INFO_BITRATE])) / 10;
  }

  return NL_SKIP;
}

void NL80211::init()
{
  // Allocate new netlink socket.
  socket_ = nl_socket_alloc();
  // Returns newly allocated netlink socket or NULL.
  if (!socket_) {
    return;
  }

  // Connect a generic netlink socket.
  // Returns 0 on success or a negative error code.
  int ret = genl_connect(socket_);
  if (ret < 0) {
    shutdown();
    return;
  }

  // Resolve generic netlink family name to its identifier.
  id_ = genl_ctrl_resolve(socket_, "nl80211");
  // Returns a positive identifier or a negative error code.
  if (id_ < 0) {
    shutdown();
    return;
  }

  // Allocate a new callback handle.
  cb_ = nl_cb_alloc(NL_CB_DEFAULT);
  // Returns newly allocated callback handle or NULL.
  if (!cb_) {
    shutdown();
    return;
  }

  // Set up a callback.
  ret = nl_cb_set(cb_, NL_CB_VALID, NL_CB_CUSTOM, callback, reinterpret_cast<void *>(&bitrate_));
  // Returns 0 on success or a negative error code.
  if (ret < 0) {
    shutdown();
    return;
  }

  initialized_ = true;
}

float NL80211::getBitrate(const char * ifa_name)
{
  int ret;
  struct nl_msg * msg;
  void * hdr;
  int index;

  bitrate_ = 0.0;

  if (!initialized_) {
    return bitrate_;
  }

  // Get index of the network interface
  index = if_nametoindex(ifa_name);
  // Returns index number of the network interface on success
  // or 0 on error and errno is set appropriately
  if (!index) {
    return bitrate_;
  }

  // Allocate a new netlink message with the default maximum payload size.
  msg = nlmsg_alloc();
  // Returns newly allocated netlink message or NULL.
  if (!msg) {
    return bitrate_;
  }

  // Add Generic Netlink headers to Netlink message.
  hdr = genlmsg_put(msg, NL_AUTO_PORT, NL_AUTO_SEQ, id_, 0, NLM_F_DUMP, NL80211_CMD_GET_STATION, 0);
  // Returns pointer to user header or NULL if an error occurred.
  if (!hdr) {
    nlmsg_free(msg);
    return bitrate_;
  }

  // Add 32 bit integer attribute to netlink message.
  ret = nla_put_u32(msg, NL80211_ATTR_IFINDEX, index);
  // Returns 0 on success or a negative error code.
  if (ret < 0) {
    nlmsg_free(msg);
    return bitrate_;
  }

  // Finalize and transmit Netlink message.
  ret = nl_send_auto(socket_, msg);
  // Returns number of bytes sent or a negative error code.
  if (ret < 0) {
    nlmsg_free(msg);
    return bitrate_;
  }

  // Receive a set of messages from a netlink socket.
  ret = nl_recvmsgs(socket_, cb_);
  // 0 on success or a negative error code from nl_recv().
  if (ret < 0) {
    nlmsg_free(msg);
    return bitrate_;
  }

  nlmsg_free(msg);
  return bitrate_;
}

void NL80211::shutdown()
{
  if (cb_) {
    nl_cb_put(cb_);
  }
  nl_close(socket_);
  nl_socket_free(socket_);
  initialized_ = false;
}
