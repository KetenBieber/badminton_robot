/**
 * @file SerialHW.hpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-01-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note : 速度3个float + 1个目标偏移量的float+
 * 1个动作位：1表示规划中，2表示捡球，3表示射球
 * 4*4+1 = 17个Bytes --- 17个uint8_t
 * @versioninfo :
 */

#pragma once

#include <XmlRpc.h>

#include <geometry_msgs/Twist.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>

#include <serial/serial.h>

#include <any_node/ThreadedPublisher.hpp>
#include <any_worker/Worker.hpp>
#include <condition_variable>
#include <move_control/ActionMsg.h>
#include <mutex>
#include <queue>
#include <signal_handler/SignalHandler.hpp>
#include <std_msgs/Float32.h>
#include <thread>

namespace serial_hw {

struct SerialMessage {
  std::vector<uint8_t> data;
  ros::Time timestamp;
};

class SerialHW {
public:
  SerialHW() = default;
  ~SerialHW() = default;

  bool init(ros::NodeHandle &nh);
  void read(const ros::Time &time);

  void sendThreadLoop();
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void actionCallback(const move_control::ActionMsg::ConstPtr &msg);
  void deltaCallback(const std_msgs::Float32::ConstPtr &msg);

  void handleSignal(int /* signum */);

private:
  // 目前不需要
  void unpack();

  // msg
  union VEL_CMD {
    uint8_t data[4];
    float velData;
  } vel_cmd_[3];      // 3个浮点数
  uint8_t action_cmd; // 1个整数
  union DELTA_DATA {
    uint8_t data[4];
    float delta;
  } delta_data_;

  /* ros relative */
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber action_sub_;
  ros::Subscriber delta_sub_;

  /* worker relative */
  std::shared_ptr<any_worker::Worker> send_worker_;

  /* thread relative */
  std::thread send_thread_;
  std::atomic<bool> is_running_{false};

  // 消息队列和同步工具
  std::queue<SerialMessage> cmd_vel_queue_;
  std::queue<SerialMessage> action_queue_;
  std::queue<SerialMessage> delta_queue_;

  std::mutex cmd_vel_mutex_;
  std::mutex action_mutex_;
  std::mutex delta_mutex_;
  std::mutex send_cv_mutex_;

  std::condition_variable msg_cv_;

  SerialMessage cmd_msg_;
  SerialMessage action_msg_;
  SerialMessage delta_msg_;

  /* serial relative */
  serial::Serial serial_{};
};

} // namespace serial_hw
