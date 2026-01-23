/**
 * @file SerialHW.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-01-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */

#include <serial_hw/SerialHW.hpp>

namespace serial_hw {

bool SerialHW::init(ros::NodeHandle &nh) {

  nh_ = nh;

  std::string serial_port;
  if (!nh_.getParam("serial_port", serial_port)) {
    ROS_ERROR("[serial_port] not set!");
    return false;
  }

  //  初始化serial库
  if (!serial_.isOpen()) {

    serial_.setPort(serial_port);
    serial_.setBaudrate(115200);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(100);
    serial_.setTimeout(time_out);
    serial_.open();
    serial_.flushInput();

    // 速度消息id为1
    cmd_msg_.data.resize(20, 0); // 20 = 2+2+1+1+2+12
    // 填充消息头
    cmd_msg_.data[0] = 0xFC; // header1
    cmd_msg_.data[1] = 0xFB; // header2
    cmd_msg_.data[2] = 0x01; // ID
    cmd_msg_.data[3] = 12;   // Data length = 4*3

    // 填充消息尾
    cmd_msg_.data[16] = 0x00; // CRC placeholder
    cmd_msg_.data[17] = 0x00; // CRC placeholder
    cmd_msg_.data[18] = 0xFD; // footer1
    cmd_msg_.data[19] = 0xFE; // footer2

    // 动作消息id为2
    action_msg_.data.resize(9, 0); // 8 + 1
    // 填充消息头
    action_msg_.data[0] = 0xFC; // header1
    action_msg_.data[1] = 0xFB; // header2
    action_msg_.data[2] = 0x02; // ID
    action_msg_.data[3] = 1;    // Data length = 1

    // 填充消息尾
    action_msg_.data[5] = 0x00; // CRC placeholder
    action_msg_.data[6] = 0x00; // CRC placeholder
    action_msg_.data[7] = 0xFD; // footer1
    action_msg_.data[8] = 0xFE; // footer2

    // delta消息id为3
    delta_msg_.data.resize(12, 0); // 8+4
    // 填充消息头
    delta_msg_.data[0] = 0xFC; // header1
    delta_msg_.data[1] = 0xFB; // header2
    delta_msg_.data[2] = 0x03; // ID
    delta_msg_.data[3] = 4;    // Data length = 4*1

    // 填充消息尾
    delta_msg_.data[8] = 0x00;  // CRC placeholder
    delta_msg_.data[9] = 0x00;  // CRC placeholder
    delta_msg_.data[10] = 0xFD; // footer1
    delta_msg_.data[11] = 0xFE; // footer2

    // 注册订阅者
    cmd_vel_sub_ =
        nh_.subscribe("/cmd_vel", 10, &SerialHW::cmdVelCallback, this);
    action_sub_ = nh_.subscribe("/action", 10, &SerialHW::actionCallback, this);
    delta_sub_ = nh_.subscribe("/delta", 10, &SerialHW::deltaCallback, this);

    /* signal handler */
    signal_handler::SignalHandler::bindAll(&SerialHW::handleSignal, this);

    /* thread init */
    is_running_ = true;
    send_thread_ = std::thread(&SerialHW::sendThreadLoop, this);

    ROS_INFO("SerialHW initialized successfully");

    return true;
  } else
    return false;
}

void SerialHW::sendThreadLoop() {
  while (is_running_) {
    SerialMessage send_msg;
    bool has_msg = false;

    {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      if (!cmd_vel_queue_.empty()) {
        send_msg = cmd_vel_queue_.front();
        cmd_vel_queue_.pop();
        has_msg = true;
      }
    }

    // cmd_vel为空时，检查action
    if (!has_msg) {
      std::lock_guard<std::mutex> lock(action_mutex_);
      if (!action_queue_.empty()) {
        send_msg = action_queue_.front();
        action_queue_.pop();
        has_msg = true;
      }
    }

    if (!has_msg) {
      std::lock_guard<std::mutex> lock(delta_mutex_);
      if (!delta_queue_.empty()) {
        send_msg = delta_queue_.front();
        delta_queue_.pop();
        has_msg = true;
      }
    }

    if (has_msg) {
      // 发送消息（在锁外执行）
      try {
        serial_.flushOutput();
        serial_.write(send_msg.data.data(), send_msg.data.size());
        ROS_DEBUG("Message sent successfully at time: %.3f",
                  send_msg.timestamp.toSec());
      } catch (const std::exception &e) {
        ROS_ERROR("Failed to write to serial port: %s", e.what());
      }
    } else {
      // 两个队列都空，等待唤醒（超时1s防止意外卡住）
      std::unique_lock<std::mutex> lock(send_cv_mutex_);
      msg_cv_.wait_for(lock, std::chrono::milliseconds(1000),
                       [this] { return is_running_ == false; });
    }
  }
}

void SerialHW::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  SerialMessage new_msg = cmd_msg_;
  new_msg.timestamp = ros::Time::now();

  float linear_x = static_cast<float>(msg->linear.x);
  float linear_y = static_cast<float>(msg->linear.y);
  float angular_z = static_cast<float>(msg->angular.z);

  memcpy(new_msg.data.data() + 4, &linear_x, 4);
  memcpy(new_msg.data.data() + 8, &linear_y, 4);
  memcpy(new_msg.data.data() + 12, &angular_z, 4);

  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    cmd_vel_queue_.push(new_msg);
  }
  msg_cv_.notify_one(); // 立即唤醒发送线程
}

void SerialHW::actionCallback(const move_control::ActionMsg::ConstPtr &msg) {
  SerialMessage new_msg = action_msg_;
  new_msg.timestamp = ros::Time::now();

  uint8_t action_flag = static_cast<uint8_t>(msg->action_flag);
  memcpy(new_msg.data.data() + 4, &action_flag, 1);

  {
    std::lock_guard<std::mutex> lock(action_mutex_);
    action_queue_.push(new_msg);
  }
  msg_cv_.notify_one(); // 立即唤醒发送线程
}

void SerialHW::deltaCallback(const std_msgs::Float32::ConstPtr &msg) {
  SerialMessage new_msg = delta_msg_;
  delta_msg_.timestamp = ros::Time::now();

  float delta = msg->data;
  memcpy(new_msg.data.data() + 4, &delta, 4);

  {
    std::lock_guard<std::mutex> lock(delta_mutex_);
    delta_queue_.push(new_msg);
  }
  msg_cv_.notify_one(); // 立即唤醒发送线程
}

void SerialHW::handleSignal(int /* signum */) {
  is_running_ = false;
  msg_cv_.notify_one();
  if (send_thread_.joinable()) {
    send_thread_.join();
  }

  ros::shutdown();
  ROS_INFO("exit serial_hw node!");
}

// 解析接收到的数据
void SerialHW::unpack() {}

} // namespace serial_hw