/**
 * @file BadmintonRobotFSM.cpp
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
#include <cmath>
#include <move_control/BadmintonRobotFSM.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace badminton_robot_fsm {

/**
 * @brief Converts an angle (in degrees) and axis to a quaternion.
 *
 * @param angle_degrees The rotation angle in degrees.
 * @param axis_x The x component of the rotation axis.
 * @param axis_y The y component of the rotation axis.
 * @param axis_z The z component of the rotation axis.
 * @return tf2::Quaternion The resulting quaternion.
 */
tf2::Quaternion createQuaternionFromAngle(double angle_degrees, double axis_x,
                                          double axis_y, double axis_z) {
  // Convert angle to radians
  double angle_radians = angle_degrees * M_PI / 180.0;
  double half_angle = angle_radians / 2.0;

  // Normalize the axis
  double axis_length =
      std::sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
  if (axis_length == 0.0) {
    throw std::invalid_argument("Rotation axis cannot be zero.");
  }
  double norm_x = axis_x / axis_length;
  double norm_y = axis_y / axis_length;
  double norm_z = axis_z / axis_length;

  // Compute quaternion components
  double w = std::cos(half_angle);
  double x = norm_x * std::sin(half_angle);
  double y = norm_y * std::sin(half_angle);
  double z = norm_z * std::sin(half_angle);

  return tf2::Quaternion(x, y, z, w);
}

bool BadmintonRobotFSM::init(const ros::NodeHandle &nh) {
  nh_ = nh;

  /* subscriber */
  camera_sub_ = nh_.subscribe("/basketball_detection_system/ball_position_3d",
                              10, &BadmintonRobotFSM::cameraDataCallback, this);

  ball_middle_ =
      nh_.subscribe("/basketball_detection_system/middle_ball", 10,
                    &BadmintonRobotFSM::ballMiddleDeltaCallback, this);
  odom_sub_ =
      nh_.subscribe("/odom", 10, &BadmintonRobotFSM::odomDataCallback, this);

  /* publisher */
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  action_pub_ = nh_.advertise<move_control::ActionMsg>("/action", 10);
  delta_pub_ = nh_.advertise<std_msgs::Float32>("/delta", 10);

  // 实例化可视化器
  visualizer_ptr_ = std::make_unique<visualization::Visualization>(nh_);
  find_ball_state_ = FindBallState::CALCULATE;
  next_state_ = RobotState::HAND_CTRL;
  state_ = RobotState::HAND_CTRL;

  // 发布tf关系，需要初始化四元数部分
  ball_tf_.child_frame_id = "ball";
  ball_tf_.header.frame_id = "odom";
  ball_tf_.transform.rotation.x = 0.0;
  ball_tf_.transform.rotation.y = 0.0;
  ball_tf_.transform.rotation.z = 0.0;
  ball_tf_.transform.rotation.w = 1.0;

  // 捏造一下odom坐标
  // debug_odom_.header.frame_id = "odom";
  // debug_odom_.child_frame_id = "base_link";
  // debug_odom_.pose.pose.position.x = 0.0;
  // debug_odom_.pose.pose.position.y = 0.0;
  // debug_odom_.pose.pose.position.z = 0.0;
  // tf2::Quaternion quaternion = createQuaternionFromAngle(0.0, 0.0, 0.0, 1.0);
  // debug_odom_.pose.pose.orientation.w = quaternion.getW();
  // debug_odom_.pose.pose.orientation.x = quaternion.getX();
  // debug_odom_.pose.pose.orientation.y = quaternion.getY();
  // debug_odom_.pose.pose.orientation.z = quaternion.getZ();

  // 启动状态机线程
  is_running_ = true;
  fsm_thread_ = std::thread(&BadmintonRobotFSM::fsmThreadLoop, this);

  /* signal handler */
  signal_handler::SignalHandler::bindAll(&BadmintonRobotFSM::handleSignal,
                                         this);
  return true;
}

bool BadmintonRobotFSM::terminalApp() {
  using_history();
  stifle_history(100);

  ros::Rate r(10);

  std::cout << "\n=== 状态控制终端 ===\n";
  std::cout << "输入数字切换标志位（一次只能一个为true）：\n";
  std::cout << "  0 → HAND_CTRL\n";
  std::cout << "  1 → AUTO_READY\n";
  std::cout << "  2 → FIND_BALL\n";
  std::cout << "  3 → FIND_POINT\n";
  std::cout << "  4 → SHOOT_BALL\n";
  std::cout << "  5 → 清空所有标志位\n";
  std::cout << "其他输入 → 无操作\n\n";

  while (ros::ok() && is_running_) {
    char *input_raw = readline("state> ");
    if (!input_raw) {
      std::cout << "\n终端输入结束\n";
      break;
    }

    std::string line = trim(input_raw);
    free(input_raw);

    if (line.empty())
      continue;

    add_history(line.c_str());
    // 只处理单个数字
    if (line.length() == 1 && std::isdigit(line[0])) {
      int choice = line[0] - '0';

      // 清空所有标志位
      if_enter_HAND_CTRL_ = false;
      if_enter_AUTO_READY_ = false;
      if_enter_FIND_BALL_ = false;
      if_enter_FIND_POINT_ = false;
      if_enter_SHOOT_BALL_ = false;

      // 根据输入设置对应标志位
      switch (choice) {
      case 0:
        if_enter_HAND_CTRL_ = true;
        std::cout << "→ 已激活: HAND_CTRL\n";
        break;
      case 1:
        if_enter_AUTO_READY_ = true;
        std::cout << "→ 已激活: AUTO_READY\n";
        break;
      case 2:
        if_enter_FIND_BALL_ = true;
        std::cout << "→ 已激活: FIND_BALL\n";
        break;
      case 3:
        if_enter_FIND_POINT_ = true;
        std::cout << "→ 已激活: FIND_POINT\n";
        break;
      case 4:
        if_enter_SHOOT_BALL_ = true;
        std::cout << "→ 已激活: SHOOT_BALL\n";
        break;
      case 5:
        std::cout << "→ 已清空所有标志位\n";
        break;
      default:
        std::cout << "无效数字（0~5）\n";
        break;
      }

      // 打印当前所有标志位状态（调试用）
      std::cout << "当前标志位状态：\n";
      std::cout << "  HAND_CTRL   : "
                << (if_enter_HAND_CTRL_ ? "true" : "false") << "\n";
      std::cout << "  AUTO_READY  : "
                << (if_enter_AUTO_READY_ ? "true" : "false") << "\n";
      std::cout << "  FIND_BALL   : "
                << (if_enter_FIND_BALL_ ? "true" : "false") << "\n";
      std::cout << "  FIND_POINT  : "
                << (if_enter_FIND_POINT_ ? "true" : "false") << "\n";
      std::cout << "  SHOOT_BALL  : "
                << (if_enter_SHOOT_BALL_ ? "true" : "false") << "\n\n";
    } else {
      std::cout << "请输入单个数字 0~5\n";
    }

    r.sleep();
  }
}

void BadmintonRobotFSM::fsmThreadLoop() {

  ros::Rate r(100);

  while (is_running_ && ros::ok()) {
    RobotState current_state;
    RobotState proposed_next;
    // 只锁一次，读取当前状态和提议状态
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_state = state_;
      proposed_next = next_state_; // 外部可能修改的提议状态
    }

    // 如果有提议切换
    if (proposed_next != current_state) {
      ROS_INFO_STREAM("Transition requested: "
                      << stringCurState(current_state) << " -> "
                      << stringCurState(proposed_next));

      // 执行退出旧状态
      exitState(current_state);

      // 切换状态（加锁写回）
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_ = proposed_next;
        next_state_ = proposed_next; // 清空提议，或设为当前防止重复
        // 或者直接 next_state_ = proposed_next; // 保持同步
      }

      // 执行进入新状态
      enterState(state_);

      ROS_INFO_STREAM("Now in state: " << stringCurState(state_));
    }

    // 状态机执行
    switch (state_) {
    case RobotState::HAND_CTRL: {
      handCtrlAction();
      break;
    }
    case RobotState::AUTO_READY: {
      autoReadyAction();
      break;
    }
    case RobotState::FIND_BALL: {
      findBallAction();
      break;
    }
    case RobotState::TAKE_BALL: {
      takeBallAction();
      break;
    };
    case RobotState::FIND_POINT: {
      findPointAction();
      break;
    }
    case RobotState::SHOOT_BALL: {
      shootBallAction();
      break;
    };
    };

    // 发布坐标关系(球到odom)
    tfBroadcast(ball_tf_);

    // 阻塞保持频率
    r.sleep();
  }
}

void BadmintonRobotFSM::cameraDataCallback(
    const geometry_msgs::PointStamped &msg) {

  // 如果z是-1（浮点数判断）,不存
  if (std::fabs(msg.point.z + 1.0) < 0.0001) {
    return;
  }

  // 如果正常，就转存数据
  {
    std::lock_guard<std::mutex> lock(read_target_mutex_);
    self_target_pos_ = msg;
    has_target_ = true;
  }
}

// 机器人位姿回调
void BadmintonRobotFSM::odomDataCallback(const nav_msgs::Odometry &msg) {
  {
    std::lock_guard<std::mutex> lock(read_odom_mutex_);
    cur_odom_ = msg;
  }
}

void BadmintonRobotFSM::ballMiddleDeltaCallback(
    const geometry_msgs::PointStamped &msg) {
  // 如果z是-1（浮点数判断）,不存
  if (-1 - msg.point.z < 0.0001f) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(read_delta_mutex_);
    ball_middle_delta_ = msg;
  }

  // 只有状态机要求发的时候我才发
  if (send_delta_) {
    delta_msg_.data = msg.point.x;
    delta_pub_.publish(delta_msg_);
  } else {
  }
}

void BadmintonRobotFSM::handCtrlAction() {
  // 手动控制
  {
    std::lock_guard<std::mutex> lock(terminal_app_mutex_);
    if (if_enter_AUTO_READY_) {
      if_enter_AUTO_READY_ = false;
      {
        // std::lock_guard<std::mutex> lock(state_mutex_);
        next_state_ = RobotState::AUTO_READY;
      }
    }
  }
  return;
}

void BadmintonRobotFSM::autoReadyAction() {

  static int flag = 0;

  switch (flag) {
  case 0:
    // 置标志位，然后让底盘转动
    action_msg_.action_flag = 2;
    action_pub_.publish(action_msg_);
    // 当收到目标时，才进入下一步
    if (has_target_) {
      flag = 1;
    }
    break;
  case 1:
    // 设置为3，底盘停止转动
    action_msg_.action_flag = 3;
    action_pub_.publish(action_msg_);
    // 等待一会，延时，以便底盘停稳
    if (wait_cnt_++ > 1000) {
      flag = 2;
      ROS_INFO("enter confirm state!");
    } else {
    }
    break;
  case 2:
    // 可视化最终目标，查看是否一致，否则不予前往
    if (!calculate_target_self_to_global()) {
      ROS_ERROR("Failed to transform ball position");
      return;
    }
    std::cout << "relative position x: " << self_target_pos_.point.x
              << " y: " << self_target_pos_.point.y << std::endl;
    std::cout << "global position x: " << global_target_pos_.point.x
              << " y: " << global_target_pos_.point.y << std::endl;
    flag = 3;
    break;
  case 3:
    // 可视化坐标
    visualizer_ptr_->visualizePoint(cur_odom_.pose.pose.position.x,
                                    cur_odom_.pose.pose.position.y, 0.0,
                                    visualization::ShapeType::Point, 0.3, 0.3,
                                    0.3, 0.0, 0.0, 1.0, 1.0, "debug", 1);
    // 可视化计算出来的点和实际的位置
    visualizer_ptr_->visualizePoint(global_target_pos_.point.x,
                                    global_target_pos_.point.y, 0.0,
                                    visualization::ShapeType::Point, 0.3, 0.3,
                                    0.3, 1.0, 0.0, 0.0, 1.0, "debug", 2);
    ball_tf_.transform.translation.x = global_target_pos_.point.x;
    ball_tf_.transform.translation.y = global_target_pos_.point.y;

    ROS_INFO_STREAM_ONCE(
        "input 0 to back to handctrl or input 2 to enter to findball");
    // 判断是否前往
    {
      std::lock_guard<std::mutex> lock(terminal_app_mutex_);
      if (if_enter_FIND_BALL_) {
        if_enter_FIND_BALL_ = false;
        {
          // std::lock_guard<std::mutex> lock(state_mutex_);
          next_state_ = RobotState::FIND_BALL; // 切换到规划状态
        }
        flag = 0;
      } else if (if_enter_HAND_CTRL_) {
        if_enter_HAND_CTRL_ = false;
        {
          // std::lock_guard<std::mutex> lock(state_mutex_);
          next_state_ = RobotState::HAND_CTRL;
        }
        flag = 0;
      }
    }
    break;
  }
}

void BadmintonRobotFSM::findBallAction() {
  // 第一次进入置为true
  if (first_send_goal_) {
    first_send_goal_ = false;
    navigateToBall();
  }

  // movebase会发布速度话题

  // 询问movebase是否到达目标点
  if (waitForNavigationResult()) {
    // 如果到达了，那么切换状态,进入近距离追踪的状态
    {
      // std::lock_guard<std::mutex> lock(state_mutex_);
      next_state_ = RobotState::TAKE_BALL;
    }
  } else {
    std::cout << "navigating to the goal ... " << std::endl;
    // 没到达的话，检查触发条件，有可能触发重新找球
    {
      std::lock_guard<std::mutex> lock(terminal_app_mutex_);
      // 如果允许返回找球状态，则返回
      if (if_enter_AUTO_READY_) {
        if_enter_AUTO_READY_ = false;
        {
          // std::lock_guard<std::mutex> lock(state_mutex_);
          next_state_ = RobotState::AUTO_READY;
        }
        cancelNavigation();
      } else if (if_enter_HAND_CTRL_) {
        if_enter_HAND_CTRL_ = false;
        {
          // std::lock_guard<std::mutex> lock(state_mutex_);
          next_state_ = RobotState::HAND_CTRL;
        }
        cancelNavigation();
      } else if (if_enter_TAKE_BALL_) {
        if_enter_TAKE_BALL_ = false;
        {
          // std::lock_guard<std::mutex> lock(state_mutex_);
          next_state_ = RobotState::TAKE_BALL;
        }
        cancelNavigation();
      }
    }
  }
}

// 近距离追踪拿球
void BadmintonRobotFSM::takeBallAction() {
  // 切换近距离跟踪

  action_msg_.action_flag = 4;
  action_pub_.publish(action_msg_);
  send_delta_ = true; // delta值开始发放

  // 距离检查，如果已经很接近本来的目标了，就确认为捡到球了
  double dx = cur_odom_.pose.pose.position.x - global_target_pos_.point.x;
  double dy = cur_odom_.pose.pose.position.y - global_target_pos_.point.y;
  double dist = sqrt(dx * dx + dy * dy);
  if (dist < 0.01) {
    {
      // std::lock_guard<std::mutex> lock(state_mutex_);
      next_state_ = RobotState::FIND_POINT;
    }
    std::cout << "Finished take ball!" << std::endl;
  } else {
    {
      std::lock_guard<std::mutex> lock(terminal_app_mutex_);
      // 如果允许返回找球状态，则返回
      if (if_enter_AUTO_READY_) {
        if_enter_AUTO_READY_ = false;
        {
          // std::lock_guard<std::mutex> lock(state_mutex_);
          next_state_ = RobotState::AUTO_READY;
        }
        // 取消本次导航任务
        cancelNavigation();
      } else if (if_enter_HAND_CTRL_) {
        if_enter_HAND_CTRL_ = false;
        {
          // std::lock_guard<std::mutex> lock(state_mutex_);
          next_state_ = RobotState::HAND_CTRL;
        }
        cancelNavigation();
      } else if (if_enter_FIND_POINT_) {
        if_enter_FIND_POINT_ = false;
        {
          // std::lock_guard<std::mutex> lock(state_mutex_);
          next_state_ = RobotState::FIND_POINT;
        }
        cancelNavigation();
      }
    }
  }
}

void BadmintonRobotFSM::findPointAction() {

  // 提取机器人yaw角
  tf2::Quaternion q(
      shoot_position_.pose.orientation.x, shoot_position_.pose.orientation.y,
      shoot_position_.pose.orientation.z, shoot_position_.pose.orientation.w);

  // 四元数归一化
  if (std::abs(q.length() - 1.0) > 1e-6) {
    ROS_WARN("Quaternion not normalized! Normalizing...");
    q.normalize();
  }

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  static bool first_flag = false;
  if (!first_flag) {
    first_flag = true;
    // 导航到射球的目标点
    sendGoalToMoveBase(shoot_position_.pose.position.x,
                       shoot_position_.pose.position.y, yaw);
  }
  // movebase发布速度话题

  // 等待是否到达
  if (waitForNavigationResult()) {
    // 如果到达了，切换状态
    {
      // std::lock_guard<std::mutex> lock(state_mutex_);
      next_state_ = RobotState::SHOOT_BALL;
    }
  } else {
    // 没到达的话，可以随时取消，回到找球状态
    {
      std::lock_guard<std::mutex> lock(terminal_app_mutex_);
      if (if_enter_AUTO_READY_) {
        if_enter_AUTO_READY_ = false;
        {
          // std::lock_guard<std::mutex> lock(state_mutex_);
          next_state_ = RobotState::AUTO_READY;
        }
        cancelNavigation();
      } else if (if_enter_HAND_CTRL_) {
        if_enter_HAND_CTRL_ = false;
        {
          // std::lock_guard<std::mutex> lock(state_mutex_);
          next_state_ = RobotState::HAND_CTRL;
        }
        cancelNavigation();
      } else if (if_enter_SHOOT_BALL_) {
        if_enter_SHOOT_BALL_ = false;
        {
          // std::lock_guard<std::mutex> lock(state_mutex_);
          next_state_ = RobotState::SHOOT_BALL;
        }
        cancelNavigation();
      }
    }
  }
}

void BadmintonRobotFSM::shootBallAction() {

  // 下发状态位
  action_pub_.publish(action_msg_);
  {
    std::lock_guard<std::mutex> lock(terminal_app_mutex_);
    if (if_enter_AUTO_READY_) {
      if_enter_AUTO_READY_ = false;
      {
        // std::lock_guard<std::mutex> lock(state_mutex_);
        next_state_ = RobotState::AUTO_READY;
      }
    } else if (if_enter_HAND_CTRL_) {
      if_enter_HAND_CTRL_ = false;
      {
        // std::lock_guard<std::mutex> lock(state_mutex_);
        next_state_ = RobotState::HAND_CTRL;
      }
    }
  }
}

// 信号处理器
void BadmintonRobotFSM::handleSignal(int /* signum */) {
  is_running_ = false;
  if (fsm_thread_.joinable()) {
    fsm_thread_.join();
  }

  ros::shutdown();
  ROS_INFO("exit robot_fsm");
}

void BadmintonRobotFSM::enterState(RobotState state) {
  switch (state) {
  case RobotState::HAND_CTRL: {
    // 进入手动模式先清空指令
    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.linear.y = 0.0;
    cmd_vel_msg_.angular.z = 0.0;

    // action动作设置为1
    action_msg_.action_flag = 1;
    has_target_ = false;
    break;
  }
  case RobotState::AUTO_READY: {
    // 标志位设置为2
    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.linear.y = 0.0;
    cmd_vel_msg_.angular.z = 0.0;
    action_msg_.action_flag = 2;
    has_target_ = false;
    wait_cnt_ = 0;
    break;
  }
  case RobotState::FIND_BALL: {
    // 标志位设置为3，规划阶段
    action_msg_.action_flag = 3;
    first_send_goal_ = true;
    break;
  }
  case RobotState::TAKE_BALL: {
    action_msg_.action_flag = 4;
    break;
  };
  case RobotState::FIND_POINT: {
    action_msg_.action_flag = 5;
    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.linear.y = 0.0;
    cmd_vel_msg_.angular.z = 0.0;
    break;
  }
  case RobotState::SHOOT_BALL: {
    action_msg_.action_flag = 6;
    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.linear.y = 0.0;
    cmd_vel_msg_.angular.z = 0.0;
    break;
  };
  };
}

// 切换出状态时需要做的事
void BadmintonRobotFSM::exitState(RobotState state) {
  switch (state) {
  case RobotState::HAND_CTRL: {
    // 退出时也先清空指令
    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.linear.y = 0.0;
    cmd_vel_msg_.angular.z = 0.0;
    break;
  }
  case RobotState::AUTO_READY: {
    // 清除rviz图示
    visualizer_ptr_->removeMarker("debug", 1);
    visualizer_ptr_->removeMarker("debug", 2);
    wait_cnt_ = 0;
    break;
  }
  case RobotState::FIND_BALL: {
    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.linear.y = 0.0;
    cmd_vel_msg_.angular.z = 0.0;
    has_target_ = false;
    first_send_goal_ = true;
    break;
  }
  case RobotState::TAKE_BALL: {
    send_delta_ = false;
    break;
  };
  case RobotState::FIND_POINT: {
    break;
  }
  case RobotState::SHOOT_BALL: {
    break;
  };
  };
}

// 发送跟踪目标到movebase
bool BadmintonRobotFSM::sendGoalToMoveBase(double goal_x, double goal_y,
                                           double yaw) {
  move_base_msgs::MoveBaseGoal goal;
  // 全局坐标系
  goal.target_pose.header.frame_id = "world";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = goal_x;
  goal.target_pose.pose.position.y = goal_y;
  goal.target_pose.pose.position.z = 0.0;

  // 保持当前朝向
  tf2::Quaternion q = createQuaternionFromAngle(yaw, 0.0, 0.0, 1.0);

  goal.target_pose.pose.orientation.x = q.getX();
  goal.target_pose.pose.orientation.y = q.getY();
  goal.target_pose.pose.orientation.z = q.getZ();
  goal.target_pose.pose.orientation.w = q.getW();

  ROS_INFO("Sending goal to move_base: (%.2f, %.2f)", goal_x, goal_y);
  ac_.sendGoal(goal);
  return true;
}

// 导航到球
bool BadmintonRobotFSM::navigateToBall() {

  return sendGoalToMoveBase(global_target_pos_.point.x,
                            global_target_pos_.point.y, 0.0);
}

// 取消本次导航
bool BadmintonRobotFSM::cancelNavigation() {
  // 停止规划
  ac_.cancelAllGoals();
  return true;
}

// 计算目标位置在全局下的位置
bool BadmintonRobotFSM::calculate_target_self_to_global() {
  try {
    // 从odom获取机器人当前在map下的位姿
    double robot_x = cur_odom_.pose.pose.position.x;
    double robot_y = cur_odom_.pose.pose.position.y;

    // 提取机器人yaw角
    tf2::Quaternion q(
        cur_odom_.pose.pose.orientation.x, cur_odom_.pose.pose.orientation.y,
        cur_odom_.pose.pose.orientation.z, cur_odom_.pose.pose.orientation.w);

    // 四元数归一化
    if (std::abs(q.length() - 1.0) > 1e-6) {
      ROS_WARN("Quaternion not normalized! Normalizing...");
      q.normalize();
    }

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 球在base_link下的相对坐标
    double ball_x_local = self_target_pos_.point.x;
    double ball_y_local = self_target_pos_.point.y;

    // 旋转 + 平移，转换到world坐标系
    double ball_x_global =
        robot_x + ball_x_local * cos(yaw) - ball_y_local * sin(yaw);
    double ball_y_global =
        robot_y + ball_x_local * sin(yaw) + ball_y_local * cos(yaw);

    // 填充全局坐标
    global_target_pos_.header.frame_id = "world";
    global_target_pos_.header.stamp = ros::Time::now();
    global_target_pos_.point.x = ball_x_global;
    global_target_pos_.point.y = ball_y_global;
    global_target_pos_.point.z = 0.0;

    ROS_INFO("Robot pose: (%.2f, %.2f, %.2f rad)", robot_x, robot_y, yaw);
    ROS_INFO("Ball local: (%.2f, %.2f)", ball_x_local, ball_y_local);
    ROS_INFO("Ball global: (%.2f, %.2f)", ball_x_global, ball_y_global);

    return true;

  } catch (const std::exception &ex) {
    ROS_ERROR("Coordinate transform failed: %s", ex.what());
    return false;
  }
}

// 获取导航状态
actionlib::SimpleClientGoalState BadmintonRobotFSM::getNavigationState() {
  return ac_.getState();
}

// 等待导航结果 ,没完成或导航失败导致的完成 返回false， 完成且成功返回true
bool BadmintonRobotFSM::waitForNavigationResult() {

  actionlib::SimpleClientGoalState state = ac_.getState();
  // 完成
  if (state.isDone()) {
    // 且成功
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      std::cout << "Navigation finished!" << std::endl;
      return true;
    } else // 或失败
    {
      std::cout << "Navigation failed! " << state.toString().c_str()
                << std::endl;
      return false;
    }
  } else {
    return false;
  }
}

void BadmintonRobotFSM::processInput(const std::string &line) {
  std::stringstream ss(line);
  std::string token;
  std::vector<std::string> args;
  while (ss >> token) {
    args.push_back(token);
  }

  if (args.empty())
    return;

  FindBallState state;
  {
    std::lock_guard<std::mutex> lock(find_ball_state_mutex_);
    state = find_ball_state_;
  }

  // 根据状态进行处理
  switch (state) {
  case FindBallState::CALCULATE:
    if (args.size() < 2) {
      std::cout << "input the relative twist: x y";
      return;
    }
    try {
      input_x_ = std::stod(args[0]);
      input_y_ = std::stod(args[1]);
      // 捏造一下odom坐标
      debug_odom_.pose.pose.position.x = 0.0;
      debug_odom_.pose.pose.position.y = 0.0;
      debug_odom_.pose.pose.position.z = 0.0;
      tf2::Quaternion quaternion = createQuaternionFromAngle(30, 0.0, 0.0, 1.0);
      debug_odom_.pose.pose.orientation.w = quaternion.getW();
      debug_odom_.pose.pose.orientation.x = quaternion.getX();
      debug_odom_.pose.pose.orientation.y = quaternion.getY();
      debug_odom_.pose.pose.orientation.z = quaternion.getZ();

      debug_calculate_target_self_to_global(input_x_, input_y_, output_x_,
                                            output_y_, debug_odom_);

      // 可视化机器人位置
      visualizer_ptr_->visualizePoint(debug_odom_.pose.pose.position.x,
                                      debug_odom_.pose.pose.position.y, 0.0,
                                      visualization::ShapeType::Point, 0.3, 0.3,
                                      0.3, 0.0, 0.0, 1.0, 1.0, "debug", 1);
      // 可视化计算出来的点和实际的位置
      visualizer_ptr_->visualizePoint(output_x_, output_y_, 0.0,
                                      visualization::ShapeType::Point, 0.3, 0.3,
                                      0.3, 1.0, 0.0, 0.0, 1.0, "debug", 2);

      {
        std::lock_guard<std::mutex> lock(find_ball_state_mutex_);
        find_ball_state_ = FindBallState::CONFIRM;
      }
    } catch (...) {
      std::cout << "unvalid input!" << std::endl;
    }
    break;
  case FindBallState::CONFIRM: {
    std::cout << "calculation finished! " << output_x_ << " " << output_y_;
    if (args.empty()) {
      break;
    }

    int choice = std::stoi(args[0]);
    if (choice == 1) {
      std::cout << "enter to go_to" << std::endl;
      {
        std::lock_guard<std::mutex> lock(find_ball_state_mutex_);
        find_ball_state_ = FindBallState::GO_TO;
      }
    } else {
      std::cout << "back to calculate" << std::endl;
      {
        std::lock_guard<std::mutex> lock(find_ball_state_mutex_);
        find_ball_state_ = FindBallState::CALCULATE;
      }
      // 清除
      visualizer_ptr_->removeMarker("debug", 1);
      visualizer_ptr_->removeMarker("debug", 2);
    }
    break;
  }
  case FindBallState::GO_TO:
    std::cout << "go to state, call the planner" << std::endl;
    {
      std::lock_guard<std::mutex> lock(find_ball_state_mutex_);
      find_ball_state_ = FindBallState::APPROACH;
    }
    break;
  case FindBallState::APPROACH:
    std::cout << "final distance: " << std::endl;
    {
      std::lock_guard<std::mutex> lock(find_ball_state_mutex_);
      find_ball_state_ = FindBallState::CALCULATE;
    }
    break;
  }
}

std::string BadmintonRobotFSM::trim(const std::string &str) {
  size_t first = str.find_first_not_of(" \t");
  if (first == std::string::npos)
    return "";
  size_t last = str.find_last_not_of(" \t");
  return str.substr(first, last - first + 1);
}

// 选择下一步
void BadmintonRobotFSM::chooseNext(RobotState state) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state == RobotState::SHOOT_BALL) {
    state = RobotState::HAND_CTRL;
  }
  state_ = static_cast<RobotState>(static_cast<int>(state) + 1);
}

// 选择上一步
void BadmintonRobotFSM::chooseBack(RobotState state) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state == RobotState::HAND_CTRL) {
    return;
  }
  state_ = static_cast<RobotState>(static_cast<int>(state) - 1);
}

void BadmintonRobotFSM::debug_calculate_target_self_to_global(
    double r_x, double r_y, double &g_x, double &g_y,
    const nav_msgs::Odometry &robo_odom) {
  // 从odom获取机器人当前在map下的位姿
  double robot_x = robo_odom.pose.pose.position.x;
  double robot_y = robo_odom.pose.pose.position.y;

  // 提取机器人yaw角
  tf2::Quaternion q(
      robo_odom.pose.pose.orientation.x, robo_odom.pose.pose.orientation.y,
      robo_odom.pose.pose.orientation.z, robo_odom.pose.pose.orientation.w);

  if (std::abs(q.length() - 1.0) > 1e-6) {
    ROS_WARN("Quaternion not normalized! Normalizing...");
    q.normalize();
  }

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // 球在base_link下的相对坐标
  double ball_x_local = r_x;
  double ball_y_local = r_y;

  // 旋转 + 平移，转换到world坐标系
  g_x = robot_x + ball_x_local * cos(yaw) - ball_y_local * sin(yaw);
  g_y = robot_y + ball_x_local * sin(yaw) + ball_y_local * cos(yaw);
}

void BadmintonRobotFSM::tfBroadcast(const geometry_msgs::TransformStamped &tf) {
  // 发布tf变换关系
  static tf2_ros::TransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped transformStamped;

  // 拷贝tf帧
  transformStamped = tf;
  transformStamped.header.stamp = ros::Time::now();

  // 发布 TF 变换
  tf_broadcaster.sendTransform(transformStamped);
}

} // namespace badminton_robot_fsm