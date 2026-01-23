// clang-format off
/**
 * @file robot_fsm_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 状态机执行
 * @version 0.1
 * @date 2026-01-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note : 上电位于HAND_CTRL状态，等待触发信号开始进入AUTO_READY
 *         FIND_BALL 将开启自转模式，开始搜寻范围之内的球
 *         TAKE_BALL 通知机构进行拾球(在距离目标点距离足够近时触发，此时机器人将会直接开启摄像头跟踪锁定)
 *         FIND_POINT 寻找期望发射球的位置，可以写死在地图中定位的点
 *         SHOOT_BALL 通知机构进行发射动作
 * @versioninfo :
 */
// clang-format on
#include <ros/ros.h>
#include <move_control/BadmintonRobotFSM.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_fsm_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start(); // 将回调机制设置到后台

  /* 实例化handler */
  badminton_robot_fsm::BadmintonRobotFSM handler;

  if (!handler.init(nh)) {
    ROS_ERROR("fsm handler init failed! ");
    return -1;
  }

  ROS_INFO("start to the terminal APP");

  // 这里会阻塞本线程，执行终端任务，持续接收用户输入
  handler.terminalApp();

  ros::waitForShutdown();
  return 0;
}
