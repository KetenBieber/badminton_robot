#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class OdomToTfNode {
private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  tf::TransformBroadcaster br_;

public:
  OdomToTfNode() {
    // 订阅 /odom 话题
    odom_sub_ = nh_.subscribe("/odom", 10, &OdomToTfNode::odomCallback, this);
    ROS_INFO("OdomToTf node started, listening to /odom topic");
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // 从 Odometry 消息中提取位置和姿态信息
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z));

    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    transform.setRotation(q);

    // 定义mid360到base_link的静态变换
    tf::Transform transform_mid360_to_base_link;
    transform_mid360_to_base_link.setOrigin(tf::Vector3(0.285, -0.14, 0.0));
    tf::Quaternion q_mid360_to_base_link;
    q_mid360_to_base_link.setRPY(0, 0, 0); // yaw, pitch, roll
    transform_mid360_to_base_link.setRotation(q_mid360_to_base_link);

    // 计算 odom -> base_link 的最终变换
    tf::Transform transform_odom_to_base_link =
        transform * transform_mid360_to_base_link;

    // 发布 tf 变换：odom -> base_link
    br_.sendTransform(tf::StampedTransform(transform_odom_to_base_link,
                                           msg->header.stamp,
                                           msg->header.frame_id, // odom
                                           msg->child_frame_id   // base_link
                                           ));
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_odom_to_baselink_node");
  OdomToTfNode node;
  ros::spin();
  return 0;
}