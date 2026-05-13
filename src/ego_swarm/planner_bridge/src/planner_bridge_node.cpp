#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

static constexpr float kDefaultAltitude = 0.6;

// 发布器
static ros::Publisher setpoint_pub;
static ros::Publisher goal_pub;
mavros_msgs::PositionTarget setpoint_raw_local;

// 当前飞控状态
static mavros_msgs::State current_state;

// 当前无人机位姿
geometry_msgs::PoseStamped current_pose;

// 飞控状态回调
void state_callback(const mavros_msgs::State::ConstPtr& msg) {
  current_state = *msg;
}

// 本地位姿回调
float init_pose_x = 0.0;
float init_pose_y = 0.0;
float init_pose_z = 0.0;
float init_pose_yaw = 0.0;
bool init_pose_flag = false;
void local_pose_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  current_pose.header = msg->header;
  current_pose.pose = msg->pose.pose;
  if (!init_pose_flag && current_pose.pose.position.x != 0 &&
      current_pose.pose.position.y != 0 && current_pose.pose.position.z != 0) {
    init_pose_x = msg->pose.pose.position.x;
    init_pose_y = msg->pose.pose.position.y;
    init_pose_z = msg->pose.pose.position.z;
    init_pose_yaw = tf::getYaw(msg->pose.pose.orientation);
    init_pose_flag = true;
  }
}

float ego_pos_x = 0.0;
float ego_pos_y = 0.0;
float ego_pos_z = 0.0;
float ego_vel_x = 0.0;
float ego_vel_y = 0.0;
float ego_vel_z = 0.0;
float ego_acc_x = 0.0;
float ego_acc_y = 0.0;
float ego_acc_z = 0.0;
float ego_yaw = 0.0;
float ego_yaw_rate = 0.0;
void position_command_callback(
    const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
  ego_pos_x = msg->position.x;
  ego_pos_y = msg->position.y;
  ego_pos_z = msg->position.z;
  ego_vel_x = msg->velocity.x;
  ego_vel_y = msg->velocity.y;
  ego_vel_z = msg->velocity.z;
  ego_acc_x = msg->acceleration.x;
  ego_acc_y = msg->acceleration.y;
  ego_acc_z = msg->acceleration.z;
  ego_yaw = msg->yaw;
  ego_yaw_rate = msg->yaw_dot;
}

// 判断是否已接近目标点（位置 + 偏航角）
bool is_near_target(double x, double y, double z, double yaw_rad,
                    double pos_tolerance = 0.3, double yaw_tolerance = 6.28) {
  double dx = current_pose.pose.position.x - x;
  double dy = current_pose.pose.position.y - y;
  double dz = current_pose.pose.position.z - z;
  double dist_sq = dx * dx + dy * dy + dz * dz;

  double current_yaw = tf::getYaw(current_pose.pose.orientation);
  double yaw_diff =
      std::fabs(angles::shortest_angular_distance(current_yaw, yaw_rad));

  return (dist_sq < pos_tolerance * pos_tolerance && yaw_diff < yaw_tolerance);
}

// 发布导航目标
bool publish_ego_navigation_goal(double x, double y, double z,
                                 double yaw_rad = 0.0) {
  static int publish_counter = 0;
  static double last_x = 0, last_y = 0, last_z = 0, last_yaw = 0;

  // 如果目标变化，重置发布次数
  if (std::fabs(last_x - x) > 1e-3 || std::fabs(last_y - y) > 1e-3 ||
      std::fabs(last_z - z) > 1e-3 || std::fabs(last_yaw - yaw_rad) > 1e-3) {
    last_x = x;
    last_y = y;
    last_z = z;
    last_yaw = yaw_rad;
    publish_counter = 30;  // 连续发布30次
  }

  if (publish_counter > 0 && !is_near_target(x + init_pose_x, y + init_pose_y,
                                             z + init_pose_z, yaw_rad)) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = x + init_pose_x;
    goal.pose.position.y = y + init_pose_y;
    goal.pose.position.z = z + init_pose_z;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_rad);

    goal_pub.publish(goal);
    --publish_counter;

    ROS_INFO_THROTTLE(
        1, "[导航] 发布目标 x=%.2f y=%.2f z=%.2f yaw=%.2f [%d 次剩余]", x, y, z,
        yaw_rad, publish_counter);
  }

  setpoint_raw_local.coordinate_frame =
      mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  setpoint_raw_local.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                 mavros_msgs::PositionTarget::IGNORE_PY |
                                 mavros_msgs::PositionTarget::IGNORE_VZ |
                                 mavros_msgs::PositionTarget::IGNORE_AFX |
                                 mavros_msgs::PositionTarget::IGNORE_AFY |
                                 mavros_msgs::PositionTarget::IGNORE_AFZ |
                                 mavros_msgs::PositionTarget::FORCE |
                                 mavros_msgs::PositionTarget::IGNORE_YAW;
  setpoint_raw_local.velocity.x = ego_vel_x;
  setpoint_raw_local.velocity.y = ego_vel_y;
  setpoint_raw_local.position.z = init_pose_z + kDefaultAltitude;
  setpoint_raw_local.yaw_rate = ego_yaw_rate;

  if (publish_counter == 0 && is_near_target(x + init_pose_x, y + init_pose_y,
                                             z + init_pose_z, yaw_rad)) {
    ROS_INFO_THROTTLE(2, "[导航] 已到达目标点附近，等待下一个任务...");
  }
  return publish_counter == 0 &&
         is_near_target(x + init_pose_x, y + init_pose_y, z + init_pose_z,
                        yaw_rad);
}

int main(int argc, char** argv) {
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "planner_bridge_node");
  ros::NodeHandle nh;

  // 初始化订阅器
  ros::Subscriber state_sub = nh.subscribe("/mavros/state", 10, state_callback);
  ros::Subscriber pose_sub =
      nh.subscribe("/mavros/local_position/odom", 10, local_pose_callback);
  ros::Subscriber command_sub =
      nh.subscribe("/drone_0_planning/pos_cmd", 10, position_command_callback);

  // 初始化发布器
  setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>(
      "/mavros/setpoint_raw/local", 10);
  goal_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  // 初始化服务客户端
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  ros::ServiceClient arm_client =
      nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  ros::Rate rate(20.0);

  // // 测试目标点发布（仅用于验证）
  // while (ros::ok()) {
  //   publish_ego_navigation_goal(3.0, 0.0, 0.5, 0.0);
  //   ros::spinOnce();
  //   rate.sleep();
  // }
  // ros::shutdown();
  // // 测试目标点发布（仅用于验证）

  // 以下为 OFFBOARD 模式启动流程
  while (ros::ok() && !current_state.connected) {
    ROS_INFO_STREAM_THROTTLE(
        2, "[飞控] 等待连接到飞控... 当前状态: " << current_state.mode);
    ros::spinOnce();
    rate.sleep();
  }

  setpoint_raw_local.coordinate_frame =
      mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  setpoint_raw_local.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                 mavros_msgs::PositionTarget::IGNORE_PY |
                                 mavros_msgs::PositionTarget::IGNORE_PZ |
                                 mavros_msgs::PositionTarget::IGNORE_VX |
                                 mavros_msgs::PositionTarget::IGNORE_VY |
                                 mavros_msgs::PositionTarget::IGNORE_VZ |
                                 mavros_msgs::PositionTarget::IGNORE_AFX |
                                 mavros_msgs::PositionTarget::IGNORE_AFY |
                                 mavros_msgs::PositionTarget::IGNORE_AFZ |
                                 mavros_msgs::PositionTarget::FORCE |
                                 mavros_msgs::PositionTarget::IGNORE_YAW |
                                 mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  setpoint_raw_local.position.x = init_pose_x;
  setpoint_raw_local.position.y = init_pose_y;
  setpoint_raw_local.position.z = init_pose_z + kDefaultAltitude;
  setpoint_raw_local.yaw = init_pose_yaw;

  for (int i = 0; i < 100 && ros::ok(); ++i) {
    setpoint_pub.publish(setpoint_raw_local);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode px4_mode_req;
  px4_mode_req.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_req;
  arm_req.request.value = true;
  bool is_offboard_enabled_once = false;

  ros::Time last_req_time = ros::Time::now();

  while (ros::ok()) {
    if (!is_offboard_enabled_once && current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_req_time > ros::Duration(5.0))) {
      if (set_mode_client.call(px4_mode_req) &&
          px4_mode_req.response.mode_sent) {
        init_pose_flag = false;
        is_offboard_enabled_once = true;
        ROS_INFO("[飞控] 请求切换至 OFFBOARD 模式");
      }
      last_req_time = ros::Time::now();
    } else if (!current_state.armed &&
               (ros::Time::now() - last_req_time > ros::Duration(5.0))) {
      if (arm_client.call(arm_req) && arm_req.response.success) {
        ROS_INFO("[飞控] 飞机已解锁");
      }
      last_req_time = ros::Time::now();
    }

    setpoint_raw_local.coordinate_frame =
        mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint_raw_local.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                   mavros_msgs::PositionTarget::IGNORE_VY |
                                   mavros_msgs::PositionTarget::IGNORE_VZ |
                                   mavros_msgs::PositionTarget::IGNORE_AFX |
                                   mavros_msgs::PositionTarget::IGNORE_AFY |
                                   mavros_msgs::PositionTarget::IGNORE_AFZ |
                                   mavros_msgs::PositionTarget::FORCE |
                                   mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    setpoint_raw_local.position.x = init_pose_x;
    setpoint_raw_local.position.y = init_pose_y;
    setpoint_raw_local.position.z = init_pose_z + kDefaultAltitude;
    setpoint_raw_local.yaw = init_pose_yaw;
    setpoint_pub.publish(setpoint_raw_local);
    if (current_state.mode == "OFFBOARD" &&
        is_near_target(init_pose_x, init_pose_y, init_pose_z + kDefaultAltitude,
                       init_pose_yaw, 0.2, 0.2) &&
        (ros::Time::now() - last_req_time > ros::Duration(8.0))) {
      ROS_INFO("[飞控] 已到达起飞点，等待下一个任务...");
      break;
    } else {
      ROS_INFO_THROTTLE(1, "[飞控] 起飞中...");
    }
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok()) {
    if (publish_ego_navigation_goal(3.0, 0.0, kDefaultAltitude, 0.0)) {
      break;
    }
    setpoint_pub.publish(setpoint_raw_local);
    ros::spinOnce();
    rate.sleep();
  }

  ros::shutdown();
  ROS_INFO("[ROS] planner_bridge_node 已关闭");
  return 0;
}
