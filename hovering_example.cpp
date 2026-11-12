/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Based on the RotorS hovering example from ETH Zurich ASL.
 * Modified for this project to add waypoint and heading interpolation,
 * ArUco-assisted alignment and descent, and climb-out.
 * See licenses/Apache-2.0.txt for the full license.
 */

#include <thread>
#include <chrono>
#include <vector>
#include <cmath> // 必须包含cmath以使用M_PI
#include <std_msgs/Bool.h>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <geometry_msgs/PoseStamped.h>  // 新增：用于ArUco位置信息
#include <nav_msgs/Odometry.h>

Eigen::Vector3d aruco_offset;           // 存储ArUco位置偏差
bool aruco_updated = false;             // 标记ArUco数据是否更新
nav_msgs::Odometry current_odom;
Eigen::Quaterniond current_orientation;
bool odom_updated = false;
bool is_target_detected_ = false;
void detectionCallback(const std_msgs::Bool::ConstPtr& msg) {
        // 实时更新检测状态
        is_target_detected_ = msg->data;
        ROS_INFO_COND(msg->data, "Target detection updated: TRUE");
    }
// ArUco位置回调函数
void arucoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    aruco_offset.x() = msg->pose.position.x;
    aruco_offset.y() = msg->pose.position.y;
    aruco_offset.z() = msg->pose.position.z;
    aruco_updated = true;
    ROS_INFO_STREAM("Raw camera frame position: " 
        << msg->pose.position.x << ", "
        << msg->pose.position.y << ", "
        << msg->pose.position.z);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom = *msg;
    current_orientation = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );
    odom_updated = true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "waypoint_example");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(4); // 使用4个线程
    spinner.start();

  // 新增：订阅ArUco位置信息
    ros::Subscriber aruco_sub = nh.subscribe("/aruco_detector/pose", 1, arucoPoseCallback);
    ros::Subscriber odom_sub = nh.subscribe("/hummingbird/odometry_sensor1/odometry", 10, odomCallback);
    ros::Subscriber detect_result = nh.subscribe("/detection_result", 1, detectionCallback);
  double adjust_gain, descend_rate, landing_height;
  nh.param("adjust_gain", adjust_gain, 0.3);
  nh.param("descend_rate", descend_rate, 0.15);
  nh.param("landing_height", landing_height, 0.47);


  ros::Publisher trajectory_pub =
  nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
  mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started waypoint example.");


  // 航点定义（已修正语法）
  std::vector<Eigen::Vector3d> waypoints = {
  {0, 0, 0},
  {0, 0, 0.8},
  {-2.6, -0.3, 1.6},
  {-3.6, 1.3, 1.6},
  {-4.0, 2.5, 1.6},
  {-4.0, 3.5, 1.6},
  {-4.0, 4.5, 1.5},
  {-3.7, 5.8, 1.4},
  {-2.6, 5.8, 1.1}, // 索引8（第9个点）
  {0.2, 6.5, 1.3}, // 索引9（第10个点）
  {0.2, 6.8, 1.5},//11
  {2.7, 6.0, 1.5},//12
  {2.7, 5.2, 1.5},//13
  {1.7, 4.8, 1.5},
  {1.7, 4.2, 1.5},//15
  {2.7, 4.0, 1.5},
  {2.7, 3.0, 1.5},//17
  {1.7, 3.0, 1.5},
  {1.7, 2.0, 1.5},//19
  {2.7, 2.0, 1.5},
  {2.7, 1.0, 1.5},//21

  {2.1, 1.0, 2.3},
  {2.1, 1.5, 2.3}, // 索引22（第25个点）
  {2.1, 1.5, 1.5},
  {2.1, 1.0, 2},
  {-0.0, 0.0, 0.0}
  };

// Yaw角度配置（弧度）
std::vector<double> yaws(waypoints.size(), 0.0);
yaws[6] = M_PI/3;
yaws[7] = M_PI/4;
yaws[8] = M_PI/2; // 第9个点（索引8）设置90度
//yaws[9] = M_PI/4;
//yaws[10] = M_PI/3; // 第10个点（索引9）设置60度

// 时间配置系统（秒）
std::vector<bool> should_stay(waypoints.size(), false);
should_stay[9] = true; // 索引3的航点不停留
should_stay[8] = true; // 索引5的航点不停留


std::vector<double> move_durations(waypoints.size()-1, 0.6); // 段间移动时间
std::vector<double> stay_durations(waypoints.size(), 0.7); // 点停留时间

for (size_t i = 0; i < should_stay.size(); ++i) {
if (!should_stay[i]) {
stay_durations[i] = 0.0; // 设置停留时间为0
// 可同时缩短前段移动时间
if (i > 0) move_durations[i-1] = 0.8;
}
}

// 特殊时间设置
//stay_durations[5] = 1.0;
//stay_durations[7] = 1.0;
stay_durations[1] = 1.0; // 初始点停留
stay_durations[8] = 3; // 第9个点停留
stay_durations[9] = 0.8;
stay_durations[22] = 2;

move_durations[4] = 0.4;
move_durations[5] = 0.4;

move_durations[7] = 1.0; // 第8到第9个点的移动时间（索引7）
move_durations[8] = 3; // 第9到第10个点的移动时间（索引8）
move_durations[9] = 2;
move_durations[12] = 0.68;
move_durations[13] = 0.68; // 第13到第14个点的移动时间（索引12）
move_durations[14] = 0.68; // 第14到第15个点的移动时间（索引13）
move_durations[15] = 0.68; // 第15到第16个点的移动时间（索引14）
move_durations[16] = 0.68; // 第16到第17个点的移动时间（索引15）
move_durations[17] = 0.68; // 第17到第18个点的移动时间（索引16）
move_durations[18] = 0.68; // 第18到第19个点的移动时间（索引17）
move_durations[19] = 0.68; // 第19到第20个点的移动时间（索引18）
move_durations[21] = 1; // 第20到第21个点的移动时间（索引19）
move_durations[22] = 1;
move_durations[23] = 1;

const int LANDING_WAYPOINT_INDEX = 9; 
const int LANDING_WAYPOINT_INDEX2 = 25; 
// 数据一致性检查
if (waypoints.size() != yaws.size() || waypoints.size() != stay_durations.size()) {
ROS_FATAL("Array size mismatch!");
return -1;
}

// 等待系统稳定
ros::Duration(2.0).sleep();

// 发布初始航点
{
trajectory_msgs::MultiDOFJointTrajectory msg;
mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(waypoints[0], yaws[0], &msg);
msg.header.stamp = ros::Time::now();
trajectory_pub.publish(msg);
ROS_INFO("Published initial waypoint, staying for %.1f seconds", stay_durations[0]);
ros::Duration(stay_durations[0]).sleep();
}

// 主轨迹发布循环
for (size_t i = 1; i < waypoints.size(); ++i) {
const int segment_index = i-1;
const double segment_duration = move_durations[segment_index];
const int interpolation_steps = 8;

// 插值参数计算
const Eigen::Vector3d pos_start = waypoints[i-1];
const Eigen::Vector3d pos_end = waypoints[i];
const double yaw_start = yaws[i-1];
const double yaw_end = yaws[i];

// 插值发布
for (int step = 1; step <= interpolation_steps; ++step) {
const double ratio = static_cast<double>(step)/interpolation_steps;

// 位置插值
const Eigen::Vector3d interp_pos = pos_start + ratio*(pos_end - pos_start);

// 角度插值（使用角度差最短路径）
const double yaw_diff = yaw_end - yaw_start;
const double shortest_yaw = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff));
const double interp_yaw = yaw_start + ratio*shortest_yaw;
// 构造并发布消息
trajectory_msgs::MultiDOFJointTrajectory msg;
mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(interp_pos, interp_yaw, &msg);
msg.header.stamp = ros::Time::now();
trajectory_pub.publish(msg);

// 计算并限制角速度
static double last_yaw = yaws[0];
const double yaw_rate = std::abs(interp_yaw - last_yaw)/(segment_duration/interpolation_steps);
last_yaw = interp_yaw;

if (yaw_rate > (M_PI/4)) { // 限制最大45度/秒
ROS_WARN("High yaw rate detected: %.1f deg/s at step %d",
yaw_rate*180/M_PI, step);
}

// 休眠控制
ros::Duration(segment_duration/interpolation_steps).sleep();
ros::spinOnce();
}

// ============ 新增降落处理逻辑 ============
    // ============ 修正后的降落逻辑 ============
    if (i == LANDING_WAYPOINT_INDEX) {
        ROS_WARN("[ArUco] Landing procedure initiated at waypoint %zu", i);

        // 阶段0：等待初始定位
        ros::Time wait_start = ros::Time::now();
        while (!odom_updated && (ros::Time::now() - wait_start).toSec() < 5.0) {
            ROS_WARN_THROTTLE(1, "Waiting for odometry data...");
            ros::Duration(0.1).sleep();
        }
        if (!odom_updated) {
            ROS_ERROR("Odometry not available! Aborting landing.");
            continue;
        }

        static Eigen::Vector3d last_offset = Eigen::Vector3d::Zero();  // 初始化上一次偏移
        ros::Time stable_start;
        // 阶段1：视觉辅助精调（带坐标系转换）
        ros::Time adjust_start = ros::Time::now();
        while ((ros::Time::now() - adjust_start).toSec() < 3 && ros::ok()) {
            if (aruco_updated && odom_updated) {
                // 坐标系转换
                Eigen::Vector3d camera_offset(0.019, 0.0, -0.045); // URDF中的安装偏移
                Eigen::Vector3d body_offset = aruco_offset + camera_offset;

                // 应用摄像头本体旋转（通过current_orientation）
                body_offset = current_orientation * body_offset;

                ROS_INFO_STREAM("Body frame offset: " << body_offset.transpose());

                // 坐标轴重映射（根据安装方向）
                Eigen::Vector3d world_offset(
                    body_offset.z(),   // 摄像头Z轴→机体前向X
                    -body_offset.y(),  // 摄像头Y轴→机体右向Y（反向）
                    -body_offset.x()   // 摄像头X轴→机体下方Z
                    );

                double dynamic_gain_x = 0.31 * tanh(world_offset.norm());
                double dynamic_gain_y = 0.12 * tanh(world_offset.norm());

               
                if (world_offset.x() * last_offset.x() < 0 ) {
                    dynamic_gain_x *= -0.7;
                    ROS_WARN(" Overshoot X detected! Reverse adjustment");
                }
                if (world_offset.y() * last_offset.y() < 0 ) {
                    dynamic_gain_y *= -0.9;
                    ROS_WARN(" Overshoot Y detected! Reverse adjustment");
                }
                last_offset = world_offset;

                // 带限幅的位置调整
                Eigen::Vector3d target_pos(
                    current_odom.pose.pose.position.x,
                    current_odom.pose.pose.position.y,
                    current_odom.pose.pose.position.z
                );
                target_pos.x() += world_offset.x() * dynamic_gain_x;
                target_pos.y() += world_offset.y() * dynamic_gain_y;

                // 高度保持
                target_pos.z() = waypoints[i].z();

                // 发布调整指令
                ROS_INFO_STREAM("World Adjustment: " << world_offset.transpose());
                ROS_INFO_STREAM("Target Position: " << target_pos.transpose());

                if (world_offset.norm() < 0.1) { // 持续稳定判断
            if (!stable_start.isValid()) stable_start = ros::Time::now();
            if ((ros::Time::now() - stable_start).toSec() > 1.0) {
                ROS_INFO("[成功] 位置已稳定(误差=%.2fm)", world_offset.norm());
                break;
            }
            }

                trajectory_msgs::MultiDOFJointTrajectory msg;
                mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
                    target_pos, yaws[i], &msg);
                msg.header.stamp = ros::Time::now();
                trajectory_pub.publish(msg);

                aruco_updated = false; // 新增：处理完成后重置标志
                odom_updated = false;  // 新增：处理完成后重置标志

                ROS_DEBUG_STREAM("Adjusting to: " << target_pos.transpose());
            }
            ros::Duration(0.05).sleep();
            ros::spinOnce();
        }

        // 阶段2：安全缓降
        ROS_WARN("[ArUco] Starting controlled descent");
        double current_z = current_odom.pose.pose.position.z;
        const double descend_step = 0.1;  // 5cm步长
        const double min_height = landing_height;

        while (current_z > min_height && ros::ok()) {
            // 获取实时高度
            current_z = current_odom.pose.pose.position.z;
            
            // 计算目标高度
            double target_z = std::max(current_z - descend_step, min_height);
            
            // 构造目标位置
            Eigen::Vector3d target_pos(
                current_odom.pose.pose.position.x,
                current_odom.pose.pose.position.y,
                target_z
            );

            // 发布指令
            trajectory_msgs::MultiDOFJointTrajectory msg;
            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
                target_pos, yaws[i], &msg);
            trajectory_pub.publish(msg);

            // 安全检测
            if (current_z - target_z > descend_step * 2) {
                ROS_ERROR("Abnormal descent detected! Aborting.");
                break;
            }

            ros::Duration(0.5).sleep();
        }

        // 阶段3：着陆确认
        ROS_INFO("[ArUco] Landing confirmed at %.2fm", landing_height);
        ros::Duration(0.5).sleep();

        // 阶段4：复飞
        ROS_WARN("[ArUco] Executing takeoff");
        trajectory_msgs::MultiDOFJointTrajectory msg;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
            waypoints[i], yaws[i], &msg);
        trajectory_pub.publish(msg);
        
        // 等待达到目标高度
        ros::Time takeoff_start = ros::Time::now();
        while (ros::ok() && 
              (current_odom.pose.pose.position.z < waypoints[i].z() - 0.1) &&
              (ros::Time::now() - takeoff_start).toSec() < 10.0) {
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
    }

    if (i == LANDING_WAYPOINT_INDEX2) {
        ros::Duration(5).sleep();
        if(is_target_detected_){
        ROS_WARN("Landing procedure initiated at waypoint %zu", i);

            trajectory_msgs::MultiDOFJointTrajectory msg;
            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
                waypoints[i], yaws[i], &msg);
            msg.header.stamp = ros::Time::now();
            trajectory_pub.publish(msg);
            ros::Duration(0.05).sleep();
            ros::spinOnce();
        }

    }

ROS_INFO("Reached waypoint %zu, staying for %.1f seconds", i, stay_durations[i]);
ros::Duration(stay_durations[i]).sleep();
}

ROS_INFO("All waypoints completed.");
return 0;
}