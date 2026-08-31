/*****************************************************************************************
 * 固定点悬停 + /position_cmd 覆盖跟踪 控制器（慢速回固定点版）
 *
 * 功能：
 * 1. 程序启动后持续预发布 setpoint，满足 OFFBOARD 进入条件
 * 2. 非 OFFBOARD 模式下，setpoint 跟随当前飞机位置，不会提前“跑向原点”
 * 3. 切入 OFFBOARD 后，目标点从当前实际位置出发，以限速方式慢慢逼近固定点
 * 4. 若收到新鲜 /position_cmd，则切换为轨迹跟踪（位置+速度前馈+yaw）
 * 5. 若 /position_cmd 断流，则从当前实际位置重新开始，慢慢回固定点
 *
 * 适用：
 * - 直接切 OFFBOARD 自动回固定点悬停
 * - 固定点位置可在 launch 中修改
 ******************************************************************************************/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <string>

// ===================== 全局状态 =====================
mavros_msgs::State current_state;

// 当前 odom
double position_x = 0.0, position_y = 0.0, position_z = 0.0;
double current_yaw = 0.0;
bool odom_received = false;

// 固定悬停目标（launch可改）
double hold_x = 0.0;
double hold_y = 0.0;
double hold_z = 1.0;
double hold_yaw = 0.0;

// “慢速逼近”的过渡目标点
double hold_cmd_x = 0.0;
double hold_cmd_y = 0.0;
double hold_cmd_z = 0.0;
bool hold_cmd_inited = false;

// /position_cmd 输入
double cmd_pos_x = 0.0, cmd_pos_y = 0.0, cmd_pos_z = 0.0;
double cmd_vel_x = 0.0, cmd_vel_y = 0.0, cmd_vel_z = 0.0;
double cmd_yaw = 0.0, cmd_yaw_rate = 0.0;
bool cmd_received = false;
ros::Time last_cmd_time;

// 状态边沿检测
bool prev_offboard = false;
bool prev_cmd_fresh = false;

// 参数
double cmd_timeout = 0.25;   // /position_cmd 超时
double ff_scale = 0.3;       // 速度前馈缩放
double max_vxy_ff = 0.3;     // 轨迹跟踪时平面速度前馈限幅
double max_vz_ff  = 0.15;    // 轨迹跟踪时垂向速度前馈限幅

// 回固定点的速度限制（重点）
double hold_vxy = 0.20;      // 回固定点时平面最大逼近速度 m/s
double hold_vz  = 0.10;      // 回固定点时垂向最大逼近速度 m/s

// ===================== 回调函数 =====================
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    position_x = msg->pose.pose.position.x;
    position_y = msg->pose.pose.position.y;
    position_z = msg->pose.pose.position.z;

    tf2::Quaternion quat;
    tf2::convert(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_yaw = yaw;

    odom_received = true;
}

void cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    cmd_pos_x = msg->position.x;
    cmd_pos_y = msg->position.y;
    cmd_pos_z = msg->position.z;

    cmd_vel_x = msg->velocity.x;
    cmd_vel_y = msg->velocity.y;
    cmd_vel_z = msg->velocity.z;

    cmd_yaw = msg->yaw;
    cmd_yaw_rate = msg->yaw_dot;

    cmd_received = true;
    last_cmd_time = ros::Time::now();
}

// ===================== 工具函数 =====================
double clamp(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

bool cmd_is_fresh()
{
    if (!cmd_received) return false;
    return (ros::Time::now() - last_cmd_time).toSec() < cmd_timeout;
}

// 用当前实际位置初始化“慢速逼近目标”
void reset_hold_cmd_to_current_pose()
{
    hold_cmd_x = position_x;
    hold_cmd_y = position_y;
    hold_cmd_z = position_z;
    hold_cmd_inited = true;
}

// 让 hold_cmd 以限速方式慢慢逼近固定点 hold_x/hold_y/hold_z
void update_hold_cmd(double dt)
{
    if (!hold_cmd_inited)
    {
        reset_hold_cmd_to_current_pose();
    }

    double dx = hold_x - hold_cmd_x;
    double dy = hold_y - hold_cmd_y;
    double dz = hold_z - hold_cmd_z;

    // 平面限速逼近
    double dxy = std::sqrt(dx * dx + dy * dy);
    double max_step_xy = hold_vxy * dt;

    if (dxy > 1e-6)
    {
        double step_xy = std::min(dxy, max_step_xy);
        hold_cmd_x += step_xy * dx / dxy;
        hold_cmd_y += step_xy * dy / dxy;
    }

    // z方向限速逼近
    double adz = std::fabs(dz);
    double max_step_z = hold_vz * dt;

    if (adz > 1e-6)
    {
        double step_z = std::min(adz, max_step_z);
        hold_cmd_z += (dz > 0.0 ? step_z : -step_z);
    }
}

// ===================== Setpoint 生成 =====================

// 非 OFFBOARD 时：持续发布“当前实际位置”
// 用于满足切模式前 >2Hz 的要求，同时避免提前跑固定点
mavros_msgs::PositionTarget make_preoffboard_sp()
{
    mavros_msgs::PositionTarget sp;
    sp.header.stamp = ros::Time::now();
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // 仅 position + yaw
    sp.type_mask =
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    sp.position.x = position_x;
    sp.position.y = position_y;
    sp.position.z = position_z;
    sp.yaw = current_yaw;
    sp.yaw_rate = 0.0;

    return sp;
}

// 固定点悬停：发布“慢速逼近后的过渡点”
mavros_msgs::PositionTarget make_fixed_hold_sp()
{
    mavros_msgs::PositionTarget sp;
    sp.header.stamp = ros::Time::now();
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // 仅 position + yaw
    sp.type_mask =
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    sp.position.x = hold_cmd_x;
    sp.position.y = hold_cmd_y;
    sp.position.z = hold_cmd_z;
    sp.yaw = hold_yaw;
    sp.yaw_rate = 0.0;

    return sp;
}

// 轨迹跟踪：位置 + 速度前馈 + yaw
mavros_msgs::PositionTarget make_track_sp()
{
    mavros_msgs::PositionTarget sp;
    sp.header.stamp = ros::Time::now();
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    sp.type_mask =
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    sp.position.x = cmd_pos_x;
    sp.position.y = cmd_pos_y;
    sp.position.z = cmd_pos_z;

    sp.velocity.x = clamp(ff_scale * cmd_vel_x, -max_vxy_ff, max_vxy_ff);
    sp.velocity.y = clamp(ff_scale * cmd_vel_y, -max_vxy_ff, max_vxy_ff);
    sp.velocity.z = clamp(ff_scale * cmd_vel_z, -max_vz_ff,  max_vz_ff);

    sp.yaw = cmd_yaw;
    sp.yaw_rate = 0.0;

    return sp;
}

// ===================== 主函数 =====================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cxr_egoctrl_v2");
    ros::NodeHandle nh("~");

    // 固定点参数
    nh.param("hold_x", hold_x, 0.0);
    nh.param("hold_y", hold_y, 0.0);
    nh.param("hold_z", hold_z, 1.0);
    nh.param("hold_yaw", hold_yaw, 0.0);

    // 回固定点速度限制（重点）
    nh.param("hold_vxy", hold_vxy, 0.20);
    nh.param("hold_vz", hold_vz, 0.10);

    // /position_cmd 跟踪参数
    nh.param("cmd_timeout", cmd_timeout, 0.25);
    nh.param("ff_scale", ff_scale, 0.30);
    nh.param("max_vxy_ff", max_vxy_ff, 0.30);
    nh.param("max_vz_ff", max_vz_ff, 0.15);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 20, state_cb);

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", 50, odom_cb);

    ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
        "/position_cmd", 50, cmd_cb);

    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 50);

    ros::Rate rate(50.0);
    const double dt = 1.0 / 50.0;

    ROS_INFO("Waiting FCU connection...");
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected.");

    ROS_INFO("Waiting odom...");
    while (ros::ok() && !odom_received)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Odom received.");

    // 初始化过渡点为当前飞机位置
    reset_hold_cmd_to_current_pose();

    ROS_INFO("Fixed hold target: x=%.2f y=%.2f z=%.2f yaw=%.2f",
             hold_x, hold_y, hold_z, hold_yaw);
    ROS_INFO("Slow return speed: hold_vxy=%.2f m/s, hold_vz=%.2f m/s",
             hold_vxy, hold_vz);
    ROS_INFO("Controller running. Switch to OFFBOARD when ready.");

    while (ros::ok())
    {
        ros::spinOnce();

        bool in_offboard = (current_state.mode == "OFFBOARD");
        bool cmd_fresh = cmd_is_fresh();

        // 边沿1：刚切入 OFFBOARD
        if (in_offboard && !prev_offboard && odom_received)
        {
            // 从当前实际位置开始慢慢回固定点，避免瞬间跳
            reset_hold_cmd_to_current_pose();
            ROS_INFO("Entered OFFBOARD. Start moving slowly to fixed hold point.");
        }

        // 边沿2：轨迹命令刚断流
        if (in_offboard && prev_cmd_fresh && !cmd_fresh && odom_received)
        {
            // 从当前实际位置重新开始慢慢回固定点
            reset_hold_cmd_to_current_pose();
            ROS_WARN("Command timeout. Return slowly to fixed hold point.");
        }

        mavros_msgs::PositionTarget sp;

        if (!in_offboard)
        {
            // 非 OFFBOARD：预发布当前位置
            sp = make_preoffboard_sp();

            ROS_INFO_THROTTLE(
                1.0,
                "[PRE-OFFBOARD] follow current pose=(%.2f %.2f %.2f) yaw=%.2f",
                sp.position.x, sp.position.y, sp.position.z, sp.yaw
            );
        }
        else
        {
            if (cmd_fresh)
            {
                // 有新鲜轨迹命令：跟踪 /position_cmd
                sp = make_track_sp();

                ROS_INFO_THROTTLE(
                    0.5,
                    "[TRACK] pos=(%.2f %.2f %.2f) vel_ff=(%.2f %.2f %.2f) yaw=%.2f",
                    sp.position.x, sp.position.y, sp.position.z,
                    sp.velocity.x, sp.velocity.y, sp.velocity.z,
                    sp.yaw
                );
            }
            else
            {
                // 无轨迹命令：慢速回固定点
                update_hold_cmd(dt);
                sp = make_fixed_hold_sp();

                ROS_INFO_THROTTLE(
                    1.0,
                    "[FIXED HOLD] cmd=(%.2f %.2f %.2f) target=(%.2f %.2f %.2f) odom=(%.2f %.2f %.2f)",
                    sp.position.x, sp.position.y, sp.position.z,
                    hold_x, hold_y, hold_z,
                    position_x, position_y, position_z
                );
            }
        }

        local_pos_pub.publish(sp);

        prev_offboard = in_offboard;
        prev_cmd_fresh = cmd_fresh;

        rate.sleep();
    }

    return 0;
}