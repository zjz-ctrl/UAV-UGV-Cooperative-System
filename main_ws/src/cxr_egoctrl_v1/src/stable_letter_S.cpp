#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <algorithm>
#include <cmath>

class SLetterTrajectoryNode
{
public:
    struct BezierSeg {
        double p0, p1;
        double Px[4], Py[4];
    };

    SLetterTrajectoryNode(ros::NodeHandle& nh)
        : nh_(nh), px_(0.0), py_(0.0), pz_(0.0), has_odom_(false)
    {
        cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);
        odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 1,
                                  &SLetterTrajectoryNode::odomCallback, this);

        // 提高一点频率，让轨迹更顺
        timer_ = nh_.createTimer(ros::Duration(1.0 / 50.0),
                                 &SLetterTrajectoryNode::timerCallback, this);

        nh_.param("start_height", z0_, 0.8);
        nh_.param("total_time", total_time_, 11.0);
        nh_.param("ramp_time", ramp_time_, 2.0);
        nh_.param("hover_time", hover_time_, 3.0);
        nh_.param("r_scale", r_, 0.8);
        nh_.param("vel_scale", vel_scale_, 0.55);
        nh_.param("max_ff_speed", max_ff_speed_, 0.35);

        buildS();

        state_ = WAIT_ODOM;
        ROS_INFO("Optimized Print-Style S Trajectory Node Initialized.");
    }

private:
    enum State { WAIT_ODOM, HOVER, TRAJECTORY, DONE };
    State state_;

    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;

    double cx_, cy_, z0_, r_;
    double px_, py_, pz_;
    bool has_odom_;

    double total_time_;
    double ramp_time_;
    double hover_time_;
    double vel_scale_;
    double max_ff_speed_;

    ros::Time start_time_;
    std::vector<BezierSeg> segs_;

    void addCurve(double p0, double p1,
                  double x0, double x1, double x2, double x3,
                  double y0, double y1, double y2, double y3)
    {
        BezierSeg s;
        s.p0 = p0;
        s.p1 = p1;
        s.Px[0] = x0; s.Px[1] = x1; s.Px[2] = x2; s.Px[3] = x3;
        s.Py[0] = y0; s.Py[1] = y1; s.Py[2] = y2; s.Py[3] = y3;
        segs_.push_back(s);
    }

    void addHover(double p0, double p1, double x, double y)
    {
        addCurve(p0, p1, x, x, x, x, y, y, y, y);
    }

    double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    void buildS()
    {
        segs_.clear();
        double r = r_;

        // 更像印刷体 S：
        // 1) 下半圈先下探
        // 2) 中腰明显收紧
        // 3) 上半圈再鼓出
        // 4) 末端略微下垂，停在 y≈-0.2

        // 第1段：起笔 -> 下半圈外侧
        addCurve(0.00, 0.18,
                0.00 * r, 0.04 * r, 0.12 * r, 0.40 * r,
                0.00 * r,-0.02 * r,-0.68 * r,-0.70 * r);

        // 第2段：下半圈 -> 中腰
        addCurve(0.18, 0.42,
                0.40 * r, 0.74 * r, 0.64 * r, 0.82 * r,
                -0.70 * r,-0.82 * r,-0.12 * r, 0.00 * r);

        // 第3段：中腰 -> 上半圈外侧
        addCurve(0.42, 0.70,
                0.82 * r, 0.98 * r, 1.04 * r, 1.28 * r,
                0.00 * r, 0.10 * r, 0.76 * r, 0.72 * r);

        // 第4段：上半圈 -> 右侧收尾（尾巴略向下）
        addCurve(0.70, 0.90,
                1.28 * r, 1.46 * r, 1.56 * r, 1.48 * r,
                0.72 * r, 0.72 * r, 0.00 * r, -0.25 * r);

        // 第5段：末端停留
        addHover(0.90, 1.00, 1.48 * r, -0.25 * r);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        px_ = msg->pose.pose.position.x;
        py_ = msg->pose.pose.position.y;
        pz_ = msg->pose.pose.position.z;
        has_odom_ = true;
    }

    void cubic_bezier(double t, const double P[4], double& pos)
    {
        double u = 1.0 - t;
        pos = u*u*u*P[0] + 3.0*u*u*t*P[1] + 3.0*u*t*t*P[2] + t*t*t*P[3];
    }

    void cubic_bezier_vel(double t, const double P[4], double dt_ds, double& vel)
    {
        double u = 1.0 - t;
        vel = (3.0*u*u*(P[1]-P[0]) +
               6.0*u*t*(P[2]-P[1]) +
               3.0*t*t*(P[3]-P[2])) * dt_ds;
    }

    void publish_target(double x, double y, double z, double vx, double vy, double vz)
    {
        double vnorm = std::sqrt(vx*vx + vy*vy + vz*vz);
        if (vnorm > max_ff_speed_ && vnorm > 1e-6) {
            double scale = max_ff_speed_ / vnorm;
            vx *= scale;
            vy *= scale;
            vz *= scale;
        }

        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "map";

        cmd.position.x = x;
        cmd.position.y = y;
        cmd.position.z = z;

        cmd.velocity.x = vx * vel_scale_;
        cmd.velocity.y = vy * vel_scale_;
        cmd.velocity.z = vz * vel_scale_;

        cmd.yaw = 0.0;
        cmd.yaw_dot = 0.0;
        cmd_pub_.publish(cmd);
    }

    void timerCallback(const ros::TimerEvent&)
    {
        if (!has_odom_) return;

        // 固定在原点附近绘制
        cx_ = 0.0;
        cy_ = 0.0;

        double now = ros::Time::now().toSec();

        if (state_ == WAIT_ODOM) {
            start_time_ = ros::Time::now();
            state_ = HOVER;
            ROS_INFO("Odometry acquired. Hovering at (0,0,z0)...");
            return;
        }

        if (state_ == HOVER) {
            if (now - start_time_.toSec() >= hover_time_) {
                state_ = TRAJECTORY;
                start_time_ = ros::Time::now();
                ROS_INFO("Executing optimized print-style S trajectory...");
            } else {
                publish_target(cx_, cy_, z0_, 0.0, 0.0, 0.0);
            }
            return;
        }

        if (state_ == TRAJECTORY) {
            double t = now - start_time_.toSec();

            if (t > total_time_ + ramp_time_) {
                state_ = DONE;
                ROS_INFO("Trajectory completed. Holding end point.");
                return;
            }

            double time_velocity =
                (ramp_time_ > 0.0 && t < ramp_time_) ? (t / ramp_time_) : 1.0;

            double ramp_integral =
                (ramp_time_ > 0.0 && t < ramp_time_)
                ? (time_velocity * t / 2.0)
                : (ramp_time_ / 2.0 + (t - ramp_time_));

            double progress = ramp_integral / total_time_;
            if (progress > 1.0) progress = 1.0;
            double d_progress = time_velocity / total_time_;

            double target_x = 0.0, target_y = 0.0;
            double vx = 0.0, vy = 0.0;

            for (const auto& seg : segs_) {
                if (progress >= seg.p0 && progress <= seg.p1) {
                    double local_t =
                        (seg.p1 == seg.p0) ? 0.0 : ((progress - seg.p0) / (seg.p1 - seg.p0));
                    local_t = clamp(local_t, 0.0, 1.0);

                    double dt_ds =
                        (seg.p1 == seg.p0) ? 0.0 : (d_progress / (seg.p1 - seg.p0));

                    cubic_bezier(local_t, seg.Px, target_x);
                    cubic_bezier(local_t, seg.Py, target_y);
                    cubic_bezier_vel(local_t, seg.Px, dt_ds, vx);
                    cubic_bezier_vel(local_t, seg.Py, dt_ds, vy);
                    break;
                }
            }

            publish_target(target_x + cx_, target_y + cy_, z0_, vx, vy, 0.0);
            return;
        }

        if (state_ == DONE) {
            double end_x = 1.48 * r_;
            double end_y = -0.25 * r_;
            publish_target(end_x + cx_, end_y + cy_, z0_, 0.0, 0.0, 0.0);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "s_letter_trajectory_node");
    ros::NodeHandle nh("~");
    SLetterTrajectoryNode node(nh);
    ros::spin();
    return 0;
}