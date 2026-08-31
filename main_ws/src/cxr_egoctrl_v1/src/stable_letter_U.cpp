#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <vector>
#include <algorithm>

class StableLetterUNode
{
public:
    struct BezierSeg {
        double p0, p1;
        double Px[4], Py[4];
    };

    StableLetterUNode(ros::NodeHandle& nh) : nh_(nh)
    {
        cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);
        odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 1, &StableLetterUNode::odomCallback, this);
        timer_ = nh_.createTimer(ros::Duration(1.0 / 50.0), &StableLetterUNode::timerCallback, this);

        nh_.param("start_height", z0_, 0.8);
        nh_.param("total_time", total_time_, 15.0);
        nh_.param("hover_time", hover_time_, 3.0);
        nh_.param("vel_scale", vel_scale_, 0.40);
        nh_.param("max_ff_speed", max_ff_speed_, 0.28);

        // 你的坐标逻辑：
        // 上 -> x增大
        // 右 -> y减小
        nh_.param("top_x", top_x_, 1.2);               // U 顶部高度
        nh_.param("right_y_span", right_y_span_, 1.0); // U 左右宽度（体现在 y 负方向）
        nh_.param("side_bottom_x", side_bottom_x_, 0.20); // 两侧竖线落到底部时的 x
        nh_.param("arc_bottom_x", arc_bottom_x_, 0.00);   // 底部弧线最低点的 x

        buildTrajectory();

        state_ = WAIT_ODOM;
        ROS_INFO("Stable Letter U Node Initialized.");
        ROS_INFO("U start near (x=%.2f, y=0), end near (x=%.2f, y=-%.2f)",
                 top_x_, top_x_, right_y_span_);
    }

private:
    enum State { WAIT_ODOM, HOVER, TRAJECTORY, DONE };

    State state_;
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;

    double cx_ = 0.0, cy_ = 0.0, z0_ = 0.8;
    double px_ = 0.0, py_ = 0.0, pz_ = 0.0;
    bool has_odom_ = false;

    double total_time_ = 15.0;
    double hover_time_ = 3.0;
    double vel_scale_ = 0.40;
    double max_ff_speed_ = 0.28;

    double top_x_ = 1.2;
    double right_y_span_ = 1.2;
    double side_bottom_x_ = 0.20;
    double arc_bottom_x_ = 0.00;

    ros::Time start_time_;
    std::vector<BezierSeg> segs_;

    static double clampd(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    void addCurve(double p0, double p1,
                  double x0, double x1, double x2, double x3,
                  double y0, double y1, double y2, double y3)
    {
        BezierSeg s;
        s.p0 = p0; s.p1 = p1;
        s.Px[0] = x0; s.Px[1] = x1; s.Px[2] = x2; s.Px[3] = x3;
        s.Py[0] = y0; s.Py[1] = y1; s.Py[2] = y2; s.Py[3] = y3;
        segs_.push_back(s);
    }

    void addLine(double p0, double p1, double x0, double y0, double x1, double y1)
    {
        addCurve(p0, p1,
                 x0, x0 + (x1 - x0) / 3.0, x0 + 2.0 * (x1 - x0) / 3.0, x1,
                 y0, y0 + (y1 - y0) / 3.0, y0 + 2.0 * (y1 - y0) / 3.0, y1);
    }

    void addHover(double p0, double p1, double x, double y)
    {
        addCurve(p0, p1, x, x, x, x, y, y, y, y);
    }

    void buildTrajectory()
    {
        segs_.clear();

        // 正确 U：
        // 起点左上：(top_x, 0)
        // 左边往下：(side_bottom_x, 0)
        // 底部弧线到右边：(side_bottom_x, -right_y_span)
        // 右边往上到终点：(top_x, -right_y_span)

        // 1) 左侧竖线：左上 -> 左下
        addLine(0.00, 0.22,
                top_x_, 0.0,
                side_bottom_x_, 0.0);

        // 2) 底部弧线：左下 -> 右下
        addCurve(0.22, 0.74,
                 side_bottom_x_, arc_bottom_x_, arc_bottom_x_, side_bottom_x_,
                 0.0,            -0.25 * right_y_span_, -0.75 * right_y_span_, -right_y_span_);

        // 3) 右侧竖线：右下 -> 右上
        addLine(0.74, 0.94,
                side_bottom_x_, -right_y_span_,
                top_x_,         -right_y_span_);

        // 4) 末端停一下
        addHover(0.94, 1.00,
                 top_x_, -right_y_span_);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        px_ = msg->pose.pose.position.x;
        py_ = msg->pose.pose.position.y;
        pz_ = msg->pose.pose.position.z;
        has_odom_ = true;
    }

    void cubicBezier(double t, const double P[4], double& pos)
    {
        double u = 1.0 - t;
        pos = u*u*u*P[0] + 3.0*u*u*t*P[1] + 3.0*u*t*t*P[2] + t*t*t*P[3];
    }

    void cubicBezierVel(double t, const double P[4], double dt_ds, double& vel)
    {
        double u = 1.0 - t;
        vel = (3.0*u*u*(P[1]-P[0]) +
               6.0*u*t*(P[2]-P[1]) +
               3.0*t*t*(P[3]-P[2])) * dt_ds;
    }

    void timeLaw(double tau, double& s, double& ds_dt)
    {
        tau = clampd(tau, 0.0, 1.0);
        double t2 = tau * tau;
        double t3 = t2 * tau;
        double t4 = t3 * tau;
        double t5 = t4 * tau;

        s = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
        ds_dt = (30.0 * t2 - 60.0 * t3 + 30.0 * t4) / total_time_;
    }

    void publishTarget(double x, double y, double z, double vx, double vy, double vz)
    {
        double v_norm = std::sqrt(vx*vx + vy*vy + vz*vz);
        if (v_norm > max_ff_speed_ && v_norm > 1e-6) {
            double scale = max_ff_speed_ / v_norm;
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

        double now = ros::Time::now().toSec();

        if (state_ == WAIT_ODOM) {
            cx_ = px_;
            cy_ = py_;
            start_time_ = ros::Time::now();
            state_ = HOVER;
            ROS_INFO("Odometry acquired. Hovering at U start point...");
            return;
        }

        if (state_ == HOVER) {
            if (now - start_time_.toSec() >= hover_time_) {
                state_ = TRAJECTORY;
                start_time_ = ros::Time::now();
                ROS_INFO("Executing corrected letter U...");
            } else {
                // U 的真正起点：左上
                publishTarget(cx_ + top_x_, cy_ + 0.0, z0_, 0.0, 0.0, 0.0);
            }
            return;
        }

        if (state_ == TRAJECTORY) {
            double t = now - start_time_.toSec();
            if (t >= total_time_) {
                state_ = DONE;
                ROS_INFO("Letter U completed. Holding final point.");
                return;
            }

            double progress = 0.0, d_progress = 0.0;
            timeLaw(t / total_time_, progress, d_progress);

            double target_x = top_x_, target_y = 0.0;
            double vx = 0.0, vy = 0.0;

            for (const auto& seg : segs_) {
                if (progress >= seg.p0 && progress <= seg.p1) {
                    double local_t = (seg.p1 > seg.p0) ? (progress - seg.p0) / (seg.p1 - seg.p0) : 0.0;
                    local_t = clampd(local_t, 0.0, 1.0);
                    double dt_ds = (seg.p1 > seg.p0) ? d_progress / (seg.p1 - seg.p0) : 0.0;

                    cubicBezier(local_t, seg.Px, target_x);
                    cubicBezier(local_t, seg.Py, target_y);
                    cubicBezierVel(local_t, seg.Px, dt_ds, vx);
                    cubicBezierVel(local_t, seg.Py, dt_ds, vy);
                    break;
                }
            }

            publishTarget(cx_ + target_x, cy_ + target_y, z0_, vx, vy, 0.0);
            return;
        }

        if (state_ == DONE) {
            // U 的终点：右上
            publishTarget(cx_ + top_x_, cy_ - right_y_span_, z0_, 0.0, 0.0, 0.0);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stable_letter_u_node");
    ros::NodeHandle nh("~");
    StableLetterUNode node(nh);
    ros::spin();
    return 0;
}