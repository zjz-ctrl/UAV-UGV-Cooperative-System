#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <vector>

class SDUTrajectoryNode
{
public:
    struct BezierSeg {
        double p0, p1; 
        double Px[4], Py[4];
    };

    SDUTrajectoryNode(ros::NodeHandle& nh) : nh_(nh)
    {
        // 初始化发布器和订阅器
        cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);
        odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 1, &SDUTrajectoryNode::odomCallback, this);
        
        // 控制指令下发频率 (30Hz)
        timer_ = nh_.createTimer(ros::Duration(1.0 / 30.0), &SDUTrajectoryNode::timerCallback, this);
        
        // 参数加载
        nh_.param("start_height", z0_, 1.2);
        nh_.param("total_time", total_time_, 40.0); // 轨迹总参考飞行时长 (秒)
        nh_.param("ramp_time", ramp_time_, 3.0);    // 起步加速阶段缓冲时间 (秒)
        
        // 轨迹基准尺寸缩放因子 (控制整体字体的长宽边界)
        r_ = 0.8; 
        
        buildTrajectory();

        state_ = WAIT_ODOM;
        ROS_INFO("SDU Trajectory Node Initialized.");
        ROS_INFO("Configuration: Reference Size r = %.2f, Total Time = %.1fs", r_, total_time_);
    }

private:
    enum State { WAIT_ODOM, HOVER, TRAJECTORY, DONE };
    State state_;
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;

    double cx_, cy_, z0_, r_;
    double px_ = 0, py_ = 0, pz_ = 0;
    bool has_odom_ = false;

    double total_time_;
    double ramp_time_;
    ros::Time start_time_;
    
    std::vector<BezierSeg> segs_;

    // 添加三次贝塞尔曲线段
    void addCurve(double p0, double p1, 
                  double x0, double x1, double x2, double x3,
                  double y0, double y1, double y2, double y3) {
        BezierSeg s;
        s.p0 = p0; s.p1 = p1;
        s.Px[0]=x0; s.Px[1]=x1; s.Px[2]=x2; s.Px[3]=x3;
        s.Py[0]=y0; s.Py[1]=y1; s.Py[2]=y2; s.Py[3]=y3;
        segs_.push_back(s);
    }

    // 将直线转化为三次贝塞尔曲线的控制点
    void addLine(double p0, double p1, double x0, double y0, double x1, double y1) {
        addCurve(p0, p1, 
            x0, x0+(x1-x0)/3.0, x0+2.0*(x1-x0)/3.0, x1,
            y0, y0+(y1-y0)/3.0, y0+2.0*(y1-y0)/3.0, y1);
    }

    // 添加零速悬停段
    // 作用：强制无人机在关键转折点降速至零并收敛位置误差，彻底消除动力学惯性导致的“切角”现象
    void addHover(double p0, double p1, double x, double y) {
        addCurve(p0, p1, x, x, x, x, y, y, y, y);
    }

    // 构建完整 SDU 轨迹拓扑
    void buildTrajectory() {
        double r = r_; 
        
        // 全局进度 progress 区间为 [0.0, 1.0]，按比例分配时间切片

        // ========== 1. 字母 S (完全中心对称贝塞尔结构) ==========
        // 底部基准严格贴合 X=0 平面，顶部达到 X=1.5r
        
        // 下半弧：起始基点居中，Y轴极值向右侧扩张
        addCurve(0.00, 0.15,  
                 0, 0.25*r, 0.5*r, 0.75*r,   
                 0, -0.8*r, -0.8*r, 0);       
                 
        // 上半弧：与下半部完美镜像，Y轴极值向左侧扩张
        addCurve(0.15, 0.30,  
                 0.75*r, 1.0*r, 1.25*r, 1.5*r,  
                 0, 0.8*r, 0.8*r, 0);

        addHover(0.30, 0.35,  1.5*r, 0); // 稳定 S 的顶部极点
        
        // ========== 2. 过渡 S -> D ==========
        // 保持高度平移，预留出无碰撞的安全间距
        addLine(0.35, 0.40,   1.5*r, 0,   1.5*r, -1.0*r);
        addHover(0.40, 0.45,  1.5*r, -1.0*r);
        
        // ========== 3. 字母 D ==========
        // 左边界直线下落至底部基准面 X=0
        addLine(0.45, 0.49,   1.5*r, -1.0*r,   0, -1.0*r);
        
        // 左下角驻留缓冲，抑制直角下坠转侧倾的惯性超调
        addHover(0.49, 0.53,  0, -1.0*r);
        
        // D字外缘弧线：通过将X基准控制点设定为极值(0,0,1.5,1.5)，驱使曲线优先产生横向扩张
        addCurve(0.53, 0.65,  
                 0, 0, 1.5*r, 1.5*r,   
                 -1.0*r, -2.8*r, -2.8*r, -1.0*r);
        
        addHover(0.65, 0.70,  1.5*r, -1.0*r); // 稳定 D 的闭合终端
                 
        // ========== 4. 过渡 D -> U ==========
        // 跨越 D 弧线最大包围盒区，前往 U 的安全启动位
        addLine(0.70, 0.75,   1.5*r, -1.0*r,   1.5*r, -2.4*r);
        addHover(0.75, 0.80,  1.5*r, -2.4*r); 
        
        // ========== 5. 字母 U ==========
        // 左侧直线下落，停留于低位缓冲区 X=0.3r
        addLine(0.80, 0.85,   1.5*r, -2.4*r,   0.3*r, -2.4*r);
        
        // 底部平滑兜底：采用特定极点的贝塞尔方程，确保动态路径精准相切于最低点 X=0 且不再下陷
        addCurve(0.85, 0.95,  
                 0.3*r, -0.1*r, -0.1*r, 0.3*r,   
                 -2.4*r, -2.6*r, -3.2*r, -3.4*r);
                 
        // 右侧直线上升闭合回路
        addLine(0.95, 1.00,   0.3*r, -3.4*r,   1.5*r, -3.4*r);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        px_ = msg->pose.pose.position.x;
        py_ = msg->pose.pose.position.y;
        pz_ = msg->pose.pose.position.z;
        has_odom_ = true;
    }

    // 三次贝塞尔曲线位置解算
    void cubic_bezier(double t, const double P[4], double& pos) {
        double u = 1.0 - t;
        pos = u*u*u*P[0] + 3*u*u*t*P[1] + 3*u*t*t*P[2] + t*t*t*P[3];
    }

    // 三次贝塞尔曲线解析导数 (速度解算)
    void cubic_bezier_vel(double t, const double P[4], double dt_ds, double& vel) {
        double u = 1.0 - t;
        vel = (3*u*u*(P[1]-P[0]) + 6*u*t*(P[2]-P[1]) + 3*t*t*(P[3]-P[2])) * dt_ds;
    }

    // 发布位姿底层指令
    void publish_target(double x, double y, double z, double vx, double vy, double vz)
    {
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "map";
        cmd.position.x = x; cmd.position.y = y; cmd.position.z = z;
        
        // 速度前馈缩放阻尼 (0.0~1.0)，适度削弱高阶导数前馈，搭配基于点的强制驻留提供极佳的循迹平顺性
        double vel_scale = 0.8; 
        cmd.velocity.x = vx * vel_scale; 
        cmd.velocity.y = vy * vel_scale; 
        cmd.velocity.z = vz * vel_scale;
        
        cmd.yaw = 0.0; cmd.yaw_dot = 0.0;
        cmd_pub_.publish(cmd);
    }

    void timerCallback(const ros::TimerEvent& event)
    {
        if (!has_odom_) return;
        double now = ros::Time::now().toSec();

        if (state_ == WAIT_ODOM) {
            cx_ = px_; cy_ = py_; 
            start_time_ = ros::Time::now();
            state_ = HOVER;
            ROS_INFO("Odometry Acquired. Hovering at start point...");
        }
        else if (state_ == HOVER) {
            // 起飞预悬停3秒
            if (now - start_time_.toSec() >= 3.0) {
                state_ = TRAJECTORY;
                start_time_ = ros::Time::now();
                ROS_INFO("Executing SDU Trajectory...");
            } else {
                publish_target(cx_, cy_, z0_, 0.0, 0.0, 0.0);
            }
        }
        else if (state_ == TRAJECTORY) {
            double t = now - start_time_.toSec();
            if (t > total_time_ + ramp_time_) {
                state_ = DONE;
                ROS_INFO("Trajectory Completed. Holding End Point.");
                return;
            }

            // 平滑进入积分器，防止启动时刻冲击
            double time_velocity = (ramp_time_ > 0 && t < ramp_time_) ? (t / ramp_time_) : 1.0;
            double ramp_integral = (ramp_time_ > 0 && t < ramp_time_) ? (time_velocity * t / 2.0) : (ramp_time_ / 2.0 + (t - ramp_time_));
            
            double progress = ramp_integral / total_time_; 
            if (progress > 1.0) progress = 1.0;
            double d_progress = time_velocity / total_time_;

            double target_x = 0, target_y = 0;
            double vx = 0, vy = 0;

            // 轨迹段状态机映射解算
            for (const auto& seg : segs_) {
                if (progress >= seg.p0 && progress <= seg.p1) {
                    double local_t = (seg.p1 == seg.p0) ? 0.0 : ((progress - seg.p0) / (seg.p1 - seg.p0));
                    if (local_t < 0.0) local_t = 0.0;
                    if (local_t > 1.0) local_t = 1.0;

                    double dt_ds = (seg.p1 == seg.p0) ? 0.0 : (d_progress / (seg.p1 - seg.p0));

                    cubic_bezier(local_t, seg.Px, target_x);
                    cubic_bezier(local_t, seg.Py, target_y);
                    cubic_bezier_vel(local_t, seg.Px, dt_ds, vx);
                    cubic_bezier_vel(local_t, seg.Py, dt_ds, vy);
                    break;
                }
            }
            
            publish_target(target_x + cx_, target_y + cy_, z0_, vx, vy, 0.0);
        }
        else if (state_ == DONE) {
            // 在终点执行无限期悬停指令
            double end_x, end_y;
            cubic_bezier(1.0, segs_.back().Px, end_x);
            cubic_bezier(1.0, segs_.back().Py, end_y);
            publish_target(end_x + cx_, end_y + cy_, z0_, 0.0, 0.0, 0.0);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sdu_trajectory_node");
    ros::NodeHandle nh("~");
    SDUTrajectoryNode node(nh);
    ros::spin();
    return 0;
}