#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <iostream>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "visualization_msgs/msg/marker.hpp"

#include "bspline_opt/bspline_optimizer.h"
#include "plan_env/grid_map.h"
#include "traj_utils/msg/bspline.hpp"
#include "traj_utils/msg/multi_bsplines.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "traj_utils/msg/data_disp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "ego_planner/planner_manager.h"
#include "traj_utils/planning_visualization.h"

using std::vector;

namespace ego_planner
{
    class EGOReplanFSM
    {
        /* ---------- flag ---------- */
        enum FSM_EXEC_STATE
        {
            INIT,
            WAIT_TARGET,
            GEN_NEW_TRAJ,
            REPLAN_TRAJ,
            EXEC_TRAJ,
            EMERGENCY_STOP,
            SEQUENTIAL_START
        };

        enum TARGET_TYPE
        {
            MANUAL_TARGET = 1,
            PRESET_TARGET = 2,
            REFENCE_PATH = 3
        };

        /* planning utils */
        EGOPlannerManager::Ptr planner_manager_;
        PlanningVisualization::Ptr visualization_;
        traj_utils::msg::DataDisp data_disp_;
        traj_utils::msg::MultiBsplines multi_bspline_msgs_buf_;

        /* parameters */
        int target_type_; // 1 mannual select, 2 hard code
        double no_replan_thresh_, replan_thresh_, goal_tolerance_;
        double replan_retry_interval_{0.25};
        double odom_timeout_sec_{0.5};
        double odom_max_speed_mps_{5.0};
        double odom_position_jump_slack_m_{0.35};
        double waypoints_[50][3];
        int waypoint_num_, wp_id_;
        bool replan_from_odom_{false};
        double planning_horizen_, planning_horizen_time_;
        double emergency_time_;
        bool flag_realworld_experiment_;
        bool enable_fail_safe_;

        /* planning data */
        bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_;
        FSM_EXEC_STATE exec_state_;
        int continously_called_times_{0};

        Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
        Eigen::Quaterniond odom_orient_;
        Eigen::Vector3d last_odom_pos_;
        rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
        rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

        Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
        Eigen::Vector3d end_pt_, end_vel_; // goal state
        Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
        std::vector<Eigen::Vector3d> wps_;
        int current_wp_;

        bool flag_escape_emergency_;

        /* ROS utils */
        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
        rclcpp::Time next_plan_attempt_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_sub_;
        rclcpp::Subscription<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_sub_;
        rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;

        rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr bspline_pub_;
        rclcpp::Publisher<traj_utils::msg::DataDisp>::SharedPtr data_disp_pub_;
        rclcpp::Publisher<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_pub_;
        rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_pub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr emergency_pub_;

        /* helper functions */
        bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
        bool callEmergencyStop(Eigen::Vector3d stop_pos); // front-end and back-end method
        bool planFromGlobalTraj(int trial_times = 1);
        bool planFromCurrentTraj(int trial_times = 1);
        // Publish a collision-free hold at the current estimate when a new
        // trajectory cannot be validated. This keeps PX4Ctrl in a controlled
        // hover and waits for a fresh goal instead of executing stale points.
        void publishHoldTrajectory();

        /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
        void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
        std::pair<int, FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
        void printFSMExecState();

        void readGivenWps();
        void planNextWaypoint(Eigen::Vector3d next_wp);
        void getLocalTarget();

        /* ROS functions */
        void execFSMCallback();
        void checkCollisionCallback();
        void waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& msg);
        void triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& msg);
        void odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry>& msg);
        void externalEmergencyCallback(const std::shared_ptr<const std_msgs::msg::Empty>& msg);
        void swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines>& msg);
        void BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline>& msg);

        bool checkCollision();
        bool odometryFresh() const;
        void publishSwarmTrajs(bool startup_pub);

    public:
        EGOReplanFSM() = default;

        ~EGOReplanFSM() = default;

        void init(rclcpp::Node::SharedPtr& node);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace ego_planner

#endif
