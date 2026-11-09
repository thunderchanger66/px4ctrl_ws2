#include <ego_planner/ego_replan_fsm.h>

#include <memory>
#include <stdexcept>
namespace ego_planner
{
    void EGOReplanFSM::init(rclcpp::Node::SharedPtr& node)
    {
        node_ = node;

        current_wp_ = 0;
        exec_state_ = INIT;
        have_target_ = false;
        have_odom_ = false;
        have_recv_pre_agent_ = false;
        flag_escape_emergency_ = true;

        node_->declare_parameter("fsm/flight_type", -1);
        node_->declare_parameter("fsm/thresh_replan_time", -1.0);
        node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
        node_->declare_parameter("fsm/goal_tolerance_m", 0.15);
        node_->declare_parameter("fsm/replan_retry_interval", 0.25);
        node_->declare_parameter("fsm/odom_timeout_sec", 0.5);
        node_->declare_parameter("fsm/odom_max_speed_mps", 5.0);
        node_->declare_parameter("fsm/odom_position_jump_slack_m", 0.35);
        node_->declare_parameter("fsm/planning_horizon", -1.0);
        node_->declare_parameter("fsm/planning_horizen_time", -1.0);
        node_->declare_parameter("fsm/emergency_time", 1.0);
        node_->declare_parameter("fsm/realworld_experiment", false);
        node_->declare_parameter("fsm/fail_safe", true);

        node_->get_parameter("fsm/flight_type", target_type_);
        node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
        node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
        node_->get_parameter("fsm/goal_tolerance_m", goal_tolerance_);
        node_->get_parameter("fsm/replan_retry_interval", replan_retry_interval_);
        if (!std::isfinite(replan_retry_interval_) || replan_retry_interval_ < 0.05)
        {
            RCLCPP_WARN(node_->get_logger(), "fsm/replan_retry_interval must be >= 0.05; using 0.25 s");
            replan_retry_interval_ = 0.25;
        }
        node_->get_parameter("fsm/odom_timeout_sec", odom_timeout_sec_);
        node_->get_parameter("fsm/odom_max_speed_mps", odom_max_speed_mps_);
        node_->get_parameter("fsm/odom_position_jump_slack_m", odom_position_jump_slack_m_);
        if (!std::isfinite(odom_timeout_sec_) || odom_timeout_sec_ <= 0.0) odom_timeout_sec_ = 0.5;
        if (!std::isfinite(odom_max_speed_mps_) || odom_max_speed_mps_ <= 0.0) odom_max_speed_mps_ = 5.0;
        if (!std::isfinite(odom_position_jump_slack_m_) || odom_position_jump_slack_m_ < 0.0)
            odom_position_jump_slack_m_ = 0.35;
        next_plan_attempt_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        node_->get_parameter("fsm/planning_horizon", planning_horizen_);
        node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
        node_->get_parameter("fsm/emergency_time", emergency_time_);
        node_->get_parameter("fsm/realworld_experiment", flag_realworld_experiment_);
        node_->get_parameter("fsm/fail_safe", enable_fail_safe_);

        have_trigger_ = !flag_realworld_experiment_;

        node_->declare_parameter("fsm/waypoint_num", -1);
        node_->get_parameter("fsm/waypoint_num", waypoint_num_);

        if (target_type_ < MANUAL_TARGET || target_type_ > REFENCE_PATH)
        {
            RCLCPP_FATAL(node_->get_logger(), "fsm/flight_type must be 1, 2, or 3 (got %d)", target_type_);
            throw std::invalid_argument("invalid EGO flight_type");
        }
        if (waypoint_num_ < 0 || waypoint_num_ > 50 ||
            (target_type_ == PRESET_TARGET && waypoint_num_ == 0))
        {
            RCLCPP_FATAL(node_->get_logger(),
                         "fsm/waypoint_num must be in [1, 50] for preset targets and [0, 50] otherwise (got %d)",
                         waypoint_num_);
            throw std::invalid_argument("invalid EGO waypoint_num");
        }

        for (int i = 0; i < waypoint_num_; i++)
        {
            node_->declare_parameter("fsm/waypoint" + to_string(i) + "_x", -1.0);
            node_->declare_parameter("fsm/waypoint" + to_string(i) + "_y", -1.0);
            node_->declare_parameter("fsm/waypoint" + to_string(i) + "_z", -1.0);

            node_->get_parameter("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0]);
            node_->get_parameter("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1]);
            node_->get_parameter("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2]);
            if (!std::isfinite(waypoints_[i][0]) || !std::isfinite(waypoints_[i][1]) ||
                !std::isfinite(waypoints_[i][2]))
            {
                RCLCPP_FATAL(node_->get_logger(), "Waypoint %d contains a non-finite coordinate", i);
                throw std::invalid_argument("non-finite EGO waypoint");
            }
        }

        /* initialize main modules */
        visualization_ = std::make_shared<PlanningVisualization>(node_);

        planner_manager_ = std::make_unique<EGOPlannerManager>();

        planner_manager_->initPlanModules(node_, visualization_);

        planner_manager_->deliverTrajToOptimizer(); // store trajectories
        planner_manager_->setDroneIdtoOpt();

        /* callback*/
        // This planner runs with use_sim_time=true.  A wall timer keeps firing
        // when Gazebo is rendering slowly or paused, while node_->now() (and
        // trajectory timestamps) advances with /clock.  That produces several
        // FSM decisions for the same simulation instant and can cause repeated
        // replans/oscillation near the goal.  Drive planning from ROS time.
        exec_timer_ = node_->create_timer(std::chrono::milliseconds(10),
                                          std::bind(&EGOReplanFSM::execFSMCallback, this));

        safety_timer_ = node_->create_timer(std::chrono::milliseconds(50),
                                            std::bind(&EGOReplanFSM::checkCollisionCallback, this));

        // odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        //     "odom_world",
        //     1,
        //     [this](const std::shared_ptr<const nav_msgs::msg::Odometry>& msg)
        //     {
        //         this->odometryCallback(msg);
        //     });
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "odom_world",
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            [this](const std::shared_ptr<const nav_msgs::msg::Odometry>& msg)
            {
                this->odometryCallback(msg);
            });
        // std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));

        if (planner_manager_->pp_.drone_id >= 1)
        {
            const string sub_topic_name = "/drone_" + std::to_string(planner_manager_->pp_.drone_id - 1) +
                "_planning/swarm_trajs";
            swarm_trajs_sub_ = node_->create_subscription<traj_utils::msg::MultiBsplines>(
                sub_topic_name,
                10,
                [this](const std::shared_ptr<const traj_utils::msg::MultiBsplines>& msg)
                {
                    this->swarmTrajsCallback(msg);
                });
        }

        // ros2 中topic名字中不能出现负号，单机id是-1需要处理
        // string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
        string pub_topic_name;
        if (planner_manager_->pp_.drone_id <= -1)
        {
            RCLCPP_INFO(node_->get_logger(), "single drone:%d", planner_manager_->pp_.drone_id);
            pub_topic_name = "/drone_single_planning/swarm_trajs";
        }
        else
        {
            pub_topic_name = "/drone_" + std::to_string(planner_manager_->pp_.drone_id) + "_planning/swarm_trajs";
        }

        swarm_trajs_pub_ = node_->create_publisher<traj_utils::msg::MultiBsplines>(pub_topic_name, 10);

        broadcast_bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>(
            "planning/broadcast_bspline_from_planner", 10);
        broadcast_bspline_sub_ = node_->create_subscription<traj_utils::msg::Bspline>(
            "planning/broadcast_bspline_to_planner",
            100,
            [this](const std::shared_ptr<const traj_utils::msg::Bspline>& msg)
            {
                this->BroadcastBsplineCallback(msg);
            });

        bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/bspline", 10);
        data_disp_pub_ = node_->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);
        emergency_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/planning/emergency_stop", 10);
        emergency_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
            "/planning/emergency_stop", 10,
            std::bind(&EGOReplanFSM::externalEmergencyCallback, this, std::placeholders::_1));

        if (target_type_ == MANUAL_TARGET)
        {
            waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose",
                1,
                [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& msg)
                {
                    this->waypointCallback(msg);
                });
        }
        else if (target_type_ == PRESET_TARGET)
        {
            trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/traj_start_trigger",
                1,
                [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& msg)
                {
                    this->triggerCallback(msg);
                });

            RCLCPP_INFO(node_->get_logger(), "Wait for 1 second.");
            int count = 0;
            while (rclcpp::ok() && count++ < 1000)
            {
                spin_some(node_);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            RCLCPP_WARN(node_->get_logger(), "Waiting for trigger from [n3ctrl] from RC");

            while (rclcpp::ok() && (!have_odom_ || !have_trigger_))
            {
                spin_some(node_);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            readGivenWps();
        }
        else
            cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
    }

    void EGOReplanFSM::readGivenWps()

    {
        if (waypoint_num_ <= 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Wrong waypoint_num_ = %d", waypoint_num_);
            return;
        }

        wps_.resize(waypoint_num_);
        for (int i = 0; i < waypoint_num_; i++)
        {
            wps_[i](0) = waypoints_[i][0];
            wps_[i](1) = waypoints_[i][1];
            wps_[i](2) = waypoints_[i][2];
        }

        // 用 visualization_->displayGoalPoint() 方法对waypoint进行可视化
        for (size_t i = 0; i < (size_t)waypoint_num_; i++)
        {
            visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // plan first global waypoint
        wp_id_ = 0;
        planNextWaypoint(wps_[wp_id_]);
    }

    void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
    {
        if (!next_wp.allFinite() || !planner_manager_->grid_map_->isInMap(next_wp) ||
            planner_manager_->grid_map_->getInflateOccupancy(next_wp) != 0)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Rejecting unreachable waypoint inside/outside the inflated planning map: %.2f %.2f %.2f",
                         next_wp.x(), next_wp.y(), next_wp.z());
            have_target_ = false;
            return;
        }
        bool success = false;
        success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp,
                                                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

        if (success)
        {
            end_pt_ = next_wp;

            constexpr double step_size_t = 0.1;
            int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
            vector<Eigen::Vector3d> gloabl_traj(i_end);
            for (int i = 0; i < i_end; i++)
            {
                gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
            }

            end_vel_.setZero();
            have_target_ = true;
            have_new_target_ = true;
            enable_fail_safe_ = true;

            /*** FSM状态转换 ***/
            // This function can be called from a subscription/timer callback.
            // Spinning this node here would try to add it to a second executor
            // and abort with "Node ... has already been added to an executor".
            // Let the existing FSM timer perform the requested transition.
            if (exec_state_ == EXEC_TRAJ)
                changeFSMExecState(REPLAN_TRAJ, "TRIG");
            else if (exec_state_ == INIT || exec_state_ == WAIT_TARGET ||
                     exec_state_ == SEQUENTIAL_START)
                changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
            else if (exec_state_ == EMERGENCY_STOP)
                changeFSMExecState(GEN_NEW_TRAJ, "NEW_GOAL");

            visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
        }
    }

    void EGOReplanFSM::triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& msg)
    {
        have_trigger_ = true;
        cout << "Triggered!" << endl;
        init_pt_ = odom_pos_;
    }

    void EGOReplanFSM::waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& msg)
    {
        if (msg->header.frame_id != node_->get_parameter("planning_frame").as_string())
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal frame %s differs from planning frame; transform the goal first", msg->header.frame_id.c_str());
            return;
        }
        if (msg->pose.position.z < -0.1)
            return;

        cout << "Triggered!" << endl;

        init_pt_ = odom_pos_;

        Eigen::Vector3d end_wp(
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        if (have_target_ && (end_pt_ - end_wp).norm() <= 1e-3)
        {
            RCLCPP_DEBUG(node_->get_logger(), "Ignoring duplicate active goal");
            return;
        }
        if (!end_wp.allFinite() || !planner_manager_->grid_map_->isInMap(end_wp))
        {
            RCLCPP_ERROR(node_->get_logger(), "Rejecting goal outside the configured occupancy-map bounds: %.2f %.2f %.2f",
                         end_wp.x(), end_wp.y(), end_wp.z());
            return;
        }
        if (planner_manager_->grid_map_->getInflateOccupancy(end_wp) != 0)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Rejecting goal inside an occupied or out-of-map inflated voxel: %.2f %.2f %.2f",
                         end_wp.x(), end_wp.y(), end_wp.z());
            return;
        }

        planNextWaypoint(end_wp);
    }

    void EGOReplanFSM::odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry>& msg)
    {
        if (msg->header.frame_id != node_->get_parameter("planning_frame").as_string())
        {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "Odometry frame %s differs from planning frame",
                                 msg->header.frame_id.c_str());
            return;
        }
        const Eigen::Vector3d position(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        if (!position.allFinite() || !planner_manager_->grid_map_->isInMap(position))
        {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                  "Rejecting odometry outside the configured occupancy-map bounds");
            return;
        }
        Eigen::Quaterniond orientation(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        if (!position.allFinite() || !std::isfinite(orientation.norm()) ||
            orientation.norm() < 1e-6) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "Ignoring odometry with invalid pose or orientation");
            return;
        }
        orientation.normalize();

        // FAST-LIO publishes twist in child_frame_id (the body frame), while
        // EGO's state and trajectory are expressed in header.frame_id (lidar).
        // Rotate the measured velocity into the planning/world frame before
        // using it as the initial condition for a trajectory.
        const Eigen::Vector3d child_velocity(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z);
        if (!child_velocity.allFinite()) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "Ignoring odometry with invalid velocity");
            return;
        }
        const rclcpp::Time stamp(msg->header.stamp, node_->get_clock()->get_clock_type());
        const double age = (node_->now() - stamp).seconds();
        const double speed = child_velocity.norm();
        // FAST-LIO stamps the scan at its simulated sensor end time.  During
        // startup that time can lead the ROS executor clock by one or two
        // sensor periods; rejecting it at 50 ms drops the first valid state
        // and can leave the planner in WAIT_TARGET with no usable odometry.
        // Sensor timestamps can lead /clock while Gazebo publishes a burst
        // after startup. Freshness is checked from callback receipt time
        // below; retain only a broad sanity bound for malformed clocks.
        const double odom_future_tolerance = 1.0;
        if (stamp.nanoseconds() <= 0 || !std::isfinite(age) || age < -odom_future_tolerance ||
            age > odom_timeout_sec_ || !std::isfinite(speed) || speed > odom_max_speed_mps_)
        {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                  "Rejecting stale/invalid odometry: age %.3f s, speed %.2f m/s", age, speed);
            return;
        }
        if (have_odom_)
        {
            const double dt = (stamp - last_odom_stamp_).seconds();
            if (!std::isfinite(dt) || dt <= 0.0)
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                     "Rejecting odometry with repeated or out-of-order timestamp");
                return;
            }
            const double step = (position - last_odom_pos_).norm();
            const double limit = odom_position_jump_slack_m_ + odom_max_speed_mps_ * dt;
            if (!std::isfinite(step) || step > limit)
            {
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                      "Rejecting odometry jump %.2f m over %.3f s (limit %.2f m)", step, dt, limit);
                return;
            }
        }
        const Eigen::Vector3d world_velocity = orientation * child_velocity;
        if (!world_velocity.allFinite()) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "Ignoring odometry with invalid world velocity");
            return;
        }
        odom_pos_ = position;
        odom_orient_ = orientation;
        odom_vel_ = world_velocity;
        last_odom_pos_ = position;
        last_odom_stamp_ = stamp;
        last_odom_time_ = node_->now();

        // odom_acc_ = estimateAcc( msg );

        have_odom_ = true;
    }

    void EGOReplanFSM::externalEmergencyCallback(
        const std::shared_ptr<const std_msgs::msg::Empty>&)
    {
        if (exec_state_ == EMERGENCY_STOP)
            return;

        RCLCPP_ERROR(node_->get_logger(), "External safety event received; latching EGO emergency stop");
        have_target_ = false;
        have_trigger_ = false;
        enable_fail_safe_ = false;
        flag_escape_emergency_ = true;
        changeFSMExecState(EMERGENCY_STOP, "EXTERNAL_SAFETY_EVENT");
    }

    bool EGOReplanFSM::odometryFresh() const
    {
        if (!have_odom_) return false;
        const double age = (node_->now() - last_odom_time_).seconds();
        // last_odom_time_ is recorded at callback receipt, so a sensor clock
        // lead cannot make a valid stream look stale or trigger a false stop.
        return std::isfinite(age) && age <= odom_timeout_sec_;
    }

    void EGOReplanFSM::BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline>& msg)
    {
        const size_t id = msg->drone_id;
        if (static_cast<int>(id) == planner_manager_->pp_.drone_id)
            return;

        // if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
        auto &clock = *node_->get_clock();
        auto msg_time = rclcpp::Time(msg->start_time, clock.get_clock_type());
        // RCLCPP_INFO(node_->get_logger(), "Clock type: %d", node_->now().get_clock_type());
        // RCLCPP_INFO(node_->get_logger(), "Start time clock type: %d", rclcpp::Time(msg->start_time).get_clock_type());
        // RCLCPP_INFO(node_->get_logger(), "msg_time: %d", msg_time.get_clock_type());
        if (abs((node_->now() - msg_time).seconds()) > 0.25)
        {
            // ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs", msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
            RCLCPP_ERROR(node_->get_logger(), "Time difference is too large! Local - Remote Agent %d = %fs",
                         msg->drone_id, (node_->now() - msg_time).seconds());
            return;
        }

        // 路径缓冲区初始化
        if (planner_manager_->swarm_trajs_buf_.size() <= id)
        {
            for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
            {
                OneTrajDataOfSwarm blank;
                blank.drone_id = -1;
                planner_manager_->swarm_trajs_buf_.push_back(blank);
            }
        }

        /* Test distance to the agent */
        Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
        Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
        Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
        Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
        if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
        {
            planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
            return; // if the current drone is too far to the received agent.
        }

        /* Store data */
        Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
        Eigen::VectorXd knots(msg->knots.size());
        for (size_t j = 0; j < msg->knots.size(); ++j)
        {
            knots(j) = msg->knots[j];
        }
        for (size_t j = 0; j < msg->pos_pts.size(); ++j)
        {
            pos_pts(0, j) = msg->pos_pts[j].x;
            pos_pts(1, j) = msg->pos_pts[j].y;
            pos_pts(2, j) = msg->pos_pts[j].z;
        }

        planner_manager_->swarm_trajs_buf_[id].drone_id = id;

        // 计算路径持续时间
        if (msg->order % 2)
        {
            double cutback = (double)msg->order / 2 + 1.5;
            planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
        }
        else
        {
            double cutback = (double)msg->order / 2 + 1.5;
            planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->
                knots[msg->knots.size() - ceil(cutback)]) / 2;
        }

        // 生成bspline并存储
        UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
        pos_traj.setKnot(knots);
        planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

        planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.
            evaluateDeBoorT(0);

        planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;

        /* Check Collision */
        if (planner_manager_->checkCollision(id))
        {
            changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
        }
    }

    void EGOReplanFSM::swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines>& msg)
    {
        multi_bspline_msgs_buf_.traj.clear();
        multi_bspline_msgs_buf_ = *msg;

        if (!have_odom_)
        {
            RCLCPP_ERROR(node_->get_logger(), "swarmTrajsCallback(): no odom!, return.");
            return;
        }

        if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
        {
            RCLCPP_ERROR(node_->get_logger(), "Wrong trajectory size!msg->traj.size()=%d, msg->drone_id_from+1=%d",
                         (int)msg->traj.size(), msg->drone_id_from + 1);
            return;
        }

        if (msg->traj[0].order != 3) // only support B-spline order equals 3.
        {
            RCLCPP_ERROR(node_->get_logger(), "Only support B-spline order equals 3.");
            return;
        }

        // Step 1. receive the trajectories
        planner_manager_->swarm_trajs_buf_.clear();
        planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

        // 处理每条路径
        for (size_t i = 0; i < msg->traj.size(); i++)
        {
            Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
            Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
            Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
            Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
            if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
            {
                planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
                continue;
            }

            // 存储路径控制点和节点
            Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
            Eigen::VectorXd knots(msg->traj[i].knots.size());
            for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
            {
                knots(j) = msg->traj[i].knots[j];
            }
            for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
            {
                pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
                pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
                pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
            }

            planner_manager_->swarm_trajs_buf_[i].drone_id = i;

            // 计算路径持续时间
            if (msg->traj[i].order % 2)
            {
                double cutback = static_cast<double>(msg->traj[i].order) / 2 + 1.5;
                planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() -
                    ceil(cutback)];
            }
            else
            {
                double cutback = static_cast<double>(msg->traj[i].order) / 2 + 1.5;
                planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() -
                    floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
            }

            // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
            UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
            pos_traj.setKnot(knots);
            planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

            planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.
                evaluateDeBoorT(0);

            planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
        }

        have_recv_pre_agent_ = true;
    }

    void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
    {
        if (new_state == exec_state_)
            continously_called_times_++;
        else
            continously_called_times_ = 1;

        static string state_str[8] = {
            "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"
        };
        int pre_s = int(exec_state_);
        exec_state_ = new_state;
        cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
    }

    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
    {
        return std::pair(continously_called_times_, exec_state_);
    }

    void EGOReplanFSM::printFSMExecState()
    {
        static string state_str[8] = {
            "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"
        };

        cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
    }

    void EGOReplanFSM::execFSMCallback()
    {
        exec_timer_->cancel(); // To avoid blockage

        static int fsm_num = 0;
        fsm_num++;
        if (fsm_num == 100)
        {
            printFSMExecState();
            if (!have_odom_)
                cout << "no odom." << endl;
            if (!have_target_)
                cout << "wait for goal or trigger." << endl;
            fsm_num = 0;
        }

        if (!odometryFresh())
        {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                  "Odometry timeout; holding the last accepted position and suspending planning");
            if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ)
            {
                flag_escape_emergency_ = true;
                changeFSMExecState(EMERGENCY_STOP, "ODOM_STALE");
            }
            else if (exec_state_ == GEN_NEW_TRAJ)
            {
                goto force_return;
            }
        }

        switch (exec_state_)
        {
        case INIT:
            {
                if (!have_odom_)
                {
                    goto force_return;
                }
                changeFSMExecState(WAIT_TARGET, "FSM");
                break;
            }

        case WAIT_TARGET:
            {
                if (!have_target_ || !have_trigger_)
                    goto force_return;
                changeFSMExecState(SEQUENTIAL_START, "FSM");
                break;
            }

        case SEQUENTIAL_START: // for swarm
            {
                if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 &&
                    have_recv_pre_agent_))
                {
                    if (have_odom_ && have_target_ && have_trigger_)
                    {
                        bool success = planFromGlobalTraj(10); // zx-todo
                        if (success)
                        {
                            changeFSMExecState(EXEC_TRAJ, "FSM");

                            publishSwarmTrajs(true);
                        }
                        else
                        {
                            RCLCPP_ERROR(node_->get_logger(), "Failed to generate the first trajectory!!!");
                            changeFSMExecState(SEQUENTIAL_START, "FSM");
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(node_->get_logger(), "No odom or no target! have_odom_=%d, have_target_=%d",
                                     have_odom_, have_target_);
                    }
                }

                break;
            }

        case GEN_NEW_TRAJ:
            {
                if (node_->now() < next_plan_attempt_)
                    break;
                bool success = planFromGlobalTraj(10); // zx-todo
                if (success)
                {
                    next_plan_attempt_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
                    changeFSMExecState(EXEC_TRAJ, "FSM");
                    flag_escape_emergency_ = true;
                    publishSwarmTrajs(false);
                }
                else
                {
                    next_plan_attempt_ = node_->now() + rclcpp::Duration::from_seconds(replan_retry_interval_);
                    changeFSMExecState(GEN_NEW_TRAJ, "FSM");
                }
                break;
            }

        case REPLAN_TRAJ:
            {
                if (node_->now() < next_plan_attempt_)
                    break;
                const bool success = replan_from_odom_ ? planFromGlobalTraj(1) : planFromCurrentTraj(1);
                if (success)
                {
                    next_plan_attempt_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
                    replan_from_odom_ = false;
                    changeFSMExecState(EXEC_TRAJ, "FSM");
                    publishSwarmTrajs(false);
                }
                else
                {
                    next_plan_attempt_ = node_->now() + rclcpp::Duration::from_seconds(replan_retry_interval_);
                    RCLCPP_ERROR(node_->get_logger(),
                                 "Unable to validate replanned trajectory; publishing a safe hover");
                    publishHoldTrajectory();
                }

                break;
            }

        case EXEC_TRAJ:
            {
                /* determine if need to replan */
                LocalTrajData* info = &planner_manager_->local_data_;
                rclcpp::Time time_now = node_->now();
                double t_cur = (time_now - info->start_time_).seconds();
                t_cur = std::min(info->duration_, t_cur);

                Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

                /* && (end_pt_ - pos).norm() < 0.5 */
                if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
                    (wp_id_ < waypoint_num_ - 1) &&
                    (end_pt_ - pos).norm() < no_replan_thresh_)
                {
                    wp_id_++;
                    planNextWaypoint(wps_[wp_id_]);
                }
                else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
                {
                    if (t_cur > info->duration_ - 1e-2)
                    {
                        const double goal_error = (end_pt_ - odom_pos_).norm();
                        if (goal_error <= goal_tolerance_)
                        {
                            have_target_ = false;
                            have_trigger_ = false;

                            if (target_type_ == TARGET_TYPE::PRESET_TARGET)
                            {
                                wp_id_ = 0;
                                planNextWaypoint(wps_[wp_id_]);
                            }

                            changeFSMExecState(WAIT_TARGET, "FSM");
                            goto force_return;
                        }

                        RCLCPP_WARN(node_->get_logger(),
                                    "Trajectory finished %.3f m from goal (tolerance %.3f m); replanning from odometry",
                                    goal_error, goal_tolerance_);
                        replan_from_odom_ = true;
                        planNextWaypoint(end_pt_);
                        if (planFromGlobalTraj(1))
                        {
                            replan_from_odom_ = false;
                            changeFSMExecState(EXEC_TRAJ, "FSM");
                            flag_escape_emergency_ = true;
                            publishSwarmTrajs(false);
                        }
                        else
                        {
                            RCLCPP_ERROR(node_->get_logger(),
                                         "Unable to validate a return trajectory; publishing a safe hover");
                            publishHoldTrajectory();
                        }
                        goto force_return;
                    }
                    else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
                    {
                        changeFSMExecState(REPLAN_TRAJ, "FSM");
                    }
                }
                else if (t_cur > replan_thresh_)
                {
                    changeFSMExecState(REPLAN_TRAJ, "FSM");
                }

                break;
            }

        case EMERGENCY_STOP:
            {
                if (flag_escape_emergency_) // Avoiding repeated calls
                {
                    callEmergencyStop(odom_pos_);
                }
                else
                {
                    if (enable_fail_safe_ && odometryFresh() && odom_vel_.norm() < 0.1)
                    {
                        const bool target_safe = have_target_ &&
                            planner_manager_->grid_map_->isInMap(end_pt_) &&
                            planner_manager_->grid_map_->getInflateOccupancy(end_pt_) == 0;
                        if (target_safe)
                        {
                            RCLCPP_WARN(node_->get_logger(),
                                        "Vehicle stopped after obstacle emergency; replanning from current odometry");
                            replan_from_odom_ = true;
                            changeFSMExecState(GEN_NEW_TRAJ, "SAFE_RECOVERY");
                        }
                        else
                        {
                            RCLCPP_ERROR(node_->get_logger(),
                                         "Emergency recovery blocked because the active goal is occupied or outside the map");
                            have_target_ = false;
                            have_trigger_ = false;
                            enable_fail_safe_ = false;
                        }
                    }
                }

                flag_escape_emergency_ = false;
                break;
            }
        }

        data_disp_.header.stamp = node_->now();
        data_disp_pub_->publish(data_disp_);

    force_return:;
        // exec_timer_.start();
        if (exec_timer_ && exec_timer_->is_canceled())
        {
            // 取消状态下无需重新创建，可以复用现有计时器
            exec_timer_->reset();
        }
    }

    bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
    {
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();

        bool flag_random_poly_init;
        if (timesOfConsecutiveStateCalls().first == 1)
            flag_random_poly_init = false;
        else
            flag_random_poly_init = true;

        for (int i = 0; i < trial_times; i++)
        {
            if (callReboundReplan(true, flag_random_poly_init))
            {
                return true;
            }
        }
        return false;
    }

    bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
    {
        LocalTrajData* info = &planner_manager_->local_data_;
        // ros::Time time_now = ros::Time::now();
        auto time_now = node_->now();
        // double t_cur = (time_now - info->start_time_).toSec();
        double t_cur = (time_now - info->start_time_).seconds();
        if (!std::isfinite(t_cur) || !std::isfinite(info->duration_) || info->duration_ <= 0.0)
            return false;
        t_cur = std::clamp(t_cur, 0.0, info->duration_);

        start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
        start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
        start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

        bool success = callReboundReplan(false, false);

        if (!success)
        {
            success = callReboundReplan(true, false);
            if (!success)
            {
                for (int i = 0; i < trial_times; i++)
                {
                    success = callReboundReplan(true, true);
                    if (success)
                        break;
                }
                if (!success)
                {
                    return false;
                }
            }
        }

        return true;
    }

    void EGOReplanFSM::checkCollisionCallback()
    {
        LocalTrajData* info = &planner_manager_->local_data_;
        auto map = planner_manager_->grid_map_;

        if (exec_state_ == WAIT_TARGET || exec_state_ == EMERGENCY_STOP ||
            info->start_time_.seconds() < 1e-5)
            return;

        if (have_target_ && planner_manager_->grid_map_->isInMap(end_pt_) &&
            planner_manager_->grid_map_->getInflateOccupancy(end_pt_) != 0)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Active goal became occupied in the inflated map; stopping instead of replanning toward it");
            have_target_ = false;
            have_trigger_ = false;
            enable_fail_safe_ = false;
            flag_escape_emergency_ = true;
            changeFSMExecState(EMERGENCY_STOP, "GOAL_OCCUPIED");
            return;
        }

        /* ---------- check lost of depth ---------- */
        if (map->getOdomDepthTimeout())
        {
            RCLCPP_ERROR(node_->get_logger(), "Depth Lost! EMERGENCY_STOP");

            enable_fail_safe_ = false;
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
            return;
        }

        /* ---------- check trajectory ---------- */
        constexpr double time_step = 0.01;
        // double t_cur = (ros::Time::now() - info->start_time_).toSec();
        double t_cur = (node_->now() - info->start_time_).seconds();
        if (!std::isfinite(t_cur) || !std::isfinite(info->duration_) || info->duration_ <= 0.0)
            return;
        t_cur = std::clamp(t_cur, 0.0, info->duration_);

        const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();

        for (double t = t_cur; t < info->duration_; t += time_step)
        {
            const Eigen::Vector3d trajectory_point = info->position_traj_.evaluateDeBoorT(t);
            bool occ = map->getInflateOccupancy(trajectory_point) != 0;

            for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
            {
                if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->
                    swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
                {
                    continue;
                }

                const double t_global = info->start_time_.seconds() + t;
                double t_X = t_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.seconds();
                const double other_duration = planner_manager_->swarm_trajs_buf_.at(id).duration_;
                if (!std::isfinite(t_X) || !std::isfinite(other_duration) || t_X < 0.0 || t_X > other_duration)
                    continue;
                Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.
                                                                    evaluateDeBoorT(t_X);
                double dist = (trajectory_point - swarm_pridicted).norm();

                if (dist < CLEARANCE)
                {
                    occ = true;
                    break;
                }
            }

            if (occ)
            {
                if (node_->now() < next_plan_attempt_)
                {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "Occupied active trajectory detected during replan cooldown; stopping instead of executing it");
                    // A failed obstacle replan is a latched safety event. Do not
                    // auto-recover into the same goal after the vehicle stops.
                    enable_fail_safe_ = false;
                    flag_escape_emergency_ = true;
                    changeFSMExecState(EMERGENCY_STOP, "SAFETY_COOLDOWN");
                    return;
                }
                if (planFromCurrentTraj()) // Replace the unsafe trajectory only after validation succeeds.
                {
                    next_plan_attempt_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
                    changeFSMExecState(EXEC_TRAJ, "SAFETY");
                    publishSwarmTrajs(false);
                }
                else
                {
                    next_plan_attempt_ = node_->now() + rclcpp::Duration::from_seconds(replan_retry_interval_);
                    RCLCPP_ERROR(node_->get_logger(),
                                 "No collision-free replacement trajectory; publishing a hold (collision lookahead %.3f s)",
                                 t - t_cur);
                    publishHoldTrajectory();
                    return;
                }
                break;
            }
        }
    }

    bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
    {
        getLocalTarget();

        bool plan_and_refine_success =
            planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_,
                                            (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
        have_new_target_ = false;

        cout << "refine_success=" << plan_and_refine_success << endl;

        if (plan_and_refine_success)
        {
            auto info = &planner_manager_->local_data_;

            traj_utils::msg::Bspline bspline;
            bspline.order = 3;
            bspline.start_time = info->start_time_;
            bspline.traj_id = info->traj_id_;

            Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
            bspline.pos_pts.reserve(pos_pts.cols());
            for (int i = 0; i < pos_pts.cols(); ++i)
            {
                geometry_msgs::msg::Point pt;
                pt.x = pos_pts(0, i);
                pt.y = pos_pts(1, i);
                pt.z = pos_pts(2, i);
                bspline.pos_pts.push_back(pt);
            }

            Eigen::VectorXd knots = info->position_traj_.getKnot();

            bspline.knots.reserve(knots.rows());
            for (int i = 0; i < knots.rows(); ++i)
            {
                bspline.knots.push_back(knots(i));
            }

            /* 1. publish traj to traj_server */
            bspline_pub_->publish(bspline);

            /* 2. publish traj to the next drone of swarm */

            /* 3. publish traj for visualization */
            visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
        }

        return plan_and_refine_success;
    }

    void EGOReplanFSM::publishHoldTrajectory()
    {
        if (!have_odom_ || !odom_pos_.allFinite()) {
            return;
        }
        planner_manager_->EmergencyStop(odom_pos_);
        auto info = &planner_manager_->local_data_;
        traj_utils::msg::Bspline bspline;
        bspline.order = 3;
        bspline.start_time = info->start_time_;
        bspline.traj_id = info->traj_id_;
        const Eigen::MatrixXd points = info->position_traj_.getControlPoint();
        bspline.pos_pts.reserve(points.cols());
        for (int i = 0; i < points.cols(); ++i) {
            geometry_msgs::msg::Point point;
            point.x = odom_pos_.x();
            point.y = odom_pos_.y();
            point.z = odom_pos_.z();
            bspline.pos_pts.push_back(point);
        }
        const Eigen::VectorXd knots = info->position_traj_.getKnot();
        bspline.knots.reserve(knots.rows());
        for (int i = 0; i < knots.rows(); ++i) {
            bspline.knots.push_back(knots(i));
        }
        bspline_pub_->publish(bspline);
        have_target_ = false;
        have_new_target_ = false;
        replan_from_odom_ = false;
        enable_fail_safe_ = false;
        flag_escape_emergency_ = true;
        changeFSMExecState(WAIT_TARGET, "SAFE_HOLD");
    }

    void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
    {
        auto info = &planner_manager_->local_data_;

        traj_utils::msg::Bspline bspline;
        bspline.order = 3;
        bspline.start_time = info->start_time_;
        bspline.drone_id = planner_manager_->pp_.drone_id;
        bspline.traj_id = info->traj_id_;

        Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
        bspline.pos_pts.reserve(pos_pts.cols());
        for (int i = 0; i < pos_pts.cols(); ++i)
        {
            geometry_msgs::msg::Point pt;
            pt.x = pos_pts(0, i);
            pt.y = pos_pts(1, i);
            pt.z = pos_pts(2, i);
            bspline.pos_pts.push_back(pt);
        }

        Eigen::VectorXd knots = info->position_traj_.getKnot();

        bspline.knots.reserve(knots.rows());
        for (int i = 0; i < knots.rows(); ++i)
        {
            bspline.knots.push_back(knots(i));
        }

        if (startup_pub)
        {
            multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
            if (static_cast<int>(multi_bspline_msgs_buf_.traj.size()) == planner_manager_->pp_.drone_id + 1)
            {
                multi_bspline_msgs_buf_.traj.back() = bspline;
            }
            else if (static_cast<int>(multi_bspline_msgs_buf_.traj.size()) == planner_manager_->pp_.drone_id)
            {
                multi_bspline_msgs_buf_.traj.push_back(bspline);
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d",
                             static_cast<int>(multi_bspline_msgs_buf_.traj.size()), planner_manager_->pp_.drone_id);
                // return plan_and_refine_success;
            }
            swarm_trajs_pub_->publish(multi_bspline_msgs_buf_);
        }

        broadcast_bspline_pub_->publish(bspline);
    }

    bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
    {
        if (emergency_pub_)
        {
            emergency_pub_->publish(std_msgs::msg::Empty{});
        }
        planner_manager_->EmergencyStop(stop_pos);

        auto info = &planner_manager_->local_data_;

        /* publish traj */
        traj_utils::msg::Bspline bspline;
        bspline.order = 3;
        bspline.start_time = info->start_time_;
        bspline.traj_id = info->traj_id_;

        Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
        bspline.pos_pts.reserve(pos_pts.cols());
        for (int i = 0; i < pos_pts.cols(); ++i)
        {
            geometry_msgs::msg::Point pt;
            pt.x = pos_pts(0, i);
            pt.y = pos_pts(1, i);
            pt.z = pos_pts(2, i);
            bspline.pos_pts.push_back(pt);
        }

        Eigen::VectorXd knots = info->position_traj_.getKnot();
        bspline.knots.reserve(knots.rows());
        for (int i = 0; i < knots.rows(); ++i)
        {
            bspline.knots.push_back(knots(i));
        }

        bspline_pub_->publish(bspline);

        return true;
    }

    void EGOReplanFSM::getLocalTarget()
    {
        const double max_vel = planner_manager_->pp_.max_vel_;
        if (!std::isfinite(planning_horizen_) || planning_horizen_ <= 0.0 ||
            !std::isfinite(max_vel) || max_vel <= 0.0 ||
            !std::isfinite(planner_manager_->global_data_.global_duration_) ||
            planner_manager_->global_data_.global_duration_ <= 0.0)
        {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                  "Cannot select local target: invalid horizon, speed, or global trajectory duration");
            local_target_pt_ = end_pt_;
            local_target_vel_.setZero();
            return;
        }
        double t;

        double t_step = planning_horizen_ / 20 / max_vel;
        double dist_min = 9999, dist_min_t = 0.0;
        for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_
             ; t += t_step)
        {
            Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
            double dist = (pos_t - start_pt_).norm();

            if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
            {
                // Important cornor case!
                for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
                {
                    Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
                    double dist_temp = (pos_t_temp - start_pt_).norm();
                    if (dist_temp < planning_horizen_)
                    {
                        pos_t = pos_t_temp;
                        dist = (pos_t - start_pt_).norm();
                        cout << "Escape cornor case \"getLocalTarget\"" << endl;
                        break;
                    }
                }
            }

            if (dist < dist_min)
            {
                dist_min = dist;
                dist_min_t = t;
            }

            if (dist >= planning_horizen_)
            {
                local_target_pt_ = pos_t;
                planner_manager_->global_data_.last_progress_time_ = dist_min_t;
                break;
            }
        }
        if (t > planner_manager_->global_data_.global_duration_) // Last global point
        {
            local_target_pt_ = end_pt_;
            planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
        }

        if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2
            * planner_manager_->pp_.max_acc_))
        {
            local_target_vel_ = Eigen::Vector3d::Zero();
        }
        else
        {
            local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
        }
    }
} // namespace ego_planner
