#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_env/grid_map.h>
#include <traj_utils/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>

namespace ego_planner
{
    // Fast Planner Manager
    // Key algorithms of mapping and planning are called

    class EGOPlannerManager
    {
        // SECTION stable
    public:
        EGOPlannerManager();
        ~EGOPlannerManager();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* main planning interface */
        bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                           Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit,
                           bool flag_randomPolyTraj);
        bool EmergencyStop(Eigen::Vector3d stop_pos);
        bool planGlobalTraj(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                            const Eigen::Vector3d& start_acc,
                            const Eigen::Vector3d& end_pos, const Eigen::Vector3d& end_vel,
                            const Eigen::Vector3d& end_acc);
        bool planGlobalTrajWaypoints(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& start_vel,
                                     const Eigen::Vector3d& start_acc,
                                     const std::vector<Eigen::Vector3d>& waypoints, const Eigen::Vector3d& end_vel,
                                     const Eigen::Vector3d& end_acc);
        // Generate a short minimum-jerk terminal correction after the primary
        // EGO spline ends outside the goal tolerance. The final spline must pass
        // the same inflated-map and physical-limit checks before publication.
        bool planTerminalCorrection(const Eigen::Vector3d& start_pos,
                                    const Eigen::Vector3d& start_vel,
                                    const Eigen::Vector3d& start_acc,
                                    const Eigen::Vector3d& end_pos);

        void initPlanModules(rclcpp::Node::SharedPtr& node, PlanningVisualization::Ptr vis = NULL);

        void deliverTrajToOptimizer() { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

        void setDroneIdtoOpt() { bspline_optimizer_->setDroneId(pp_.drone_id); }

        double getSwarmClearance() { return bspline_optimizer_->getSwarmClearance(); }

        bool checkCollision(int drone_id);


        PlanParameters pp_;
        LocalTrajData local_data_;
        GlobalTrajData global_data_;
        GridMap::Ptr grid_map_;
        SwarmTrajData swarm_trajs_buf_;

    private:
        /* main planning algorithms & modules */
        rclcpp::Clock::SharedPtr clock_;
        PlanningVisualization::Ptr visualization_;

        // ros::Publisher obj_pub_; //zx-todo

        BsplineOptimizer::Ptr bspline_optimizer_;

        int continous_failures_count_{0};

        void updateTrajInfo(const UniformBspline& position_traj, const rclcpp::Time time_now);

        bool collisionFree(UniformBspline& spline, double speed_bound);

        // !SECTION stable

        // SECTION developing

    public:
        typedef unique_ptr<EGOPlannerManager> Ptr;

        // !SECTION
    };
} // namespace ego_planner

#endif
