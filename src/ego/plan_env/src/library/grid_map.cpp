#include <utility>
#include <limits>
#include <stdexcept>

#include "plan_env/grid_map.h"

using namespace std;
void GridMap::initMap(rclcpp::Node::SharedPtr node)
{
    node_ = std::move(node);

    /* get parameter */
    double x_size, y_size, z_size;
    node_->declare_parameter("grid_map/resolution", -1.0);
    node_->declare_parameter("grid_map/map_size_x", -1.0);
    node_->declare_parameter("grid_map/map_size_y", -1.0);
    node_->declare_parameter("grid_map/map_size_z", -1.0);
    node_->declare_parameter("grid_map/local_update_range_x", -1.0);
    node_->declare_parameter("grid_map/local_update_range_y", -1.0);
    node_->declare_parameter("grid_map/local_update_range_z", -1.0);
    node_->declare_parameter("grid_map/obstacles_inflation", -1.0);
    node_->declare_parameter("grid_map/min_flight_height", 0.25);
    node_->declare_parameter("grid_map/p_hit", 0.70);
    node_->declare_parameter("grid_map/p_miss", 0.35);
    node_->declare_parameter("grid_map/p_min", 0.12);
    node_->declare_parameter("grid_map/p_max", 0.97);
    node_->declare_parameter("grid_map/p_occ", 0.80);
    node_->declare_parameter("grid_map/visualization_truncate_height", -0.1);
    node_->declare_parameter("grid_map/virtual_ceil_height", -0.1);
    node_->declare_parameter("grid_map/virtual_ceil_yp", -0.1);
    node_->declare_parameter("grid_map/virtual_ceil_yn", -0.1);
    node_->declare_parameter("grid_map/show_occ_time", false);
    node_->declare_parameter("grid_map/frame_id", "world");
    node_->declare_parameter("grid_map/local_map_margin", 1);
    node_->declare_parameter("grid_map/ground_height", 1.0);
    node_->declare_parameter("grid_map/cloud_voxel_retention_sec", 4.0);

    node_->get_parameter("grid_map/resolution", mp_.resolution_);
    node_->get_parameter("grid_map/map_size_x", x_size);
    node_->get_parameter("grid_map/map_size_y", y_size);
    node_->get_parameter("grid_map/map_size_z", z_size);
    node_->get_parameter("grid_map/local_update_range_x", mp_.local_update_range_(0));
    node_->get_parameter("grid_map/local_update_range_y", mp_.local_update_range_(1));
    node_->get_parameter("grid_map/local_update_range_z", mp_.local_update_range_(2));
    node_->get_parameter("grid_map/obstacles_inflation", mp_.obstacles_inflation_);
    node_->get_parameter("grid_map/min_flight_height", mp_.min_flight_height_);
    node_->get_parameter("grid_map/p_hit", mp_.p_hit_);
    node_->get_parameter("grid_map/p_miss", mp_.p_miss_);
    node_->get_parameter("grid_map/p_min", mp_.p_min_);
    node_->get_parameter("grid_map/p_max", mp_.p_max_);
    node_->get_parameter("grid_map/p_occ", mp_.p_occ_);
    node_->get_parameter("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_);
    node_->get_parameter("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_);
    node_->get_parameter("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_);
    node_->get_parameter("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_);
    node_->get_parameter("grid_map/show_occ_time", mp_.show_occ_time_);
    node_->get_parameter("grid_map/frame_id", mp_.frame_id_);
    node_->get_parameter("grid_map/local_map_margin", mp_.local_map_margin_);
    node_->get_parameter("grid_map/ground_height", mp_.ground_height_);
    node_->get_parameter("grid_map/cloud_voxel_retention_sec", mp_.cloud_voxel_retention_sec_);
    if (!std::isfinite(mp_.resolution_) || mp_.resolution_ <= 0.0 ||
        !std::isfinite(x_size) || !std::isfinite(y_size) || !std::isfinite(z_size) ||
        x_size <= 0.0 || y_size <= 0.0 || z_size <= 0.0)
    {
        RCLCPP_FATAL(node_->get_logger(),
                     "grid_map resolution and map sizes must be finite and positive");
        throw std::invalid_argument("invalid EGO grid map geometry");
    }
    if (!std::isfinite(mp_.p_hit_) || mp_.p_hit_ <= 0.0 || mp_.p_hit_ >= 1.0 ||
        !std::isfinite(mp_.p_miss_) || mp_.p_miss_ <= 0.0 || mp_.p_miss_ >= 1.0 ||
        !std::isfinite(mp_.p_min_) || !std::isfinite(mp_.p_max_) || !std::isfinite(mp_.p_occ_) ||
        mp_.p_min_ <= 0.0 || mp_.p_min_ >= mp_.p_occ_ || mp_.p_occ_ >= mp_.p_max_ || mp_.p_max_ >= 1.0)
    {
        RCLCPP_FATAL(node_->get_logger(), "grid_map occupancy probabilities must satisfy 0 < p_min < p_occ < p_max < 1");
        throw std::invalid_argument("invalid EGO occupancy probabilities");
    }
    if (!std::isfinite(mp_.min_flight_height_) || mp_.min_flight_height_ <= mp_.ground_height_ ||
        mp_.min_flight_height_ >= mp_.ground_height_ + z_size)
    {
        RCLCPP_FATAL(node_->get_logger(),
                     "grid_map/min_flight_height must lie inside the vertical map bounds");
        throw std::invalid_argument("invalid EGO minimum flight height");
    }
    if (!std::isfinite(mp_.cloud_voxel_retention_sec_) ||
        mp_.cloud_voxel_retention_sec_ < 0.5 || mp_.cloud_voxel_retention_sec_ > 60.0)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "grid_map/cloud_voxel_retention_sec must be in [0.5, 60]; using 4.0 s");
        mp_.cloud_voxel_retention_sec_ = 4.0;
    }
    if (mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
    {
        mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
    }

    mp_.resolution_inv_ = 1 / mp_.resolution_;
    mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
    mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

    mp_.prob_hit_log_ = logit(mp_.p_hit_);
    mp_.prob_miss_log_ = logit(mp_.p_miss_);
    mp_.clamp_min_log_ = logit(mp_.p_min_);
    mp_.clamp_max_log_ = logit(mp_.p_max_);
    mp_.min_occupancy_log_ = logit(mp_.p_occ_);
    mp_.unknown_flag_ = 0.01;

    size_t voxel_count = 1;
    for (int i = 0; i < 3; ++i)
    {
        const double voxel_dim = std::ceil(mp_.map_size_(i) / mp_.resolution_);
        if (!std::isfinite(voxel_dim) || voxel_dim < 1.0 ||
            voxel_dim > static_cast<double>(std::numeric_limits<int>::max()))
            throw std::invalid_argument("EGO grid map voxel dimension is out of range");
        mp_.map_voxel_num_(i) = static_cast<int>(voxel_dim);
        if (voxel_count > static_cast<size_t>(std::numeric_limits<int>::max()) /
                              static_cast<size_t>(mp_.map_voxel_num_(i)))
            throw std::invalid_argument("EGO grid map voxel count exceeds supported buffer size");
        voxel_count *= static_cast<size_t>(mp_.map_voxel_num_(i));
    }

    mp_.map_min_boundary_ = mp_.map_origin_;
    mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

    // initialize data buffers

    const int buffer_size = static_cast<int>(voxel_count);

    md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
    md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
    md_.cloud_hit_time_sec_ = vector<double>(buffer_size, 0.0);


    // LiDAR map input uses independent odometry and point-cloud subscriptions.
    indep_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "grid_map/cloud", rclcpp::QoS(10), std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));

    indep_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "grid_map/odom", rclcpp::QoS(100).best_effort(), std::bind(&GridMap::odomCallback, this, std::placeholders::_1));

    // 定时器
    // Map updates are simulation-time work.  With the Gazebo GUI enabled the
    // real-time factor can drop; wall timers then run ahead of /clock and make
    // the planner process the same simulated pose repeatedly.
    vis_timer_ = node_->create_timer(
        std::chrono::duration<double>(0.11),
        std::bind(&GridMap::visCallback, this));

    // 发布者
    map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", 10);
    map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", 10);

    md_.has_odom_ = false;

    // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
    // rand_noise2_ = normal_distribution<double>(0, 0.2);
    // random_device rd;
    // eng_ = default_random_engine(rd());
}

void GridMap::resetBuffer()
{
    Eigen::Vector3d min_pos = mp_.map_min_boundary_;
    Eigen::Vector3d max_pos = mp_.map_max_boundary_;

    resetBuffer(min_pos, max_pos);

    md_.local_bound_min_ = Eigen::Vector3i::Zero();
    md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
    Eigen::Vector3i min_id, max_id;
    posToIndex(min_pos, min_id);
    posToIndex(max_pos, max_id);

    boundIndex(min_id);
    boundIndex(max_id);

    /* reset occ and dist buffer */
    for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
            for (int z = min_id(2); z <= max_id(2); ++z)
            {
                md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
            }
}

void GridMap::visCallback()
{
    publishMapInflate(true);
    publishMap();
}

void GridMap::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    if (odom->header.frame_id != mp_.frame_id_)
    {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Odometry frame %s differs from grid map frame %s",
                             odom->header.frame_id.c_str(), mp_.frame_id_.c_str());
        return;
    }
    md_.lidar_pos_(0) = odom->pose.pose.position.x;
    md_.lidar_pos_(1) = odom->pose.pose.position.y;
    md_.lidar_pos_(2) = odom->pose.pose.position.z;

    md_.has_odom_ = true;
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& img)
{
    if (img->header.frame_id != mp_.frame_id_)
    {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Point cloud frame %s differs from grid map frame %s",
                             img->header.frame_id.c_str(), mp_.frame_id_.c_str());
        return;
    }
    pcl::PointCloud<pcl::PointXYZ> latest_cloud;
    pcl::fromROSMsg(*img, latest_cloud);

    if (!md_.has_odom_)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Ignoring world cloud until a valid odometry sample is available");
        return;
    }

    if (latest_cloud.points.size() == 0)
        return;

    if (!md_.lidar_pos_.allFinite())
        return;

    const double now_sec = node_->now().seconds();
    Eigen::Vector3i min_idx, max_idx;
    posToIndex(md_.lidar_pos_ - mp_.local_update_range_, min_idx);
    posToIndex(md_.lidar_pos_ + mp_.local_update_range_, max_idx);
    boundIndex(min_idx);
    boundIndex(max_idx);

    // Keep recent hit voxels so a sparse scan does not erase walls that are
    // temporarily outside the current Livox return set.
    for (const auto &pt : latest_cloud.points)
    {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        Eigen::Vector3d point(pt.x, pt.y, pt.z);
        if (!isInMap(point) ||
            ((point - md_.lidar_pos_).cwiseAbs().array() > mp_.local_update_range_.array()).any())
            continue;
        Eigen::Vector3i idx;
        posToIndex(point, idx);
        md_.cloud_hit_time_sec_[toAddress(idx)] = now_sec;
    }

    const int inflate_steps = std::max(0, static_cast<int>(
        std::ceil(mp_.obstacles_inflation_ / mp_.resolution_)));
    Eigen::Vector3i clear_min = min_idx;
    Eigen::Vector3i clear_max = max_idx;
    if (md_.cloud_window_initialized_)
    {
        clear_min = clear_min.cwiseMin(md_.last_cloud_min_idx_);
        clear_max = clear_max.cwiseMax(md_.last_cloud_max_idx_);
    }
    clear_min -= Eigen::Vector3i::Constant(inflate_steps);
    clear_max += Eigen::Vector3i::Constant(inflate_steps);
    boundIndex(clear_min);
    boundIndex(clear_max);
    for (int x = clear_min.x(); x <= clear_max.x(); ++x)
        for (int y = clear_min.y(); y <= clear_max.y(); ++y)
            for (int z = clear_min.z(); z <= clear_max.z(); ++z)
                md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;

    const double cutoff = now_sec - mp_.cloud_voxel_retention_sec_;
    for (int x = min_idx.x(); x <= max_idx.x(); ++x)
        for (int y = min_idx.y(); y <= max_idx.y(); ++y)
            for (int z = min_idx.z(); z <= max_idx.z(); ++z)
            {
                const int address = toAddress(x, y, z);
                const double hit_time = md_.cloud_hit_time_sec_[address];
                if (hit_time <= 0.0) continue;
                if (hit_time < cutoff)
                {
                    md_.cloud_hit_time_sec_[address] = 0.0;
                    continue;
                }
                for (int dx = -inflate_steps; dx <= inflate_steps; ++dx)
                    for (int dy = -inflate_steps; dy <= inflate_steps; ++dy)
                        for (int dz = -1; dz <= 1; ++dz)
                        {
                            Eigen::Vector3i neighbor(x + dx, y + dy, z + dz);
                            if (isInMap(neighbor))
                                md_.occupancy_buffer_inflate_[toAddress(neighbor)] = 1;
                        }
            }

    md_.local_bound_min_ = clear_min;
    md_.local_bound_max_ = clear_max;
    md_.last_cloud_min_idx_ = min_idx;
    md_.last_cloud_max_idx_ = max_idx;
    md_.cloud_window_initialized_ = true;

    // add virtual ceiling to limit flight height
    // 添加虚拟天花板控制飞行高度
    if (mp_.virtual_ceil_height_ > -0.5)
    {
        int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
        for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
            for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
            {
                md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
            }
    }
}

void GridMap::publishMap()
{
    if (map_pub_->get_subscription_count() <= 0)
        return;

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3i min_cut = md_.local_bound_min_;
    Eigen::Vector3i max_cut = md_.local_bound_max_;

    int lmm = mp_.local_map_margin_ / 2;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);

    boundIndex(min_cut);
    boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
            for (int z = min_cut(2); z <= max_cut(2); ++z)
            {
                if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
                    continue;

                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(x, y, z), pos);
                if (pos(2) > mp_.visualization_truncate_height_)
                    continue;

                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                cloud.push_back(pt);
            }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::msg::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = node_->now();
    map_pub_->publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{
    if (map_inf_pub_->get_subscription_count() <= 0)
        return;

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3i min_cut = md_.local_bound_min_;
    Eigen::Vector3i max_cut = md_.local_bound_max_;

    if (all_info)
    {
        int lmm = mp_.local_map_margin_;
        min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
        max_cut += Eigen::Vector3i(lmm, lmm, lmm);
    }

    boundIndex(min_cut);
    boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
            for (int z = min_cut(2); z <= max_cut(2); ++z)
            {
                if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
                    continue;

                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(x, y, z), pos);
                if (pos(2) > mp_.visualization_truncate_height_)
                    continue;

                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                cloud.push_back(pt);
            }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::msg::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = node_->now();
    map_inf_pub_->publish(cloud_msg);

    // RCLCPP_INFO(rclcpp::get_logger("publishMapInflate"), "pub map");
}

bool GridMap::odomValid() { return md_.has_odom_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

void GridMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size)
{
    ori = mp_.map_origin_, size = mp_.map_size_;
}
