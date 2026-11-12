#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#define logit(x) (log((x) / (1 - (x))))

// constant parameters

struct MappingParameters
{
    /* map properties */
    Eigen::Vector3d map_origin_, map_size_;
    Eigen::Vector3d map_min_boundary_, map_max_boundary_; // map range in pos
    Eigen::Vector3i map_voxel_num_; // map range in index
    Eigen::Vector3d local_update_range_;
    double resolution_, resolution_inv_;
    double obstacles_inflation_;
    std::string frame_id_;

    // Retain LiDAR hit voxels briefly between sparse world-cloud messages.
    double cloud_voxel_retention_sec_;

    /* occupancy thresholds */
    double p_hit_, p_miss_, p_min_, p_max_, p_occ_; // occupancy probability
    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
           min_occupancy_log_; // logit of occupancy probability

    /* local map update and clear */
    int local_map_margin_;

    /* visualization and computation time display */
    double visualization_truncate_height_, virtual_ceil_height_, ground_height_, virtual_ceil_yp_, virtual_ceil_yn_;
    bool show_occ_time_;

    /* active mapping */
    double unknown_flag_;
};

// intermediate mapping data for fusion

struct MappingData
{
    // main map data, occupancy of each voxel and Euclidean distance

    std::vector<double> occupancy_buffer_;
    std::vector<char> occupancy_buffer_inflate_;
    std::vector<double> cloud_hit_time_sec_;
    Eigen::Vector3d lidar_pos_{Eigen::Vector3d::Zero()};

    // flags of map state

    bool has_odom_;

    // range of updating grid

    Eigen::Vector3i local_bound_min_, local_bound_max_;

    Eigen::Vector3i last_cloud_min_idx_{Eigen::Vector3i::Zero()};
    Eigen::Vector3i last_cloud_max_idx_{Eigen::Vector3i::Zero()};
    bool cloud_window_initialized_{false};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
    GridMap() = default;

    ~GridMap() = default;

    enum
    {
        INVALID_IDX = -10000
    };

    // occupancy map management
    void resetBuffer();
    void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

    inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
    inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
    inline int toAddress(const Eigen::Vector3i& id);
    inline int toAddress(int& x, int& y, int& z);
    inline bool isInMap(const Eigen::Vector3d& pos);
    inline bool isInMap(const Eigen::Vector3i& idx);

    inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
    inline void setOccupied(Eigen::Vector3d pos);
    inline int getOccupancy(Eigen::Vector3d pos);
    inline int getOccupancy(Eigen::Vector3i id);
    inline int getInflateOccupancy(Eigen::Vector3d pos);

    inline void boundIndex(Eigen::Vector3i& id);
    inline bool isUnknown(const Eigen::Vector3i& id);
    inline bool isUnknown(const Eigen::Vector3d& pos);
    inline bool isKnownFree(const Eigen::Vector3i& id);
    inline bool isKnownOccupied(const Eigen::Vector3i& id);

    void initMap(rclcpp::Node::SharedPtr node);

    void publishMap();
    void publishMapInflate(bool all_info = false);

    bool odomValid();
    void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
    inline double getResolution() const;
    Eigen::Vector3d getOrigin();
    int getVoxelNum();

    typedef std::shared_ptr<GridMap> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    MappingParameters mp_;
    MappingData md_;

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    void visCallback();

    inline void inflatePoint(const Eigen::Vector3i& pt, int step, std::vector<Eigen::Vector3i>& pts);
    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr indep_cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr indep_odom_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;

    rclcpp::TimerBase::SharedPtr vis_timer_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i& id)
{
    return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int GridMap::toAddress(int& x, int& y, int& z)
{
    return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void GridMap::boundIndex(Eigen::Vector3i& id)
{
    Eigen::Vector3i id1;
    id1(0) = std::max(std::min(id(0), mp_.map_voxel_num_(0) - 1), 0);
    id1(1) = std::max(std::min(id(1), mp_.map_voxel_num_(1) - 1), 0);
    id1(2) = std::max(std::min(id(2), mp_.map_voxel_num_(2) - 1), 0);
    id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector3i& id)
{
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool GridMap::isUnknown(const Eigen::Vector3d& pos)
{
    Eigen::Vector3i idc;
    posToIndex(pos, idc);
    return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i& id)
{
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    int adr = toAddress(id1);

    // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
    //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
    return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i& id)
{
    Eigen::Vector3i id1 = id;
    boundIndex(id1);
    int adr = toAddress(id1);

    return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline void GridMap::setOccupied(Eigen::Vector3d pos)
{
    if (!isInMap(pos))
        return;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
        id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;
}

inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ)
{
    if (occ != 1 && occ != 0)
    {
        std::cout << "occ value error!" << std::endl;
        return;
    }

    if (!isInMap(pos))
        return;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    md_.occupancy_buffer_[toAddress(id)] = occ;
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos)
{
    if (!isInMap(pos))
        return -1;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos)
{
    if (!isInMap(pos))
        return -1;

    Eigen::Vector3i id;
    posToIndex(pos, id);

    return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

inline int GridMap::getOccupancy(Eigen::Vector3i id)
{
    if (id(0) < 0 || id(0) >= mp_.map_voxel_num_(0) || id(1) < 0 || id(1) >= mp_.map_voxel_num_(1) ||
        id(2) < 0 || id(2) >= mp_.map_voxel_num_(2))
        return -1;

    return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector3d& pos)
{
    if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
        pos(2) < mp_.map_min_boundary_(2) + 1e-4)
    {
        return false;
    }
    if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
        pos(2) > mp_.map_max_boundary_(2) - 1e-4)
    {
        return false;
    }
    return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i& idx)
{
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
    {
        return false;
    }
    if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
        idx(2) > mp_.map_voxel_num_(2) - 1)
    {
        return false;
    }
    return true;
}

inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
{
    for (int i = 0; i < 3; ++i)
        id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos)
{
    for (int i = 0; i < 3; ++i)
        pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline void GridMap::inflatePoint(const Eigen::Vector3i& pt, int step, std::vector<Eigen::Vector3i>& pts)
{
    int num = 0;
    /* ---------- + shape inflate ---------- */
    // for (int x = -step; x <= step; ++x)
    // {
    //   if (x == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
    // }
    // for (int y = -step; y <= step; ++y)
    // {
    //   if (y == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
    // }
    // for (int z = -1; z <= 1; ++z)
    // {
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
    // }

    /* ---------- all inflate ---------- */
    for (int x = -step; x <= step; ++x)
        for (int y = -step; y <= step; ++y)
            for (int z = -step; z <= step; ++z)
            {
                pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
            }
}

inline double GridMap::getResolution() const { return mp_.resolution_; }

#endif
