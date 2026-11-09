#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr**[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr*[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != nullptr)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i& start_idx,
                                                  Vector3i& end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    // A collision segment can start well inside the inflated obstacle, so a
    // fixed 0.5 m endpoint nudge is not enough for larger obstacles. Bound the
    // adjustment by the actual remaining search-pool margin on every axis.
    double pool_margin = std::numeric_limits<double>::infinity();
    for (int axis = 0; axis < 3; ++axis)
    {
        const double endpoint_offset = std::abs(start_pt(axis) - center_(axis)) * inv_step_size_;
        const double remaining_cells = POOL_SIZE_(axis) / 2.0 - endpoint_offset - 2.0;
        pool_margin = std::min(pool_margin, remaining_cells * step_size_);
    }
    const double max_adjustment = std::min(2.0, pool_margin);
    const auto move_to_free_endpoint = [&](Vector3d& point, const Vector3d& free_side,
                                           Vector3i& index) {
        // The search endpoint is embedded in the obstacle run. Move it away
        // from the opposite endpoint so it exits the occupied interval on
        // its own free side, rather than stepping farther into the obstacle.
        const Vector3d origin = point;
        Vector3d direction = point - free_side;
        const double length = direction.norm();
        if (!std::isfinite(length) || length < 1e-6 || !std::isfinite(max_adjustment) ||
            max_adjustment < step_size_)
            return false;
        direction /= length;
        for (double moved = step_size_; moved <= max_adjustment + 1e-9; moved += step_size_)
        {
            const Vector3d candidate = origin + direction * moved;
            Vector3i candidate_index;
            if (!Coord2Index(candidate, candidate_index))
                return false;
            if (!checkOccupancy(Index2Coord(candidate_index)))
            {
                point = candidate;
                index = candidate_index;
                return true;
            }
        }
        RCLCPP_WARN(rclcpp::get_logger("AstarSearch"),
                    "Occupied A* endpoint has no free voxel within %.2f m adjustment bound",
                    max_adjustment);
        return !checkOccupancy(Index2Coord(index));
    };

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        if (!move_to_free_endpoint(start_pt, end_pt, start_idx))
            return false;
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        if (!move_to_free_endpoint(end_pt, start_pt, end_idx))
            return false;
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    rclcpp::Time time_1 = rclcpp::Clock().now();
    ++rounds_;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        RCLCPP_ERROR(rclcpp::get_logger("AstarSearch"), "Unable to handle the initial or end point, force return!");
        return false;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = nullptr;
    GridNodePtr current = nullptr;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = nullptr;
    openSet_.push(startPtr); //put start in open set

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) ==
            endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 ||
                        neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) -
                        1)
                    {
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    // Disallow diagonal corner cutting: every proper
                    // axis-aligned subset of this voxel transition must be
                    // free before the diagonal edge is considered.
                    bool corner_blocked = false;
                    for (int mask = 1; mask < 8; ++mask)
                    {
                        if (mask == 7 || (dx == 0 && (mask & 1)) ||
                            (dy == 0 && (mask & 2)) || (dz == 0 && (mask & 4)))
                            continue;
                        Vector3i intermediate = current->index;
                        if (mask & 1) intermediate(0) += dx;
                        if (mask & 2) intermediate(1) += dy;
                        if (mask & 4) intermediate(2) += dz;
                        if (checkOccupancy(Index2Coord(intermediate)))
                        {
                            corner_blocked = true;
                            break;
                        }
                    }
                    if (corner_blocked)
                        continue;

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    if (checkOccupancy(Index2Coord(neighborPtr->index)))
                    {
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    {
                        //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }
        rclcpp::Time time_2 = rclcpp::Clock().now();
        if ((time_2 - time_1).seconds() > 0.2)
        {
            RCLCPP_WARN(rclcpp::get_logger("AstarSearch"),
                        "Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }

    rclcpp::Time time_2 = rclcpp::Clock().now();

    if ((time_2 - time_1).seconds() > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("AstarSearch"),
                "Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).seconds(), num_iter);

    return false;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
