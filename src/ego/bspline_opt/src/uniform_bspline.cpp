#include "bspline_opt/uniform_bspline.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_planner
{
    UniformBspline::UniformBspline(const Eigen::MatrixXd& points, const int& order,
                                   const double& interval)
    {
        setUniformBspline(points, order, interval);
    }

    UniformBspline::~UniformBspline()
    {
    }

    void UniformBspline::setUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                           const double& interval)
    {
        control_points_ = points;
        p_ = order;
        interval_ = interval;

        n_ = points.cols() - 1;
        m_ = n_ + p_ + 1;

        u_ = Eigen::VectorXd::Zero(m_ + 1);
        for (int i = 0; i <= m_; ++i)
        {
            if (i <= p_)
            {
                u_(i) = double(-p_ + i) * interval_;
            }
            else if (i > p_ && i <= m_ - p_)
            {
                u_(i) = u_(i - 1) + interval_;
            }
            else if (i > m_ - p_)
            {
                u_(i) = u_(i - 1) + interval_;
            }
        }
    }

    void UniformBspline::setKnot(const Eigen::VectorXd& knot) { this->u_ = knot; }

    Eigen::VectorXd UniformBspline::getKnot() { return this->u_; }

    bool UniformBspline::getTimeSpan(double& um, double& um_p)
    {
        if (p_ > u_.rows() || m_ - p_ > u_.rows())
            return false;

        um = u_(p_);
        um_p = u_(m_ - p_);

        return true;
    }

    Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

    Eigen::VectorXd UniformBspline::evaluateDeBoor(const double& u)
    {
        double ub = min(max(u_(p_), u), u_(m_ - p_));

        // determine which [ui,ui+1] lay in
        int k = p_;
        while (true)
        {
            if (u_(k + 1) >= ub)
                break;
            ++k;
        }

        /* deBoor's alg */
        vector<Eigen::VectorXd> d;
        for (int i = 0; i <= p_; ++i)
        {
            d.push_back(control_points_.col(k - p_ + i));
            // cout << d[i].transpose() << endl;
        }

        for (int r = 1; r <= p_; ++r)
        {
            for (int i = p_; i >= r; --i)
            {
                double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
                // cout << "alpha: " << alpha << endl;
                d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
            }
        }

        return d[p_];
    }

    // Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double& t) {
    //   return evaluateDeBoor(t + u_(p_));
    // }

    Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
    {
        // The derivative of a b-spline is also a b-spline, its order become p_-1
        // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
        Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
        for (int i = 0; i < ctp.cols(); ++i)
        {
            ctp.col(i) =
                p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
        }
        return ctp;
    }

    UniformBspline UniformBspline::getDerivative()
    {
        Eigen::MatrixXd ctp = getDerivativeControlPoints();
        UniformBspline derivative(ctp, p_ - 1, interval_);

        /* cut the first and last knot */
        Eigen::VectorXd knot(u_.rows() - 2);
        knot = u_.segment(1, u_.rows() - 2);
        derivative.setKnot(knot);

        return derivative;
    }

    double UniformBspline::getInterval() { return interval_; }

    void UniformBspline::setPhysicalLimits(const double& vel, const double& acc, const double& jerk,
                                            const double& tolerance)
    {
        limit_vel_ = vel;
        limit_acc_ = acc;
        limit_jerk_ = jerk;
        limit_ratio_ = 1.1;
        feasibility_tolerance_ = tolerance;
    }

    bool UniformBspline::checkFeasibility(double& ratio, bool show)
    {
        ratio = std::numeric_limits<double>::infinity();
        if (p_ != 3 || control_points_.cols() <= p_ || !control_points_.allFinite() ||
            !u_.allFinite() || !(limit_vel_ > 0 && limit_acc_ > 0 && limit_jerk_ > 0))
            return false;
        for (Eigen::Index i = 1; i < u_.size(); ++i)
            if (u_(i) <= u_(i - 1))
                return false;

        const auto velocity = getDerivative();
        auto acceleration = getDerivative().getDerivative();
        const auto max_norm = [](const Eigen::MatrixXd& points) {
            return points.allFinite() ? points.colwise().norm().maxCoeff() :
                std::numeric_limits<double>::infinity();
        };
        const double speed = max_norm(velocity.control_points_);
        const double accel = max_norm(acceleration.control_points_);
        const double jerk = max_norm(acceleration.getDerivative().control_points_);
        ratio = std::max({1.0, speed / limit_vel_, std::sqrt(accel / limit_acc_),
                          std::cbrt(jerk / limit_jerk_)});
        const double margin = 1.0 + feasibility_tolerance_;
        const bool feasible = std::isfinite(ratio) && speed <= limit_vel_ * margin + 1e-4 &&
            accel <= limit_acc_ * margin + 1e-4 && jerk <= limit_jerk_ * margin + 1e-4;
        if (show && !feasible)
            std::cout << "Spline derivative bounds (value / limit): speed " << speed << " / " << limit_vel_
                      << ", acceleration " << accel << " / " << limit_acc_
                      << ", jerk " << jerk << " / " << limit_jerk_ << std::endl;
        return feasible;
    }

    bool UniformBspline::parameterizeToBspline(const double& ts,
                                               const vector<Eigen::Vector3d>& point_set,
                                               const vector<Eigen::Vector3d>& derivatives,
                                               Eigen::MatrixXd& ctrl_pts)
    {
        if (!std::isfinite(ts) || ts <= 0.0 || point_set.size() < 5 || derivatives.size() != 4)
            return false;

        const Eigen::Index samples = point_set.size();
        const Eigen::Index controls = samples + 2;
        const Eigen::Index free_controls = controls - 6;
        Eigen::MatrixXd fixed = Eigen::MatrixXd::Zero(3, controls);
        const auto set_boundary = [&](Eigen::Index i, const Eigen::Vector3d& position,
                                      const Eigen::Vector3d& velocity, const Eigen::Vector3d& accel) {
            const Eigen::Vector3d offset = accel * (ts * ts / 3.0);
            fixed.col(i) = position + offset - ts * velocity;
            fixed.col(i + 1) = position - offset / 2.0;
            fixed.col(i + 2) = position + offset + ts * velocity;
        };
        set_boundary(0, point_set.front(), derivatives[0], derivatives[2]);
        set_boundary(controls - 3, point_set.back(), derivatives[1], derivatives[3]);

        // Each cubic knot sample has weights [1, 4, 1]/6. Solve for
        // interior points only, with exact endpoint position, velocity and acceleration.
        Eigen::MatrixXd basis = Eigen::MatrixXd::Zero(samples - 2, free_controls);
        Eigen::MatrixXd rhs(samples - 2, 3);
        for (Eigen::Index i = 1; i < samples - 1; ++i)
        {
            Eigen::Vector3d sample = point_set[i];
            for (Eigen::Index j = i; j <= i + 2; ++j)
            {
                const double weight = (j == i + 1 ? 4.0 : 1.0) / 6.0;
                if (j >= 3 && j < controls - 3)
                    basis(i - 1, j - 3) = weight;
                else
                    sample -= weight * fixed.col(j);
            }
            rhs.row(i - 1) = sample.transpose();
        }
        if (!fixed.allFinite() || !rhs.allFinite())
            return false;
        fixed.middleCols(3, free_controls) = basis.colPivHouseholderQr().solve(rhs).transpose();
        if (!fixed.allFinite())
            return false;
        ctrl_pts = std::move(fixed);
        return true;
    }

    double UniformBspline::getTimeSum()
    {
        double tm, tmp;
        if (getTimeSpan(tm, tmp))
            return tmp - tm;
        else
            return -1.0;
    }

    double UniformBspline::getLength(const double& res)
    {
        double length = 0.0;
        double dur = getTimeSum();
        Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
        for (double t = res; t <= dur + 1e-4; t += res)
        {
            p_n = evaluateDeBoorT(t);
            length += (p_n - p_l).norm();
            p_l = p_n;
        }
        return length;
    }

    double UniformBspline::getJerk()
    {
        UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

        Eigen::VectorXd times = jerk_traj.getKnot();
        Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
        int dimension = ctrl_pts.rows();

        double jerk = 0.0;
        for (int i = 0; i < ctrl_pts.cols(); ++i)
        {
            for (int j = 0; j < dimension; ++j)
            {
                jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
            }
        }

        return jerk;
    }

    void UniformBspline::getMeanAndMaxVel(double& mean_v, double& max_v)
    {
        UniformBspline vel = getDerivative();
        double tm, tmp;
        vel.getTimeSpan(tm, tmp);

        double max_vel = -1.0, mean_vel = 0.0;
        int num = 0;
        for (double t = tm; t <= tmp; t += 0.01)
        {
            Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
            double vn = vxd.norm();

            mean_vel += vn;
            ++num;
            if (vn > max_vel)
            {
                max_vel = vn;
            }
        }

        mean_vel = mean_vel / double(num);
        mean_v = mean_vel;
        max_v = max_vel;
    }

    void UniformBspline::getMeanAndMaxAcc(double& mean_a, double& max_a)
    {
        UniformBspline acc = getDerivative().getDerivative();
        double tm, tmp;
        acc.getTimeSpan(tm, tmp);

        double max_acc = -1.0, mean_acc = 0.0;
        int num = 0;
        for (double t = tm; t <= tmp; t += 0.01)
        {
            Eigen::VectorXd axd = acc.evaluateDeBoor(t);
            double an = axd.norm();

            mean_acc += an;
            ++num;
            if (an > max_acc)
            {
                max_acc = an;
            }
        }

        mean_acc = mean_acc / double(num);
        mean_a = mean_acc;
        max_a = max_acc;
    }
} // namespace ego_planner
