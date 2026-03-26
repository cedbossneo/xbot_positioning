#ifndef XBOT_POSITIONING_LIDAR_ODOMETRY_MEASUREMENT_MODEL_HPP_
#define XBOT_POSITIONING_LIDAR_ODOMETRY_MEASUREMENT_MODEL_HPP_

#include <cmath>
#include <kalman/LinearizedMeasurementModel.hpp>
#include "SystemModel.hpp"

namespace xbot
{
namespace positioning
{

/**
 * @brief Measurement vector for a LiDAR-derived pose estimate (x, y, theta)
 *
 * @param T Numeric scalar type
 */
template<typename T>
class LidarOdometryMeasurement : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(LidarOdometryMeasurement, T, 3)

    //! X position
    static constexpr size_t X = 0;

    //! Y position
    static constexpr size_t Y = 1;

    //! Orientation
    static constexpr size_t THETA = 2;

    T x_pos()  const { return (*this)[ X ]; }
    T y_pos()  const { return (*this)[ Y ]; }
    T theta()  const { return (*this)[ THETA ]; }

    T& x_pos() { return (*this)[ X ]; }
    T& y_pos() { return (*this)[ Y ]; }
    T& theta() { return (*this)[ THETA ]; }
};

/**
 * @brief Measurement model for a LiDAR-derived pose (x, y, theta)
 *
 * This model fuses a full 2D pose estimate from LiDAR odometry into the EKF.
 * The measurement directly observes the x, y, and theta state components.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       covariance square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class LidarOdometryMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, LidarOdometryMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef xbot::positioning::State<T> S;

    //! Measurement type shortcut definition
    typedef xbot::positioning::LidarOdometryMeasurement<T> M;

    LidarOdometryMeasurementModel()
    {
        // Setup noise jacobian. As this one is static, we can define it once
        // and do not need to update it dynamically
        this->V.setIdentity();
    }

    /**
     * @brief Definition of (linear) measurement function
     *
     * Maps the system state to the expected LiDAR pose measurement.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;

        // Calculate the LiDAR sensor position given the current system state.
        // Same rotation-aware offset as GPS antenna (PositionMeasurementModel).
        measurement.x_pos() = x.x_pos() + std::cos(x.theta()) * lidar_offset_x - std::sin(x.theta()) * lidar_offset_y;
        measurement.y_pos() = x.y_pos() + std::sin(x.theta()) * lidar_offset_x + std::cos(x.theta()) * lidar_offset_y;
        measurement.theta() = x.theta();

        return measurement;
    }

    double lidar_offset_x = 0;
    double lidar_offset_y = 0;

protected:

    /**
     * @brief Update Jacobian matrices for the measurement function
     *
     * The measurement function is linear in x, y, and theta, so the
     * Jacobian H is constant and set once here.
     *
     * @param x The current system state around which to linearize
     */
    void updateJacobians( const S& x )
    {
        this->H.setZero();
        this->H(M::X,     S::X)     = 1;
        this->H(M::Y,     S::Y)     = 1;
        this->H(M::THETA, S::THETA) = 1;

        // Partial derivatives of h() w.r.t. theta (due to lidar offset rotation)
        this->H(M::X, S::THETA) = -std::sin(x.theta()) * lidar_offset_x - std::cos(x.theta()) * lidar_offset_y;
        this->H(M::Y, S::THETA) =  std::cos(x.theta()) * lidar_offset_x - std::sin(x.theta()) * lidar_offset_y;
    }
};

} // namespace positioning
} // namespace xbot

#endif // XBOT_POSITIONING_LIDAR_ODOMETRY_MEASUREMENT_MODEL_HPP_
