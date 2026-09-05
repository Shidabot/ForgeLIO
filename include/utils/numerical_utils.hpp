#ifndef LIO_LIVOX_NUMERICAL_UTILS_HPP
#define LIO_LIVOX_NUMERICAL_UTILS_HPP

#include <algorithm>
#include <cmath>
#include <Eigen/Dense>

namespace lio_livox {
namespace numerics {

/**
 * Fit n.dot(x) + d = 0 to a point set and return ||n|| = 1.
 *
 * The normal is the eigenvector of the centered scatter matrix associated
 * with its smallest eigenvalue. Coincident and effectively collinear point
 * sets are rejected: they do not define a unique plane.
 */
inline bool FitNormalizedPlane(const Eigen::Matrix<double, 5, 3>& points,
                               Eigen::Vector4d* plane,
                               double min_absolute_spread = 1e-12,
                               double min_relative_spread = 1e-6) {
  if (plane == NULL || !points.allFinite()) {
    return false;
  }

  const Eigen::Vector3d centroid = points.colwise().mean().transpose();
  const Eigen::Matrix<double, 5, 3> centered =
      points.rowwise() - centroid.transpose();
  const Eigen::Matrix3d scatter =
      (centered.transpose() * centered) / 5.0;
  if (!centroid.allFinite() || !scatter.allFinite()) {
    return false;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite()) {
    return false;
  }

  // Eigenvalues are ascending. A plane needs observable spread in two
  // independent tangent directions; lambda_1 ~= 0 denotes a line or point.
  const Eigen::Vector3d eigenvalues = solver.eigenvalues();
  const double largest = eigenvalues(2);
  const double tangent_floor =
      std::max(min_absolute_spread, min_relative_spread * largest);
  if (!(largest > min_absolute_spread) || eigenvalues(1) <= tangent_floor) {
    return false;
  }

  Eigen::Vector3d normal = solver.eigenvectors().col(0);
  const double normal_norm = normal.norm();
  if (!normal.allFinite() || !std::isfinite(normal_norm) ||
      normal_norm <= 1e-12) {
    return false;
  }
  normal /= normal_norm;

  plane->head<3>() = normal;
  (*plane)(3) = -normal.dot(centroid);
  return plane->allFinite();
}

/**
 * Construct W such that W.transpose() * W is a regularized covariance inverse.
 *
 * This avoids covariance.inverse(), which is unstable for singular or poorly
 * conditioned IMU covariance matrices. The covariance is symmetrized, checked
 * for positive semidefiniteness, and small eigenvalues are floored before the
 * inverse square root is taken.
 */
template <int N>
inline bool ComputeRegularizedSqrtInformation(
    const Eigen::Matrix<double, N, N>& covariance,
    Eigen::Matrix<double, N, N>* sqrt_information,
    double min_absolute_eigenvalue = 1e-12,
    double min_relative_eigenvalue = 1e-9) {
  if (sqrt_information == NULL || !covariance.allFinite() ||
      !(min_absolute_eigenvalue > 0.0) ||
      !(min_relative_eigenvalue > 0.0)) {
    return false;
  }

  const Eigen::Matrix<double, N, N> symmetric_covariance =
      0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N> > solver(
      symmetric_covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite()) {
    return false;
  }

  const Eigen::Matrix<double, N, 1> eigenvalues = solver.eigenvalues();
  const double largest = eigenvalues.maxCoeff();
  const double spectral_radius = eigenvalues.cwiseAbs().maxCoeff();
  if (!(largest > min_absolute_eigenvalue) || !std::isfinite(largest)) {
    return false;
  }

  // A clearly negative eigenvalue means the propagated matrix is not a valid
  // covariance. Tiny negative values caused by round-off are regularized.
  const double negative_tolerance =
      std::max(min_absolute_eigenvalue, 1e-8 * spectral_radius);
  if (eigenvalues.minCoeff() < -negative_tolerance) {
    return false;
  }

  const double eigenvalue_floor = std::max(
      min_absolute_eigenvalue, min_relative_eigenvalue * largest);
  Eigen::Matrix<double, N, 1> inverse_sqrt_eigenvalues;
  for (int i = 0; i < N; ++i) {
    const double regularized = std::max(eigenvalues(i), eigenvalue_floor);
    inverse_sqrt_eigenvalues(i) = 1.0 / std::sqrt(regularized);
  }

  // W = Lambda^(-1/2) * V^T, hence W^T W = V Lambda^(-1) V^T.
  *sqrt_information = inverse_sqrt_eigenvalues.asDiagonal() *
                      solver.eigenvectors().transpose();
  return sqrt_information->allFinite();
}

}  // namespace numerics
}  // namespace lio_livox

#endif  // LIO_LIVOX_NUMERICAL_UTILS_HPP
