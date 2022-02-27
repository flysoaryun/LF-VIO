#ifndef PNP_SOLVER_H
#define PNP_SOLVER_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <vector>
class PnpSolver
{
public:
  PnpSolver();
  ~PnpSolver(){};
  void set_internal_parameters(double cx, double cy, double fx, double fy);
  void set_maximum_number_of_correspondences(int n);
  void reset_number_of_correspondences();
  void add_correspondence(Eigen::Vector3d point3d, Eigen::Vector3d uvz);

  double compute_pose(Eigen::Matrix3d &R, Eigen::Vector3d &T);
  void projectPoints(std::vector<Eigen::Vector3d> &src, std::vector<Eigen::Vector2d> &dst, Eigen::Matrix3d &R, Eigen::Vector3d &T);

  double cx, cy, fx, fy;

private:
  void choose_control_points();
  void compute_barycentric_coordinates();

  inline double dot(const double *v1, const double *v2);
  void compute_L_6x10(const Eigen::Matrix<double, 12, 12> &Ut, Eigen::Matrix<double, 6, 10> &L_6x10);
  void compute_rho(Eigen::Matrix<double, 6, 1> &Rho);
  void find_betas_0(const Eigen::Matrix<double, 6, 10> &L_6x10, const Eigen::Matrix<double, 6, 1> &Rho,
                    Eigen::Vector4d &Betas);
  void find_betas_1(const Eigen::Matrix<double, 6, 10> &L_6x10, const Eigen::Matrix<double, 6, 1> &Rho,
                    Eigen::Vector4d &Betas);
  void find_betas_2(const Eigen::Matrix<double, 6, 10> &L_6x10, const Eigen::Matrix<double, 6, 1> &Rho,
                    Eigen::Vector4d &Betas);
  void compute_ccs(const Eigen::Vector4d &Betas, const Eigen::Matrix<double, 12, 12> &Ut);
  void compute_pcs();
  void solve_for_sign();
  void estimate_R_and_t(Eigen::Matrix3d &R, Eigen::Vector3d &T);
  double reprojection_error(const Eigen::Matrix3d &R, const Eigen::Vector3d &T);
  double compute_R_and_t(const Eigen::Matrix<double, 12, 12> &Ut, const Eigen::Vector4d &Betas,
                         Eigen::Matrix3d &R, Eigen::Vector3d &T);
  void gauss_newton(const Eigen::Matrix<double, 6, 10> &L_6x10, const Eigen::Matrix<double, 6, 1> &Rho, Eigen::Vector4d &betas);
  void compute_A_and_b_gauss_newton(const Eigen::Matrix<double, 6, 10> &L_6x10, const Eigen::Matrix<double, 6, 1> &Rho,
                                    Eigen::Vector4d &betas, Eigen::Matrix<double, 6, 4> &A, Eigen::Matrix<double, 6, 1> &b);
  void qr_solve(Eigen::Matrix<double, 6, 4> &A, Eigen::Matrix<double, 6, 1> &B, Eigen::Vector4d &X);

  int maximum_number_of_correspondences;
  int number_of_correspondences;
  Eigen::MatrixXd pws, us, alphas, pcs, signs;
  Eigen::Matrix<double, 4, 3> cws, ccs;
  double cws_determinant;
};

#endif
