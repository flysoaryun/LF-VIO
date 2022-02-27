#include "pnp_solver.h"
#include <iostream>
PnpSolver::PnpSolver()
    : maximum_number_of_correspondences(0),
      number_of_correspondences(0)
{
}
void PnpSolver::set_internal_parameters(double cx, double cy, double fx, double fy)
{
  this->cx = cx;
  this->cy = cy;
  this->fx = fx;
  this->fy = fy;
}
void PnpSolver::set_maximum_number_of_correspondences(int n)
{
  maximum_number_of_correspondences = n;
  pws.resize(maximum_number_of_correspondences, 3);
  us.resize(maximum_number_of_correspondences, 3);
  alphas.resize(maximum_number_of_correspondences, 4);
  pcs.resize(maximum_number_of_correspondences, 3);
  signs.resize(maximum_number_of_correspondences, 1);
}
void PnpSolver::reset_number_of_correspondences()
{
  number_of_correspondences = 0;
}
void PnpSolver::add_correspondence(Eigen::Vector3d point3d, Eigen::Vector3d uvz)
{

  pws.row(number_of_correspondences) = point3d.transpose();
  if (uvz.z() > 0)
  {
    signs(number_of_correspondences) = 1;
  }
  else
  {
    signs(number_of_correspondences) = -1;
  }
  Eigen::Vector3d point2d(uvz.x(), uvz.y(), uvz.z());
  us.row(number_of_correspondences) = point2d.transpose();

  number_of_correspondences++;
}
void PnpSolver::choose_control_points()
{
  cws.setZero(4, 3);

  for (int i = 0; i < number_of_correspondences; i++)
    cws.row(0) += pws.row(i);
  cws.row(0) /= number_of_correspondences;

  Eigen::MatrixXd PW0(number_of_correspondences, 3);
  Eigen::Matrix3d PW0tPW0;
  Eigen::Vector3d DC;
  Eigen::Matrix3d UCt;

  for (int i = 0; i < number_of_correspondences; i++)
    PW0.row(i) = pws.row(i) - cws.row(0);

  PW0tPW0 = PW0.transpose() * PW0;

  // TODO:maybe wron
  // opencv  cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);
  // lmq
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(PW0tPW0, Eigen::ComputeFullU);
  DC = svd.singularValues();
  UCt = svd.matrixU().transpose();

  for (int i = 1; i < 4; i++)
  {
    double k = sqrt(DC(i - 1) / number_of_correspondences);
    cws.row(i) = cws.row(0) + k * UCt.row(i - 1);
  }
}
void PnpSolver::compute_barycentric_coordinates()
{
  Eigen::Matrix3d CC, CC_inv;

  for (int i = 0; i < 3; i++)
    for (int j = 1; j < 4; j++)
      CC(i, j - 1) = cws(j, i) - cws(0, i);

  CC_inv = CC.inverse();

  for (int i = 0; i < number_of_correspondences; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      alphas(i, 1 + j) = CC_inv(j, 0) * (pws(i, 0) - cws(0, 0)) +
                         CC_inv(j, 1) * (pws(i, 1) - cws(0, 1)) +
                         CC_inv(j, 2) * (pws(i, 2) - cws(0, 2));
    }
    alphas(i, 0) = 1.0f - alphas(i, 1) - alphas(i, 2) - alphas(i, 3);
  }
}
inline double PnpSolver::dot(const double *v1, const double *v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}
void PnpSolver::compute_L_6x10(const Eigen::Matrix<double, 12, 12> &Ut,
                               Eigen::Matrix<double, 6, 10> &L_6x10)
{
  int a, b;
  double dv[4][6][3];
  for (int i = 0; i < 4; i++)
  {
    a = 0, b = 1;
    for (int j = 0; j < 6; j++)
    {
      for (int k = 0; k < 3; k++)
        dv[i][j][k] = Ut(11 - i, 3 * a + k) - Ut(11 - i, 3 * b + k);

      if (++b > 3)
      {
        a++;
        b = a + 1;
      }
    }
  }

  for (int i = 0; i < 6; i++)
  {
    L_6x10(i, 0) = dot(dv[0][i], dv[0][i]);
    L_6x10(i, 1) = 2.0f * dot(dv[0][i], dv[1][i]);
    L_6x10(i, 2) = dot(dv[1][i], dv[1][i]);
    L_6x10(i, 3) = 2.0f * dot(dv[0][i], dv[2][i]);
    L_6x10(i, 4) = 2.0f * dot(dv[1][i], dv[2][i]);
    L_6x10(i, 5) = dot(dv[2][i], dv[2][i]);
    L_6x10(i, 6) = 2.0f * dot(dv[0][i], dv[3][i]);
    L_6x10(i, 7) = 2.0f * dot(dv[1][i], dv[3][i]);
    L_6x10(i, 8) = 2.0f * dot(dv[2][i], dv[3][i]);
    L_6x10(i, 9) = dot(dv[3][i], dv[3][i]);
  }
}
void PnpSolver::compute_rho(Eigen::Matrix<double, 6, 1> &Rho)
{
  Rho[0] = (cws.row(0) - cws.row(1)).squaredNorm();
  Rho[1] = (cws.row(0) - cws.row(2)).squaredNorm();
  Rho[2] = (cws.row(0) - cws.row(3)).squaredNorm();
  Rho[3] = (cws.row(1) - cws.row(2)).squaredNorm();
  Rho[4] = (cws.row(1) - cws.row(3)).squaredNorm();
  Rho[5] = (cws.row(2) - cws.row(3)).squaredNorm();
}
void PnpSolver::find_betas_0(const Eigen::Matrix<double, 6, 10> &L_6x10, const Eigen::Matrix<double, 6, 1> &Rho,
                             Eigen::Vector4d &Betas)
{
  Eigen::Matrix<double, 6, 4> L_6x4;
  Eigen::Vector4d B4;

  L_6x4.col(0) = L_6x10.col(0);
  L_6x4.col(1) = L_6x10.col(1);
  L_6x4.col(2) = L_6x10.col(3);
  L_6x4.col(3) = L_6x10.col(6);

  B4 = L_6x4.colPivHouseholderQr().solve(Rho);

  if (B4[0] < 0)
  {
    Betas[0] = sqrt(-B4[0]);
    Betas[1] = -B4[1] / Betas[0];
    Betas[2] = -B4[2] / Betas[0];
    Betas[3] = -B4[3] / Betas[0];
  }
  else
  {
    Betas[0] = sqrt(B4[0]);
    Betas[1] = B4[1] / Betas[0];
    Betas[2] = B4[2] / Betas[0];
    Betas[3] = B4[3] / Betas[0];
  }
}
// to modify
void PnpSolver::find_betas_1(const Eigen::Matrix<double, 6, 10> &L_6x10, const Eigen::Matrix<double, 6, 1> &Rho,
                             Eigen::Vector4d &Betas)
{
  Eigen::Matrix<double, 6, 3> L_6x3;
  Eigen::Vector3d B3;

  L_6x3.col(0) = L_6x10.col(0);
  L_6x3.col(1) = L_6x10.col(1);
  L_6x3.col(2) = L_6x10.col(2);

  B3 = L_6x3.colPivHouseholderQr().solve(Rho); // cv_svd 用fullPivHouseholderQr()呢？或者是使用eigen/svd里面的东西

  if (B3[0] < 0)
  {
    Betas[0] = sqrt(-B3[0]);
    Betas[1] = (B3[2] < 0) ? sqrt(-B3[2]) : 0.0;
  }
  else
  {
    Betas[0] = sqrt(B3[0]);
    Betas[1] = (B3[2] > 0) ? sqrt(B3[2]) : 0.0;
  }
  if (B3[1] < 0)
    Betas[0] = -Betas[0];

  Betas[2] = 0.0;
  Betas[3] = 0.0;
}
void PnpSolver::find_betas_2(const Eigen::Matrix<double, 6, 10> &L_6x10, const Eigen::Matrix<double, 6, 1> &Rho,
                             Eigen::Vector4d &Betas)
{
  Eigen::Matrix<double, 6, 5> L_6x5;
  Eigen::Matrix<double, 5, 1> B5;

  L_6x5.col(0) = L_6x10.col(0);
  L_6x5.col(1) = L_6x10.col(1);
  L_6x5.col(2) = L_6x10.col(2);
  L_6x5.col(3) = L_6x10.col(3);
  L_6x5.col(4) = L_6x10.col(4);

  B5 = L_6x5.colPivHouseholderQr().solve(Rho);

  if (B5[0] < 0)
  {
    Betas[0] = sqrt(-B5[0]);
    Betas[1] = (B5[2] < 0) ? sqrt(-B5[2]) : 0.0;
  }
  else
  {
    Betas[0] = sqrt(B5[0]);
    Betas[1] = (B5[2] > 0) ? sqrt(B5[2]) : 0.0;
  }
  if (B5[1] < 0.0)
    B5[0] = -B5[0];
  Betas[2] = B5[3] / Betas[0];
  Betas[3] = 0.0;
}
void PnpSolver::compute_ccs(const Eigen::Vector4d &Betas, const Eigen::Matrix<double, 12, 12> &Ut)
{
  ccs.setZero(4, 3);

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 3; k++)
        ccs(j, k) += Betas[i] * Ut(11 - i, 3 * j + k);
  }
}
void PnpSolver::compute_pcs()
{
  pcs = alphas * ccs;
}
void PnpSolver::solve_for_sign()
{

  if ((pcs(0, 2) < 0.0 && signs(0) > 0) | (pcs(0, 2) > 0.0 && signs(0) < 0))
  {
    ccs = -ccs;
    pcs = -pcs;
  }
}
void PnpSolver::estimate_R_and_t(Eigen::Matrix3d &R, Eigen::Vector3d &T)
{
  Eigen::Vector3d pc0, pw0;

  pc0.setZero();
  pw0.setZero();

  for (int i = 0; i < number_of_correspondences; i++)
  {
    pc0 += pcs.row(i);
    pw0 += pws.row(i);
  }
  pc0 /= number_of_correspondences;
  pw0 /= number_of_correspondences;

  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for (int i = 0; i < number_of_correspondences; i++)
  {
    W += (pcs.row(i).transpose() - pc0) * ((pws.row(i).transpose() - pw0).transpose());
  }

  // SVD on W
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  R = U * (V.transpose());
  T = pc0 - R * pw0;
}
double PnpSolver::reprojection_error(const Eigen::Matrix3d &R, const Eigen::Vector3d &T)
{
  double sum2 = 0.0;
  Eigen::Vector3d point3d;
  Eigen::Vector2d point2d;
  for (int i = 0; i < number_of_correspondences; i++)
  {
    point3d = R * pws.row(i).transpose() + T;
    sum2 += (us.row(i).transpose() - point3d).squaredNorm();
  }
  return sum2 / number_of_correspondences;
}
double PnpSolver::compute_R_and_t(const Eigen::Matrix<double, 12, 12> &Ut, const Eigen::Vector4d &Betas,
                                  Eigen::Matrix3d &R, Eigen::Vector3d &T)
{
  compute_ccs(Betas, Ut);
  compute_pcs();
  solve_for_sign();
  estimate_R_and_t(R, T);
  return reprojection_error(R, T);
}
double PnpSolver::compute_pose(Eigen::Matrix3d &R, Eigen::Vector3d &T)
{
  choose_control_points();
  compute_barycentric_coordinates();
  Eigen::MatrixXd M(2 * number_of_correspondences, 12);

  // fill M function
  for (int i = 0; i < number_of_correspondences; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      M(2 * i, 3 * j) = alphas(i, j) * fx;
      M(2 * i, 3 * j + 1) = 0.0;
      M(2 * i, 3 * j + 2) = alphas(i, j) * (cx - us(i, 0)) / us(i, 2);

      M(2 * i + 1, 3 * j) = 0.0;
      M(2 * i + 1, 3 * j + 1) = alphas(i, j) * fy;
      M(2 * i + 1, 3 * j + 2) = alphas(i, j) * (cy - us(i, 1)) / us(i, 2);
    }
  }
  Eigen::Matrix<double, 12, 12> MtM;
  Eigen::Matrix<double, 12, 1> D;
  Eigen::Matrix<double, 12, 12> Ut;

  MtM = M.transpose() * M;
  Eigen::JacobiSVD<Eigen::Matrix<double, 12, 12>> svd(MtM, Eigen::ComputeFullU);
  D = svd.singularValues().transpose();
  Ut = svd.matrixU().transpose();

  Eigen::Matrix<double, 6, 10> L_6x10;
  Eigen::Matrix<double, 6, 1> Rho;
  compute_L_6x10(Ut, L_6x10);
  compute_rho(Rho);

  Eigen::Vector4d Betas0, Betas1, Betas2;
  Eigen::Matrix3d R0, R1, R2;
  Eigen::Vector3d T0, T1, T2;
  double errors[3];

  find_betas_0(L_6x10, Rho, Betas0);
  gauss_newton(L_6x10, Rho, Betas0);
  errors[0] = compute_R_and_t(Ut, Betas0, R0, T0);
  find_betas_1(L_6x10, Rho, Betas1);
  gauss_newton(L_6x10, Rho, Betas1);
  errors[1] = compute_R_and_t(Ut, Betas1, R1, T1);
  find_betas_2(L_6x10, Rho, Betas2);
  gauss_newton(L_6x10, Rho, Betas2);
  errors[2] = compute_R_and_t(Ut, Betas2, R2, T2);

  int N = 0;
  R = R0;
  T = T0;
  if (errors[1] < errors[0])
  {
    N = 1;
    R = R1;
    T = T1;
  }
  if (errors[2] < errors[N])
  {
    N = 2;
    R = R2;
    T = T2;
  }
  return errors[N];
  return 1.0;
}

// attention
void PnpSolver::projectPoints(std::vector<Eigen::Vector3d> &src, std::vector<Eigen::Vector2d> &dst, Eigen::Matrix3d &R, Eigen::Vector3d &T)
{
  int size = src.size();
  for (int i = 0; i < size; ++i)
  {

    Eigen::Vector3d point3d = R * src[i] + T;

    dst[i][0] = cx + fx * point3d[0] / point3d[2];
    dst[i][1] = cy + fy * point3d[1] / point3d[2];
  }
}

void PnpSolver::gauss_newton(const Eigen::Matrix<double, 6, 10> &L_6x10, const Eigen::Matrix<double, 6, 1> &Rho,
                             Eigen::Vector4d &betas)
{
  const int iterations_number = 15;

  Eigen::Matrix<double, 6, 4> A;
  Eigen::Matrix<double, 6, 1> B;
  Eigen::Vector4d X;

  for (int k = 0; k < iterations_number; k++)
  {
    compute_A_and_b_gauss_newton(L_6x10, Rho, betas, A, B);
    qr_solve(A, B, X);

    for (int i = 0; i < 4; i++)
      betas[i] += X[i];
  }
}

void PnpSolver::compute_A_and_b_gauss_newton(const Eigen::Matrix<double, 6, 10> &L_6x10,
                                             const Eigen::Matrix<double, 6, 1> &Rho, Eigen::Vector4d &betas,
                                             Eigen::Matrix<double, 6, 4> &A, Eigen::Matrix<double, 6, 1> &b)
{
  for (int i = 0; i < 6; ++i)
  {

    Eigen::RowVectorXd rowL = L_6x10.row(i);
    Eigen::RowVector4d rowA = A.row(i);

    rowA[0] = 2 * rowL[0] * betas[0] + rowL[1] * betas[1] + rowL[3] * betas[2] + rowL[6] * betas[3];
    rowA[1] = rowL[1] * betas[0] + 2 * rowL[2] * betas[1] + rowL[4] * betas[2] + rowL[7] * betas[3];
    rowA[2] = rowL[3] * betas[0] + rowL[4] * betas[1] + 2 * rowL[5] * betas[2] + rowL[8] * betas[3];
    rowA[3] = rowL[6] * betas[0] + rowL[7] * betas[1] + rowL[8] * betas[2] + 2 * rowL[9] * betas[3];

    b[i] = Rho[i] -
           (rowL[0] * betas[0] * betas[0] +
            rowL[1] * betas[0] * betas[1] +
            rowL[2] * betas[1] * betas[1] +
            rowL[3] * betas[0] * betas[2] +
            rowL[4] * betas[1] * betas[2] +
            rowL[5] * betas[2] * betas[2] +
            rowL[6] * betas[0] * betas[3] +
            rowL[7] * betas[1] * betas[3] +
            rowL[8] * betas[2] * betas[3] +
            rowL[9] * betas[3] * betas[3]);
    A.row(i) = rowA;
  }
}

void PnpSolver::qr_solve(Eigen::Matrix<double, 6, 4> &A, Eigen::Matrix<double, 6, 1> &B, Eigen::Vector4d &X)
{
  X = A.colPivHouseholderQr().solve(B);
}
