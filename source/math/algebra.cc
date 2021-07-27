#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>

#include "math/algebra.h"

namespace sdrobot::math
{

  void PseudoInverse(Eigen::Ref<MatrixX> ret, Eigen::Ref<MatrixX const> const &inmat, double sigma_threshold)
  {

    if ((1 == inmat.rows()) && (1 == inmat.cols()))
    {
      ret.resize(1, 1);
      if (inmat.coeff(0, 0) > sigma_threshold)
      {
        ret.coeffRef(0, 0) = 1.0 / inmat.coeff(0, 0);
      }
      else
      {
        ret.coeffRef(0, 0) = 0.0;
      }
      return;
    }

    Eigen::JacobiSVD<MatrixX> svd(inmat,
                                  Eigen::ComputeThinU | Eigen::ComputeThinV);
    // not sure if we need to svd.sort()... probably not
    int const nrows(svd.singularValues().rows());
    MatrixX invS;
    invS = MatrixX::Zero(nrows, nrows);
    for (int ii(0); ii < nrows; ++ii)
    {
      if (svd.singularValues().coeff(ii) > sigma_threshold)
      {
        invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
      }
      else
      {
        // invS.coeffRef(ii, ii) = 1.0/ sigma_threshold;
        // printf("sigular value is too small: %f\n",
        // svd.singularValues().coeff(ii));
      }
    }
    ret = svd.matrixV() * invS * svd.matrixU().transpose();
  }
} // namespace sdrobot::dynamics
