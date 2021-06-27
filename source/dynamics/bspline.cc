#include "assert.h"

#include "sd/dynamics/bspline.h"

namespace sd::dynamics
{

  bool is_equal(double x, double y)
  {
    return (x - y) * (x - y) < kZeroEpsilon;
  }

  BSpline::BSpline()
      : num_knots_(bspline::kDegree + bspline::kNumMiddle + 2 + bspline::kConstLevelIni + bspline::kConstLevelFin +
                   1),
        num_cps_(bspline::kNumMiddle + 2 + bspline::kConstLevelIni + bspline::kConstLevelFin)
  {
    knots_.fill(0.);
    for (auto &i : cpoints_)
    {
      i.fill(0.);
    }
    assert(num_knots_ >= 2 * (bspline::kDegree + 1));
  }

  bool BSpline::SetParam(
      const std::array<double, 3 * bspline::kDim> &init, const std::array<double, 3 * bspline::kDim> &fin,
      const std::array<std::array<double, bspline::kDim>, 1> &middle_pt, double fin_time)
  {
    CalcKnot(fin_time);
    CalcConstrainedCPoints(init, fin, fin_time);
    CalcCPoints(middle_pt);

    return true;
  }

  bool BSpline::GetCurvePoint(double u, std::array<double, bspline::kDim> &ret)
  {
    int _span;

    if (u < knots_[0])
      u = knots_[0];
    else if (u > knots_[num_knots_ - 1])
    {
      u = knots_[num_knots_ - 1];
    }

    if (!FindSpan(_span, u))
      return false;

    std::array<double, bspline::kDegree + 1> _N;
    BasisFuns(_N, _span, u);

    std::array<double, bspline::kDim> _C;

    for (int j = 0; j < bspline::kDim; ++j)
    {
      _C[j] = 0.0;
      for (int i = 0; i <= bspline::kDegree; ++i)
      {
        _C[j] += _N[i] * cpoints_[_span - bspline::kDegree + i][j];
      }
    }

    ret = _C;

    return true;
  }

  bool BSpline::GetCurveDerPoint(double u, int d, std::array<double, bspline::kDim> &ret)
  {
    if (d > bspline::kDegree)
      return 0.0;

    if (u < knots_[0])
      u = knots_[0];
    else if (u > knots_[num_knots_ - 1])
    {
      u = knots_[num_knots_ - 1];
    }

    std::vector<std::array<double, bspline::kDim>> _CK(d + 1);

    if (CurveDerivsAlg1V(_CK, u, d))
    {
      for (int m = 0; m < bspline::kDim; ++m)
        ret[m] = _CK[d][m];
      return true;
    }
    return false;
  }

  void BSpline::CalcKnot(double Tf)
  {
    int _i = 0;
    int _j = 0;
    int _NumMidKnot(num_knots_ - 2 * bspline::kDegree - 2);
    double _TimeStep = Tf / (_NumMidKnot + 1);

    // augment knot sequence for the initial part, # of order ( degree + 1 )
    for (_j = 0; _j < bspline::kDegree + 1; ++_j)
      knots_[_i++] = 0.0;

    // uniform knot sequence for the middle part,
    // #: NumKnot - degree - degree = NumKnot - order - order + 2
    for (_j = 0; _j < _NumMidKnot; ++_j)
    {
      knots_[_i] = knots_[_i - 1] + _TimeStep;
      ++_i;
    }
    // augment knot sequence for the final part, # of order ( degree + 1 )
    for (_j = 0; _j < bspline::kDegree + 1; ++_j)
      knots_[_i++] = Tf;

    // for(int i = 0; i< num_knots_; ++i)
    // std::cout<<knots_[i]<<std::endl;
  }

  bool BSpline::CurveDerivsAlg1V(std::vector<std::array<double, bspline::kDim>> &CK, double u, int d)
  {
    assert(d <= bspline::kDegree);

    int _k = 0;
    int _j = 0;

    std::vector<std::array<double, bspline::kDegree + 1>> _nders(d + 1);

    int _span;
    if (!FindSpan(_span, u))
      return false;

    BasisFunsDers(_nders, _span, u, d);

    for (_k = 0; _k <= d; ++_k)
    {
      // Clean Up Column
      for (int m = 0; m < bspline::kDim; ++m)
        CK[_k][m] = 0.;

      for (_j = 0; _j <= bspline::kDegree; ++_j)
      {
        for (int m = 0; m < bspline::kDim; ++m)
        {
          CK[_k][m] += _nders[_k][_j] * cpoints_[_span - bspline::kDegree + _j][m];
        }
      }
    }

    return true;
  }

  bool BSpline::BasisFunsDers(
      std::vector<std::array<double, bspline::kConstLevelIni + 2>> &ders,
      double u, int n)
  {
    // int _span = FindSpan(u);
    int _span;
    if (!FindSpan(_span, u))
      return false;

    BasisFunsDers(ders, _span, u, n);
    return true;
  }

  bool BSpline::BasisFunsDers(
      std::vector<std::array<double, bspline::kConstLevelIni + 2>> &ders,
      int span, double u, int n)
  {
    int _j, _r, _k;
    int _s1, _s2;
    int _j1, _j2;
    int _rk;
    int _pk;

    double _saved = 0.0;
    double _left = 0.0;
    double _right = 0.0;
    double _temp = 0.0;
    double _d = 0.0;

    // to store the basis functions and knot differences
    std::array<std::array<double, bspline::kDegree + 1>, bspline::kDegree + 1> _ndu;

    // to store (in an alternating fashion) the two most recently computed
    // rows a(k,j) and a(k-1,j)
    std::array<std::array<double, bspline::kDegree + 1>, 2> _a;

    _ndu[0][0] = 1.0;
    for (_j = 1; _j <= bspline::kDegree; ++_j)
    {
      _saved = 0.0;
      for (_r = 0; _r < _j; ++_r)
      {
        _left = Left(span, _j - _r, u);
        _right = Right(span, _r + 1, u);

        // Lower triangle
        _ndu[_j][_r] = _right + _left;
        _temp = _ndu[_r][_j - 1] / _ndu[_j][_r];

        // Upper triangle
        _ndu[_r][_j] = _saved + _right * _temp;
        _saved = _left * _temp;
      }
      _ndu[_j][_j] = _saved;
    }

    // Load the basis functions
    for (_j = 0; _j <= bspline::kDegree; ++_j)
      ders[0][_j] = _ndu[_j][bspline::kDegree];

    // This section computes the derivatives (Eq. [2.9])
    for (_r = 0; _r <= bspline::kDegree; ++_r)
    {
      _s1 = 0;
      _s2 = 1;
      _a[0][0] = 1.0;

      // Loop to compute k-th derivative
      for (_k = 1; _k <= n; ++_k)
      {
        _d = 0.0;
        _rk = _r - _k;
        _pk = bspline::kDegree - _k;

        if (_r >= _k)
        {
          _a[_s2][0] = _a[_s1][0] / _ndu[_pk + 1][_rk];
          _d = _a[_s2][0] * _ndu[_rk][_pk];
        }

        if (_rk >= -1)
          _j1 = 1;
        else
          _j1 = -_rk;

        if (_r - 1 <= _pk)
          _j2 = _k - 1;
        else
          _j2 = bspline::kDegree - _r;

        for (_j = _j1; _j <= _j2; ++_j)
        {
          _a[_s2][_j] =
              (_a[_s1][_j] - _a[_s1][_j - 1]) / _ndu[_pk + 1][_rk + _j];
          _d += _a[_s2][_j] * _ndu[_rk + _j][_pk];
        }

        if (_r <= _pk)
        {
          _a[_s2][_k] = -_a[_s1][_k - 1] / _ndu[_pk + 1][_r];
          _d += _a[_s2][_k] * _ndu[_r][_pk];
        }
        ders[_k][_r] = _d;

        // Switch rows
        _j = _s1;
        _s1 = _s2;
        _s2 = _j;
      }
    }

    // Multiply through by the correct factors
    // (Eq. [2.9])
    _r = bspline::kDegree;
    for (_k = 1; _k <= n; ++_k)
    {
      for (_j = 0; _j <= bspline::kDegree; ++_j)
        ders[_k][_j] *= _r;

      _r *= (bspline::kDegree - _k);
    }
    return true;
  }

  void BSpline::BasisFuns(std::array<double, bspline::kDegree + 1> &N, double u)
  {
    // Original
    /*int _span = FindSpan(u);
      BasisFuns(N, _span, u);*/

    int _span;

    if (FindSpan(_span, u))
    {
      BasisFuns(N, _span, u);
    }
  }

  void BSpline::BasisFuns(std::array<double, bspline::kDegree + 1> &N, int span, double u)
  {
    int _j, _r;
    double _left = 0.0;
    double _right = 0.0;
    double _saved = 0.0;
    double _temp = 0.0;

    N[0] = 1.0;
    for (_j = 1; _j <= bspline::kDegree; ++_j)
    {
      _saved = 0.0;
      for (_r = 0; _r < _j; ++_r)
      {
        _left = Left(span, _j - _r, u);
        _right = Right(span, _r + 1, u);

        if ((_right + _left) != 0)
        {
          _temp = N[_r] / (_right + _left);
        }

        N[_r] = _saved + _right * _temp;
        _saved = _left * _temp;
      }
      N[_j] = _saved;
    }
  }

  bool BSpline::FindSpan(int &ret, double u)
  {
    if (u < knots_[0] || knots_[num_knots_ - 1] < u)
      return false;

    if (is_equal(u, knots_[num_knots_ - 1]))
    {
      for (int i(num_knots_ - 2); i > -1; --i)
      {
        if (knots_[i] < u && u <= knots_[i + 1])
        {
          ret = i;
          return true;
        }
      }
      return false;
    }
    // Binary search
    int _low = 0;
    int _high = num_knots_ - 1;
    int _mid = (_low + _high) >> 1;

    while (u < knots_[_mid] || u >= knots_[_mid + 1])
    {
      if (u < knots_[_mid])
        _high = _mid;
      else
        _low = _mid;
      _mid = (_low + _high) >> 1;
    }
    ret = _mid;
    return true;
  }

  void BSpline::CalcConstrainedCPoints(
      const std::array<double, 3 * bspline::kDim> &init,
      const std::array<double, 3 * bspline::kDim> &fin,
      double Tf)
  {
    // Position
    for (int m = 0; m < bspline::kDim; ++m)
    {
      cpoints_[0][m] = init[m];
      cpoints_[num_cps_ - 1][m] = fin[m];
    }

    // Initial Constraints
    std::vector<std::array<double, bspline::kDegree + 1>> d_mat(bspline::kConstLevelIni + 1);

    BasisFunsDers(d_mat, 0., bspline::kConstLevelIni);

    std::array<double, bspline::kDim> ini_const;
    // Vel, Acc, ...
    for (int j(1); j < bspline::kConstLevelIni + 1; ++j)
    {
      for (int k = 0; k < bspline::kDim; ++k)
      {
        ini_const[k] = init[j * bspline::kDim + k];

        for (int h(j); h > 0; --h)
        {
          ini_const[k] -= d_mat[j][h - 1] * cpoints_[h - 1][k];
        }
        cpoints_[j][k] = ini_const[k] / d_mat[j][j];
      }
    }

    // Final Constraints
    std::vector<std::array<double, bspline::kDegree + 1>> c_mat(bspline::kConstLevelFin + 1);

    BasisFunsDers(c_mat, Tf, bspline::kConstLevelFin);

    // Vel, Acc, ...
    int idx(1);
    for (int j(num_cps_ - 2); j > num_cps_ - 2 - bspline::kConstLevelFin; --j)
    {
      for (int k = 0; k < bspline::kDim; ++k)
      {
        ini_const[k] = fin[idx * bspline::kDim + k];

        for (int h(idx); h > 0; --h)
        {
          ini_const[k] -=
              c_mat[idx][bspline::kConstLevelFin + 2 - h] * cpoints_[num_cps_ - h][k];
        }
        cpoints_[j][k] = ini_const[k] / c_mat[idx][bspline::kConstLevelFin + 1 - idx];
      }
      ++idx;
    }
  }

  void BSpline::CalcCPoints(const std::array<std::array<double, bspline::kDim>, bspline::kNumMiddle> &middle_pt)
  {
    for (int i = 0; i < bspline::kNumMiddle; ++i)
    {
      for (int m = 0; m < bspline::kDim; ++m)
      {
        cpoints_[bspline::kConstLevelIni + 1 + i][m] = middle_pt[i][m];
      }
    }
  }

}
