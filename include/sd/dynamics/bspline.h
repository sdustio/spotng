#pragma once

#include <array>
#include <vector>

#include "sd/robot/model.h"

namespace sd::dynamics
{

  /*!
 * Basic Bspline  <br/>
 * DIM : Dimension of control points <br/>
 * DEGREE : Derivation is going to be 0 when it is over DEGREE <br/>
 * NUM_MIDDLE : Num middle points (the points except initial and final) <br/>
 * <br/>
 * CONST_LEVEL_INI/FIN : constraint level <br/>
 * 0: position <br/>
 * 1: + velocity <br/>
 * 2: + acceleration <br/>
 * <br/>
 * ******************************  WARNING
 * *********************************************** <br/> NumKnots(DEGREE +
 * NUM_MIDDLE + 2 + CONST_LEVEL_INI + CONST_LEVEL_FIN) >= 2 * (DEGREE + 1) <br/>
 * ******************************  WARNING
 * *********************************************** <br/>
 */

  constexpr int DIM = robot::QuadrupedProperties::num_act_joint;
  constexpr int DEGREE = 3;
  constexpr int NUM_MIDDLE = 1;
  constexpr int CONST_LEVEL_INI = 2;
  constexpr int CONST_LEVEL_FIN = 2;

  class BSpline
  {
  public:
    BSpline();

    /*!
   * size of T: DIM * CONST_LEVEL_INI (or CONST_LEVEL_FIN) <br/>
   * ex) if dim:3, const level ini: 3(pos, vel, acc) <br/>
   * ini[0 ~ 2]: pos <br/>
   * ini[3 ~ 5]: vel <br/>
   * ini[6 ~ 8]: acc <br/>
   * @param init : initial point information (vector) <br/>
   * @param fin : finial point information (vector) <br/>
   * @param middle_pt   : middle point information (matrix) <br/>
   * @param fin_time : duration of the spline <br/>
   * @return boolean : success <br/>
   */
    bool SetParam(
        const std::array<double, 3 * DIM> &init,
        const std::array<double, 3 * DIM> &fin,
        const std::array<std::array<double, DIM>, NUM_MIDDLE> &middle_pt,
        double fin_time);

    /*!
   * get spline position at the given time <br/>
   * If the input time is before 0, it returns the initial <br/>
   * If the input time is after the final time, it returns the final <br/>
   * @param u : time <br/>
   * @return ret : position at the given time.  <br/>
   */
    bool GetCurvePoint(double u, std::array<double, DIM> &ret);

    /*!
   * get spline derivative information at the given time <br/>
   * If the input time is before 0, it returns the initial <br/>
   * If the input time is after the final time, it returns the final <br/>
   * @param u : time <br/>
   * @param d : drivative level (e.g. 1: velocity, 2: acceleration) <br/>
   * @return ret : derivative information at the given time.  <br/>
   */
    bool GetCurveDerPoint(double u, int d, std::array<double, DIM> &ret);

    // protected:
  private:
    void CalcKnot(double Tf);

    bool CurveDerivsAlg1V(std::vector<std::array<double, DIM>> &CK, double u, int d);

    bool BasisFunsDers(
      std::vector<std::array<double, DEGREE + 1>> &ders,
      double u, int n);

    bool BasisFunsDers(
      std::vector<std::array<double, DEGREE + 1>> &ders,
      int span, double u, int n);

    void BasisFuns(std::array<double, DEGREE + 1> &N, double u);

    void BasisFuns(std::array<double, DEGREE + 1> &N, int span, double u);

    inline double Left(int i, int j, double u) { return u - knots_[i + 1 - j]; }

    inline double Right(int i, int j, double u) { return knots_[i + j] - u; }

    bool FindSpan(int &ret, double u);

    void CalcConstrainedCPoints(
      const std::array<double, 3 * DIM> &init,
      const std::array<double, 3 * DIM> &fin,
      double Tf);

    void CalcCPoints(const std::array<std::array<double, DIM>, 1> &middle_pt);

    double fin_time_;
    int num_knots_;
    int num_cps_;

    std::array<double, DEGREE + NUM_MIDDLE + 2 + CONST_LEVEL_INI + CONST_LEVEL_FIN + 1> knots_;
    std::array<std::array<double, DIM>, NUM_MIDDLE + 2 + CONST_LEVEL_INI + CONST_LEVEL_FIN> cpoints_;
  };

}
