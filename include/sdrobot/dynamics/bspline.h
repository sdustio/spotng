#pragma once

#include <array>
#include <vector>

#include "sdrobot/robot/model.h"

namespace sdrobot::dynamics
{

  /*!
 * Basic Bspline  <br/>
 * kDim : Dimension of control points <br/>
 * kDegree : Derivation is going to be 0 when it is over kDegree <br/>
 * kNumMiddle : Num middle points (the points except initial and final) <br/>
 * <br/>
 * kConstLevelIni/FIN : constraint level <br/>
 * 0: position <br/>
 * 1: + velocity <br/>
 * 2: + acceleration <br/>
 * <br/>
 * ******************************  WARNING
 * *********************************************** <br/> NumKnots(kDegree +
 * kNumMiddle + 2 + kConstLevelIni + kConstLevelFin) >= 2 * (kDegree + 1) <br/>
 * ******************************  WARNING
 * *********************************************** <br/>
 */

  namespace bspline
  {

    constexpr int kDim = robot::ModelAttrs::num_act_joint;
    constexpr int kDegree = 3;
    constexpr int kNumMiddle = 1;
    constexpr int kConstLevelIni = 2;
    constexpr int kConstLevelFin = 2;

  }

  class BSpline
  {
  public:
    BSpline();

    /*!
   * size of T: kDim * kConstLevelIni (or kConstLevelFin) <br/>
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
        const std::array<double, 3 * bspline::kDim> &init,
        const std::array<double, 3 * bspline::kDim> &fin,
        const std::array<std::array<double, bspline::kDim>, bspline::kNumMiddle> &middle_pt,
        double fin_time);

    /*!
   * get spline position at the given time <br/>
   * If the input time is before 0, it returns the initial <br/>
   * If the input time is after the final time, it returns the final <br/>
   * @param u : time <br/>
   * @return ret : position at the given time.  <br/>
   */
    bool GetCurvePoint(double u, std::array<double, bspline::kDim> &ret);

    /*!
   * get spline derivative information at the given time <br/>
   * If the input time is before 0, it returns the initial <br/>
   * If the input time is after the final time, it returns the final <br/>
   * @param u : time <br/>
   * @param d : drivative level (e.g. 1: velocity, 2: acceleration) <br/>
   * @return ret : derivative information at the given time.  <br/>
   */
    bool GetCurveDerPoint(double u, int d, std::array<double, bspline::kDim> &ret);

    // protected:
  private:
    void CalcKnot(double Tf);

    bool CurveDerivsAlg1V(std::vector<std::array<double, bspline::kDim>> &CK, double u, int d);

    bool BasisFunsDers(
        std::vector<std::array<double, bspline::kDegree + 1>> &ders,
        double u, int n);

    bool BasisFunsDers(
        std::vector<std::array<double, bspline::kDegree + 1>> &ders,
        int span, double u, int n);

    void BasisFuns(std::array<double, bspline::kDegree + 1> &N, double u);

    void BasisFuns(std::array<double, bspline::kDegree + 1> &N, int span, double u);

    inline double Left(int i, int j, double u) { return u - knots_[i + 1 - j]; }

    inline double Right(int i, int j, double u) { return knots_[i + j] - u; }

    bool FindSpan(int &ret, double u);

    void CalcConstrainedCPoints(
        const std::array<double, 3 * bspline::kDim> &init,
        const std::array<double, 3 * bspline::kDim> &fin,
        double Tf);

    void CalcCPoints(const std::array<std::array<double, bspline::kDim>, bspline::kNumMiddle> &middle_pt);

    double fin_time_;
    int num_knots_;
    int num_cps_;

    std::array<double, bspline::kDegree + bspline::kNumMiddle + 2 + bspline::kConstLevelIni + bspline::kConstLevelFin + 1> knots_;
    std::array<std::array<double, bspline::kDim>, bspline::kNumMiddle + 2 + bspline::kConstLevelIni + bspline::kConstLevelFin> cpoints_;
  };

}
