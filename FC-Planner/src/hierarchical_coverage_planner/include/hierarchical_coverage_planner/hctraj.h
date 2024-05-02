/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of TrajGenerator class, which implements
 *                   collision-free coverage trajectory generation in FC-Planner.
 * License      :    GNU General Public License <http://www.gnu.org/licenses/>.
 * Project      :    FC-Planner is free software: you can redistribute it and/or 
 *                   modify it under the terms of the GNU Lesser General Public 
 *                   License as published by the Free Software Foundation, 
 *                   either version 3 of the License, or (at your option) any 
 *                   later version.
 *                   FC-Planner is distributed in the hope that it will be useful,
 *                   but WITHOUT ANY WARRANTY; without even the implied warranty 
 *                   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *                   See the GNU General Public License for more details.
 * Website      :    https://hkust-aerial-robotics.github.io/FC-Planner/
 *⭐⭐⭐*****************************************************************⭐⭐⭐*/

#ifndef _HCTRAJ_H_
#define _HCTRAJ_H_

#include <quadrotor_msgs/PolynomialTraj.h>
#include <gcopter/gcopter.hpp>
#include <gcopter/minco.hpp>
#include <gcopter/lbfgs_new.hpp>
#include <gcopter/firi.hpp>
#include <gcopter/voxel_map.hpp>
#include <gcopter/sfc_gen.hpp>
#include <gcopter/quickhull.hpp>
#include <gcopter/geo_utils.hpp>
#include <traj_utils/planning_visualization.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <chrono>

using namespace std;
using std::shared_ptr;

namespace predrecon
{

class PlanningVisualization;

class TrajGenerator
{
public:
  typedef Eigen::Matrix3Xd PolyhedronV;
  typedef Eigen::MatrixX4d PolyhedronH;
  typedef std::vector<PolyhedronV> PolyhedraV;
  typedef std::vector<PolyhedronH> PolyhedraH;

  TrajGenerator();
  ~TrajGenerator();
  /* Func */
  void init(ros::NodeHandle& nh);
  void wpsTraj(Eigen::MatrixXd &wps,Eigen::Matrix3d& iniState, Eigen::Matrix3d& finState, const Eigen::Vector3d& now_odom, std::vector<Eigen::Vector3d> &given_wps, vector<bool>& given_indi, bool add_odom);
  void TrajOpt();
  void PitchTrajOpt(vector<double>& given_pitch);
  void YawTrajOpt(vector<double>& given_yaw);
  void HCTraj(Eigen::Vector3d& now_odom, vector<Eigen::Vector3d>& waypts, vector<bool> waypt_indi, vector<double>& given_pitch, vector<double>& given_yaw, voxel_map::VoxelMap& vMap, double& Progress, double coeff);
  void outputTraj(double& compT);
  void outputCloud(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& visible_cloud, double& compT);
  /* Param */
  Eigen::Vector3d curOdom;
  Eigen::MatrixXd opt_wps, way_wps, view_wps, fused_wps;
  vector<bool> opt_indi;
  Eigen::SparseMatrix<double> select_waypt, select_viewpt;
  int piece_nums, waypt_count;
  Eigen::Matrix3d opt_inistate, opt_finstate;
  Eigen::VectorXd opt_times;
  double rho_T,rho_z,rho_p,rho_a,rho_v,rho_j, rho_e;
  double vel_bound, acc_bound, jerk_bound, yawd_bound, max_v_squared, max_a_squared, max_j_squared;
  Eigen::VectorXd opt_times_Pitch;
  Eigen::VectorXd opt_times_Yaw;
  int traj_id_{1};
  Trajectory<7> minco_traj;
  Eigen::MatrixX3d posCoeff;
  int posPieceNum; double posDuration;
  Trajectory<7> mincoPitch_traj;
  Eigen::MatrixX3d pitchCoeff;
  int pitchPieceNum; double pitchDuration;
  Trajectory<7> mincoYaw_traj;
  Eigen::MatrixX3d yawCoeff;
  int yawPieceNum; double yawDuration;
  double holyProg;
  bool bmk_, tripod_head_trigger_;
  string TrajFile, CloudFile;
  string PosFile, PitchFile, YawFile;
  bool visFlag;
  bool zFlag; double zPos;
  double drone;
  /* Data */
  vector<double> velocityBound;
  vector<Eigen::Vector3d> route;
  vector<double> pitchRoute;
  vector<double> yawRoute;
  vector<Eigen::MatrixX4d> hPolys; // H-Representation corridor
  vector<Eigen::Vector3d> pc;
  Eigen::VectorXi hPolyIdx;
  vector<bool> hPolyState;
  int polyN, pieceN;
  vector<Eigen::VectorXd> TrajPose;
  /* Vis */
  ros::Publisher corridorMesh_pub_;
  ros::Publisher corridorEdge_pub_;
  ros::Publisher routePub;
  ros::Publisher wayPointsPub;
  ros::Publisher appliedTrajectoryPub;
  ros::Publisher textPub;
  ros::Publisher PitchPub;
  ros::Publisher YawPub;
  /* Tools */
  void velocityPieceBound(vector<Eigen::Vector3d>& waypts, vector<double>& given_pitch, vector<double>& given_yaw, double angleCoeff);
  Eigen::Vector3d jetColorMap(double value);
  void genCorridor(Eigen::Vector3d& start, vector<Eigen::Vector3d>& waypts, double& Progress);
  void setCorridor();
  bool isPointInsidePolytope(Eigen::MatrixX4d& polytope, Eigen::Vector3d& point1, Eigen::Vector3d& point2);
  void YawInterpolation(double& duration, double& start, double& end, vector<double>& newYaw, vector<double>& newDur, double& CompT);
  void getAngularVelInter(double& t, double& gap, double& omega);
  void getPitch(double& t, double& pitch);
  void getPitchd(double& t, double& pd_);
  void getYaw(double& t, double& yaw);
  void getYawd(double& t, double& yd_);
  /* Utils */
  voxel_map::VoxelMap corridorMap;
  minco::MINCO_S4NU minco_anal;
  shared_ptr<PlanningVisualization> vis_utils_;
  gcopter::GCOPTER_PolytopeSFC gcopter;
  
  static double innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad);
  void TrajVisCallback(const ros::TimerEvent& e);
  
  void visualizePolytope(vector<Eigen::MatrixX4d>& hPolys);
  void visualize(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT);
  void visualizePitch(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT);
  void visualizeYaw(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT);
  void polynomialTrajConverter(const Trajectory<7> &traj,
                                               quadrotor_msgs::PolynomialTraj &msg,
                                                const ros::Time &iniStamp);
  void calConstrainCostGrad(double& cost, Eigen::MatrixXd& gdCxy, Eigen::VectorXd &gdTxy)
  {
    cost = 0.0;
    gdCxy.resize(8*piece_nums, 3);
    gdCxy.setZero();
    gdTxy.resize(piece_nums);
    gdTxy.setZero();

    Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;

    Eigen::Vector3d pos, vel, acc, jer, sna, outerNormal;
    Eigen::Vector3d grad_z = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_j = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 8, 1> beta0_xy, beta1_xy, beta2_xy, beta3_xy, beta4_xy;
    double s1, s2, s3, s4, s5, s6, s7;
    double step, alpha, omg, pena, smoothFactor;
    double violaPosPena, violaPosPenaD;
    double violaVelPena, violaVelPenaD;
    double violaAccPena, violaAccPenaD;
    double violaJerPena, violaJerPenaD;
    smoothFactor = 1e-2;

    Eigen::Vector3d gradPos, gradVel, gradAcc, gradJer;

    int int_K = 256;
    const double integralFrac = 1.0 / int_K;
    for (int i=0; i<piece_nums; i++)
    {
      const Eigen::Matrix<double, 8, 3> &c_xy = minco_anal.getCoeffs().block<8, 3>(i * 8, 0);
      step = minco_anal.T1(i) * integralFrac;
      s1 = 0.0;

      max_v_squared = velocityBound[i]*velocityBound[i];

      for (int j=0; j<=int_K; j++)
      {
        // analyse xy
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        s6 = s4 * s2;
        s7 = s4 * s3;
        beta0_xy << 1.0, s1, s2, s3, s4, s5, s6, s7;
        beta1_xy << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
        beta2_xy << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
        beta3_xy << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
        beta4_xy << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1, 360.0 * s2, 840.0 * s3;
        pos = c_xy.transpose() * beta0_xy;
        vel = c_xy.transpose() * beta1_xy;
        acc = c_xy.transpose() * beta2_xy;
        jer = c_xy.transpose() * beta3_xy;
        sna = c_xy.transpose() * beta4_xy;

        pena = 0.0;
        gradPos.setZero();
        gradVel.setZero();
        gradAcc.setZero();
        gradJer.setZero();

        omg = (j == 0 || j == int_K) ? 0.5 : 1.0;

        double v_snorm = vel.squaredNorm();
        double a_snorm = acc.squaredNorm();
        double j_snorm = jer.squaredNorm();

        // pos : corridor-based constraint
        if (hPolyState[i] == true)
        {
          int L = hPolyIdx(i);
          int K = hPolys[L].rows();
          for (int k = 0; k < K; k++)
          {
            outerNormal = hPolys[L].block<1, 3>(k, 0);
            double violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
            if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
            {
              gradPos += rho_p * violaPosPenaD * outerNormal;
              pena += rho_p * violaPosPena;
            }
          }
        }

        // vel : gimbal angle constraint
        double vViola = v_snorm - max_v_squared;
        if (smoothedL1(vViola, smoothFactor, violaVelPena, violaVelPenaD))
        {
          gradVel += rho_v * violaVelPenaD * 2.0 * vel;
          pena += rho_v * violaVelPena;
        }

        // acc : constant bound constraint
        double aViola = a_snorm - max_a_squared;
        if (smoothedL1(aViola, smoothFactor, violaAccPena, violaAccPenaD)) 
        {
          gradAcc += rho_a * violaAccPenaD * 2.0 * acc;
          pena += rho_a * violaAccPena;
        }

        // jer : constant bound constraint
        double jViola = j_snorm - max_j_squared;
        if (smoothedL1(jViola, smoothFactor, violaJerPena, violaJerPenaD)) 
        {
          gradJer += rho_j * violaJerPenaD * 2.0 * jer;
          pena += rho_j * violaJerPena;
        }

        totalGradPos.setZero();
        totalGradVel = gradVel;
        totalGradAcc = gradAcc;
        totalGradJer = gradJer;

        // add all grad into C,T
        // note that xy = Cxy*β(j/K*T_xy), yaw = Cyaw*β(i*T_xy+j/K*T_xy-yaw_idx*T_yaw)
        // ∂p/∂Cxy, ∂v/∂Cxy, ∂a/∂Cxy
        alpha = j * integralFrac;
        gdCxy.block<8, 3>(i * 8, 0) += (beta0_xy * totalGradPos.transpose() + 
                                        beta1_xy * totalGradVel.transpose() + 
                                        beta2_xy * totalGradAcc.transpose() + 
                                        beta3_xy * totalGradJer.transpose()) * omg * step;
        // ∂p/∂Txy, ∂v/∂Txy, ∂a/∂Txy
        gdTxy(i) += (totalGradPos.dot(vel) +
                                 totalGradVel.dot(acc) +
                                 totalGradAcc.dot(jer) +
                                 totalGradJer.dot(sna)) *
                                    alpha * omg * step +
                                omg * integralFrac * pena;
        
        cost += omg * step * pena;
        s1 += step;
      }
    }
  }
  // T = e^τ
  double expC2(const double& tau)
  {
    return tau > 0.0 ? ((0.5 * tau + 1.0) * tau + 1.0) : 1.0 / ((0.5 * tau - 1.0) * tau + 1.0);
  }
  // τ = ln(T)
  double logC2(const double& T)
  {
    return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
  }

  bool smoothedL1(const double &x, const double &mu, double &f, double &df)
  {
    if (x < 0.0)
    {
      return false;
    }
    else if (x > mu)
    {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    }
    else
    {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
      return true;
    }
  }

  static inline void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T)
  {
    const int sizeTau = tau.size();
    T.resize(sizeTau);
    for (int i = 0; i < sizeTau; i++)
    {
        T(i) = tau(i) > 0.0
                    ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                    : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau)
  {
    const int sizeT = T.size();
    tau.resize(sizeT);
    for (int i = 0; i < sizeT; i++)
    {
        tau(i) = T(i) > 1.0
                        ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                        : (1.0 - sqrt(2.0 / T(i) - 1.0));
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void backwardGradT(const Eigen::VectorXd &tau, const Eigen::VectorXd &gradT, EIGENVEC &gradTau)
  {
    const int sizeTau = tau.size();
    gradTau.resize(sizeTau);
    double denSqrt;
    for (int i = 0; i < sizeTau; i++)
    {
        if (tau(i) > 0)
        {
            gradTau(i) = gradT(i) * (tau(i) + 1.0);
        }
        else
        {
            denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
            gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
        }
    }
    return;
  }

private:
  /* Timer */
  ros::Timer traj_vis_timer_;

};

inline void TrajGenerator::setCorridor()
{
  pieceN = (int)route.size()-1;
  hPolyIdx.resize(pieceN);
  hPolyState.resize(pieceN, false);
  for(int i=0; i<pieceN; ++i)
  {
    Eigen::Vector3d pt1 = route[i];
    Eigen::Vector3d pt2 = route[i+1];
    for (int j=0; j<(int)hPolys.size(); ++j)
    {
      if (isPointInsidePolytope(hPolys[j], pt1, pt2))
      {
        hPolyIdx[i] = j;
        hPolyState[i] = true;
        break;
      }
    }
  }
  for (int i=0; i<(int)hPolyIdx.size(); ++i)
  {
    if (hPolyState[i] == false)
    {
      vector<Eigen::MatrixX4d> tempPolys;
      Eigen::Vector3d pt1 = route[i];
      Eigen::Vector3d pt2 = route[i+1];
      vector<Eigen::Vector3d> tempRoute = {pt1, pt2};
      if (i>1)
      {
        Eigen::Vector3d pt1Pre = route[i-1];
        tempRoute.insert(tempRoute.begin(), pt1Pre);
      }
      if (i<pieceN-1)
      {
        Eigen::Vector3d pt2Suc = route[i+2];
        tempRoute.push_back(pt2Suc);
      }
      sfc_gen::convexCover(tempRoute, pc, corridorMap.getOrigin(), corridorMap.getCorner(), 1.5*holyProg, 0.5*holyProg, tempPolys);
      int tempFlag = hPolys.size();
      hPolys.insert(hPolys.end(), tempPolys.begin(), tempPolys.end());
      for (int k=tempFlag; k<(int)hPolys.size(); ++k)
      {
        if (isPointInsidePolytope(hPolys[k], pt1, pt2))
        {
          hPolyIdx[i] = k;
          hPolyState[i] = true;
          break;
        }
      }
    }
  }
}

inline bool TrajGenerator::isPointInsidePolytope(Eigen::MatrixX4d& polytope, Eigen::Vector3d& point1, Eigen::Vector3d& point2)
{
  int numHalfSpaces = polytope.rows();
  for (int i = 0; i < numHalfSpaces; ++i)
  {
    Eigen::Vector3d outerNormal = polytope.block<1, 3>(i, 0);
    if ((outerNormal.dot(point1) + polytope(i, 3)) > 0 || (outerNormal.dot(point2) + polytope(i, 3)) > 0)
      return false;
  }

  return true;
}

inline void TrajGenerator::YawInterpolation(double& duration, double& start, double& end, vector<double>& newYaw, vector<double>& newDur, double& CompT)
{
  double yaw_start = start*180.0/M_PI;
  double yaw_end = end*180.0/M_PI;
  double yaw_gap = abs(yaw_end - yaw_start)>180.0? (360.0-abs(yaw_end - yaw_start)):abs(yaw_end - yaw_start);
  double omega = yaw_gap/duration;
  int cal_flag_yaw = yaw_end - yaw_start > 0? 1:-1;
  int cal_dir = abs(yaw_end - yaw_start)>180.0? -1:1;
  int num = floor(duration/CompT);
  double lastDur = duration-num*CompT;

  if (lastDur < 0.4*CompT)
  {
    num = num-1;
    lastDur += CompT;
  }

  double totalDur = 0.0;
  double tempYaw;
  for (int i=1; i<num+1; ++i)
  {
    totalDur += CompT;
    tempYaw = (yaw_start + cal_dir*cal_flag_yaw*omega*totalDur)*M_PI/180.0;
    while (tempYaw < -M_PI)
      tempYaw += 2 * M_PI;
    while (tempYaw > M_PI)
      tempYaw -= 2 * M_PI;
    newYaw.push_back(tempYaw);
    newDur.push_back(CompT);
  }
  newYaw.push_back(end);
  newDur.push_back(lastDur);
}

inline void TrajGenerator::getAngularVelInter(double& t, double& gap, double& omega)
{
  Eigen::Vector3d YAW = mincoYaw_traj.getPos(t);
  double yawAng = atan2(YAW(1), YAW(0)); 
  Eigen::Vector3d YAWPre = mincoYaw_traj.getPos(t-gap);
  double yawAngPre = atan2(YAWPre(1), YAWPre(0)); 
  double yaw_gap = abs(yawAng - yawAngPre)>M_PI? (2*M_PI-abs(yawAng - yawAngPre)):abs(yawAng - yawAngPre);
  omega = yaw_gap/gap;
}

inline void TrajGenerator::getPitch(double& t, double& pitch)
{
  Eigen::Vector3d ap = mincoPitch_traj.getPos(t);
  pitch = atan2(ap(1), ap(0)+1e-6);
}

inline void TrajGenerator::getPitchd(double& t, double& pd_)
{
  Eigen::Vector3d av = mincoPitch_traj.getVel(t);
  pd_ = av.norm();
}

inline void TrajGenerator::getYaw(double& t, double& yaw)
{
  Eigen::Vector3d ap = mincoYaw_traj.getPos(t);
  yaw = atan2(ap(1), ap(0)+1e-6);
}
/* angular velocity = d(string_length)/dt */
inline void TrajGenerator::getYawd(double& t, double& yd_)
{
  Eigen::Vector3d av = mincoYaw_traj.getVel(t);
  yd_ = av.norm();
}

}

#endif