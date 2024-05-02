/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of HCSolver class, which implements
 *                   basic solver for planning modules in FC-Planner.
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

#ifndef _HCSOLVER_H_
#define _HCSOLVER_H_

#include <path_searching/astar2.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <rosa/rosa_main.h>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/convex_hull.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <random>
#include <math.h>
#include <thread>
#include <unistd.h>
#include <mutex>
#include <condition_variable>
#include <iomanip>

using namespace std;
using std::unique_ptr;
using std::shared_ptr;

class RayCaster;

namespace predrecon
{

struct Site
{
  Site *Pred;
  Site *Suc;
  bool Start;
  bool End;
  double X;
  double Y;
  double Z;
  double Pitch;
  double Yaw;
  int GlobalID;
  int LocalID;
};

class Astar;
class SDFMap;

class HCSolver
{
public:
  HCSolver();
  ~HCSolver();

  /* Func */
  void init(ros::NodeHandle& nh, Eigen::Vector3d& path_start_, SDFMap::Ptr& hcmap, double& res_, Eigen::Vector3d& origin_);
  vector<int> GlobalSubspaceSequence(map<int, vector<Eigen::VectorXd>>& sub_vps);
  map<int, vector<int>> GlobalBoundaryPoints(map<int, vector<Eigen::VectorXd>>& sub_vps, vector<int>& globalseq);
  tuple<map<int, vector<Eigen::VectorXd>>, map<int, vector<vector<Eigen::VectorXd>>>> LocalConditionalPath(
    map<int, vector<Eigen::VectorXd>>& sub_vps, map<int, vector<int>>& global_boundary, bool turn);
  tuple<vector<Eigen::VectorXd>, vector<vector<Eigen::VectorXd>>> CoverageFullPath(
    Eigen::Vector3d& cur_pos, vector<int>& globalseq, map<int, vector<Eigen::VectorXd>>& localviewpts, map<int, vector<vector<Eigen::VectorXd>>>& localwaypts);
  vector<Eigen::VectorXd> LocalRefine(vector<Eigen::Vector3d>& Joints,double& Range, vector<Eigen::VectorXd>& Path, bool turn);
  /* Data */
  vector<Eigen::Vector3d> centroids;
  vector<vector<Eigen::Vector3d>> JointVps;
  /* Tools */
  void AngleInterpolation(Eigen::VectorXd& start, Eigen::VectorXd& end, vector<Eigen::Vector3d>& waypts_, vector<Eigen::VectorXd>& updates_waypts_);
  double search_Path(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, vector<Eigen::Vector3d>& path);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* Param */
  int precision_, system_back_, local_system_back_;
  string GlobalSolver_;
  string GlobalPar_, GlobalProF_, GlobalResult_;
  string LocalFolder_;
  int GlobalRuns_;
  int LocalRuns_;
  bool tripod_head_trigger_;
  double vm_, am_, yd_, ydd_, jm_;
  double bridge_length_, tempLength;
  int local2optNum;
  double astarSearchingRes;
  int swapTimes;
  /* Data */
  Eigen::Vector3d solver_start_;
  map<int, Eigen::MatrixXd> local_sub_costmat_;
  map<int, Eigen::MatrixXd> local_sub_phymat_;
  map<vector<double>, double> allAstarCost_; // vector: [i_pos(0), i_pos(1), i_pos(2), j_pos(0), j_pos(1), j_pos(2)] --> Astar cost
  map<vector<double>, vector<Eigen::VectorXd>> allAstarPath_; // vector: [i_pos(0), i_pos(1), i_pos(2), j_pos(0), j_pos(1), j_pos(2)] --> Astar search path
  map<int, vector<int>> local_sub_init_idx_; // boundary-conditioned initial ids of sub-space viewpoints
  // vector<int>: sub_id, viewpoint1, viewpoint2; vector<Eigen::Vector3d>: waypoints from Astar
  map<vector<int>, vector<Eigen::VectorXd>> path_waypts_;
  map<int, vector<Eigen::VectorXd>> local_sub_path_viewpts_;
  map<int, vector<vector<Eigen::VectorXd>>> local_sub_path_waypts_;
  /* Refine Data */
  vector<Eigen::Vector3d> LocalVps;
  map<Eigen::Vector3d, int, Vector3dCompare> RefineID;
  vector<Site*> AllPathSite;
  vector<Site*> LocalSite;
  double inter_cost;
  /* Sub-space Param */
  double phy_dist, opt_dist;
  /* Utils */ 
  unique_ptr<Astar> astar_;
  unique_ptr<RayCaster> solve_raycaster_;
  shared_ptr<SDFMap> solve_map_;
  SDFMap::Ptr mt_map_;
  ros::NodeHandle mt_nh_;
  Eigen::MatrixXd GlobalCostMat(Eigen::Vector3d& start_, vector<Eigen::Vector3d>& targets);
  void GlobalParWrite();
  void GlobalProblemWrite(Eigen::MatrixXd& costMat);
  vector<int> GlobalResultsRead();
  int FindSphereNearestPoint(Eigen::Vector3d& spherePtA, Eigen::Vector3d& spherePtB, vector<Eigen::VectorXd>& vps);
  void LocalFindPath(vector<Eigen::VectorXd> vps, vector<int> boundary_, int sub_id_, bool turn);
  void LocalPathFinder(vector<Eigen::VectorXd> vps, vector<int> boundary_, int sub_id_);
  /* Tools */
  double compute_Cost(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const double& y1, const double& y2, vector<Eigen::Vector3d>& path);
  double computeTimeCost(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const double& y1, const double& y2, vector<Eigen::Vector3d>& path);
  double compute_Cost_tripod_head(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const double& y1, const double& y2, vector<Eigen::Vector3d>& path);
  double computeTimeCost_tripod_head(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const double& y1, const double& y2, vector<Eigen::Vector3d>& path);
  void ConstructKDTree(vector<Eigen::VectorXd>& pathlist, pcl::KdTreeFLANN<pcl::PointXYZ>& tree);
  void Make2Opt(Site *s1, Site *s2, Site *s3, Site *s4, bool turn);
  void Make3Opt(Site *s1, Site *s2, Site *s3, Site *s4, Site *s5, Site *s6, bool turn);
  void RandomLocal2Opt(bool turn);
  void RandomLocal3Opt(bool turn);
  void shortenPath(vector<Eigen::Vector3d>& path);
  double kOptTimeCost(vector<Eigen::Vector3d>& path);
  /* Utils */
  double pathTimeUniAcc(Eigen::Vector3d& pred, Eigen::Vector3d& cur, Eigen::Vector3d& suc);
  vector<string> split(string str, string pattern);
};

inline vector<string> HCSolver::split(string str, string pattern)
{
  vector<string> ret;
  if (pattern.empty()) return ret;
  size_t start = 0, index = str.find_first_of(pattern, 0);
  while (index != str.npos)
  {
    if (start != index)
      ret.push_back(str.substr(start, index - start));
    start = index + 1;
    index = str.find_first_of(pattern, start);
  }
  if (!str.substr(start).empty())
    ret.push_back(str.substr(start));
  return ret;
}

} // namespace predrecon

#endif