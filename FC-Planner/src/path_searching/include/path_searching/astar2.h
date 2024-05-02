/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of Astar class, which implements
 *                   safe path searching in FC-Planner.
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

#ifndef _ASTAR2_H
#define _ASTAR2_H

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <path_searching/matrix_hash.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>
#include <sstream>

namespace predrecon {
class Node {
public:
  Eigen::Vector3i index;
  Eigen::Vector3d position;
  double g_score, f_score;
  Node* parent;

  /* -------------------- */
  Node() {
    parent = NULL;
  }
  ~Node(){};
};
typedef Node* NodePtr;

class NodeComparator0 {
public:
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

class Astar {
public:
  Astar();
  ~Astar();
  enum { REACH_END = 1, NO_PATH = 2 };

  void init_hc(ros::NodeHandle& nh, const SDFMap::Ptr& hc_map_);
  void reset();
  int hc_search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt);
  void setResolution(const double& res);
  static double pathLength(const vector<Eigen::Vector3d>& path);

  std::vector<Eigen::Vector3d> getPath();
  std::vector<Eigen::Vector3d> getVisited();
  double getEarlyTerminateCost();

  double lambda_heu_hc_;
  double max_search_time_;
  bool zFlag;
  double groundz, safeheight; 

private:
  void backtrack(const NodePtr& end_node, const Eigen::Vector3d& end);
  void posToIndex(const Eigen::Vector3d& pt, Eigen::Vector3i& idx);
  double getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);
  double getManhHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);
  double getEuclHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);

  // main data structure
  vector<NodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
  std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> open_set_map_;
  std::unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> close_set_map_;
  std::vector<Eigen::Vector3d> path_nodes_;
  double early_terminate_cost_;

  EDTEnvironment::Ptr edt_env_;
  SDFMap::Ptr map_hc_;

  // parameter
  double margin_;
  int allocate_num_;
  double tie_breaker_;
  double resolution_, inv_resolution_;
  Eigen::Vector3d map_size_3d_, origin_;
};

}  // namespace predrecon

#endif