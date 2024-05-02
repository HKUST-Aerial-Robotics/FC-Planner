/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of PerceptionUtils class, which 
 *                   implements perception utils for coverage in FC-Planner.
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

#ifndef _PERCEPTION_UTILS_H_
#define _PERCEPTION_UTILS_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <vector>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace predrecon {
class PerceptionUtils {
public:
  PerceptionUtils();
  ~PerceptionUtils();

  void init(ros::NodeHandle& nh);
  void setPose_PY(const Vector3d& pos, const double& pitch, const double& yaw);
  void getFOV_PY(vector<Vector3d>& list1, vector<Vector3d>& list2);
  bool insideFOV(const Vector3d& point);
  void getFOVBoundingBox(Vector3d& bmin, Vector3d& bmax);

private:
  // Data
  // Current camera pos and yaw
  Vector3d pos_;
  double pitch_, yaw_;
  // Camera plane's normals in world frame
  vector<Vector3d> normals_;

  /* Params */
  // Sensing range of camera
  double left_angle_, right_angle_, top_angle_, max_dist_, vis_dist_;
  // Normal vectors of camera FOV planes in camera frame
  Vector3d n_top_, n_bottom_, n_left_, n_right_;
  // Transform between camera and body
  Eigen::Matrix4d T_cb_, T_bc_;
  // FOV vertices in body frame
  vector<Vector3d> cam_vertices1_, cam_vertices2_;
};

} // namespace predrecon
#endif