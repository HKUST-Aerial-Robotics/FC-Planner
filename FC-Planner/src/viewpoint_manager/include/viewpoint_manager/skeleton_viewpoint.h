/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of SkeletonViewpoint class, which gives
 *                   one example of independent skeleton-guided viewpoint generation.
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

#ifndef _SKELETON_VIEWPOINT_H_
#define _SKELETON_VIEWPOINT_H_

#include <rosa/rosa_main.h>
#include <rosa/Extra_Del.h>
#include <viewpoint_manager/viewpoint_manager.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>
#include <active_perception/perception_utils.h>
#include <traj_utils/planning_visualization.h>
#include <ros/ros.h>

using namespace std;
using std::unique_ptr;
using std::shared_ptr;

class RayCaster;

namespace predrecon
{

class PlanningVisualization;
class ROSA_main;
class SDFMap;
class PerceptionUtils;

class SkeletonViewpoint
{

struct VectorHash {
    size_t operator()(const std::vector<double>& v) const {
        std::hash<double> hasher;
        size_t seed = 0;
        for (double i : v) {
            seed ^= hasher(i) + 0x9e3779b9 + (seed<<6) + (seed>>2);
        }
        return seed;
    }
};

struct Viewpoint
{
  int sub_id;
  int vp_id;
  Eigen::VectorXd pose;
  int vox_count;
};

struct ViewpointResult
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ori_model;
  pcl::PointCloud<pcl::PointXYZ>::Ptr occ_model;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model;
  map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> sub_vps_inflate;
  map<int, Eigen::MatrixXd> sub_vps_pose;
  map<int, vector<bool>> cover_state; // [int: seg_id], coverage state of each voxel in each segment
  map<int, vector<double>> cover_contrib; // [int: seg_id], the contribution degree of covered viewpoints to this voxel in each segment
  map<int, vector<Eigen::Vector2i>> contrib_id; // [int: seg_id], the index of covered viewpoints to this voxel in each segment
  map<Eigen::Vector3d, int, Vector3dCompare> vp_seg_pairs; // this viewpoints sampled from which segment
  map<Eigen::Vector3d, Eigen::Vector3d, Vector3dCompare0> pt_normal_pairs;
  vector<Eigen::VectorXd> outer_normals;
  map<Eigen::Vector3d, Eigen::Vector3d, Vector3dCompare> pt_proj_pairs;
  map<Eigen::Vector3d, int, Vector3dCompare> pt_sub_pairs;
  map<Eigen::Vector3d, Eigen::Vector3d, Vector3dCompare> pt_vp_pairs;
  map<Eigen::Vector3d, Eigen::Vector3d, Vector3dCompare> vp_pt_pairs;
  /* for uncovered viewpoints evaluation */
  map<Eigen::Vector3d, Eigen::Vector3d, Vector3dCompare> vp_direction_pairs;
  /* final viewpoints */
  vector<Viewpoint> vps_set_;
  map<int, vector<Eigen::VectorXd>> final_sub_vps_pairs;
};

public:
  SkeletonViewpoint();
  ~SkeletonViewpoint();
  /* Func */
  void init(ros::NodeHandle& nh);
  void execution(ros::NodeHandle& nh);
  /* Data */
  ViewpointResult PR;
  /* Utils */
  unique_ptr<ROSA_main> skeleton_operator;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* Func */
  void viewpointGeneration();
  /* Params */
  double dist_vp, model_ds_size, safe_radius_vp;
  double fov_h, fov_w, fov_base;
  double normal_step;
  double avg_inner_dist_;
  int inner_count_;
  bool gimbal, zFlag;
  double GroundZPos, safeHeight;
  string fullcloud;
  /* Data */
  string mesh;
  pcl::PointCloud<pcl::PointXYZ> uncovered_area;
  vector<Eigen::VectorXd> valid_viewpoints;
  vector<double> pitch_set;
  vector<bool> seg_visited_buffer_;
  pcl::KdTreeFLANN<pcl::PointXYZ> model_tree, OriModelTree, ROSATree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr rosaCloud;
  pcl::PointCloud<pcl::PointNormal>::Ptr all_safe_normal_vps;
  /* Utils */
  shared_ptr<PlanningVisualization> vis_utils_;
  shared_ptr<SDFMap> HCMap;
  shared_ptr<ViewpointManager> viewpoint_manager_;
  unique_ptr<RayCaster> raycaster_;
  unique_ptr<PerceptionUtils> percep_utils_;

  void visCallback(const ros::TimerEvent& e);
  void cutting_plane_normals();
  void plane_query(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, Eigen::Vector3d& point_on_plane, Eigen::Vector3d& plane_normal, double thickness);
  bool lightStateDet(Eigen::Vector3d& vpCandi, Eigen::Vector3d& vpCandiNormal);
  Eigen::Vector3d pyToVec(Eigen::Vector2d& pitch_yaw);
  /* Timer */
  ros::Timer vis_timer_;
};

} // namespace predrecon

#endif