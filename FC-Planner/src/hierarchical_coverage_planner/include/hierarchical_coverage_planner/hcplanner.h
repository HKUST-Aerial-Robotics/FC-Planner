/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
                     Mingjie Zhang <zagerzhang at gmail dot com>, STAR Group, SYSU.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of hierarchical_coverage_planner
 *                   class, which is hierarchical coverage planning in FC-Planner.
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

#ifndef _HCPLANNER_H_
#define _HCPLANNER_H_

#include <rosa/rosa_main.h>
#include <rosa/Extra_Del.h>
#include <hierarchical_coverage_planner/hcsolver.h>
#include <active_perception/perception_utils.h>
#include <viewpoint_manager/viewpoint_manager.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>
#include <traj_utils/planning_visualization.h>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <string>
#include <vector>
#include <list>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <gcopter/firi.hpp>
#include <gcopter/voxel_map.hpp>
#include <gcopter/sfc_gen.hpp>

using namespace std;
using std::unique_ptr;
using std::shared_ptr;

class RayCaster;

namespace predrecon
{

struct Vector3iCompare {
    bool operator()(const Eigen::Vector3i& v1, const Eigen::Vector3i& v2) const {
        if (v1(0) != v2(0)) return v1(0) < v2(0);
        if (v1(1) != v2(1)) return v1(1) < v2(1);
        return v1(2) < v2(2);
    }
};

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

class PlanningVisualization;
class ROSA_main;
class SDFMap;
class PerceptionUtils;
class HCSolver;
class ViewpointManager;

class hierarchical_coverage_planner
{

struct Viewpoint
{
  int sub_id;
  int vp_id;
  Eigen::VectorXd pose;
  int vox_count;
};

struct PathResult
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
  /* planning data */
  vector<Eigen::Vector3d> sub_centroids_;
  vector<int> global_seq_;
  map<int, vector<int>> global_boundary_id_; // sub-space: [start_id, end_id] 
  map<int, vector<Eigen::VectorXd>> local_paths_viewpts_; // sub-space: [viewpoint, viewpoint, ...]
  map<int, vector<vector<Eigen::VectorXd>>> local_paths_waypts_; // sub-space: [waypts, waypts, ...]
  vector<Eigen::VectorXd> full_viewpoints_;
  vector<vector<Eigen::VectorXd>> full_waypoints_;
  vector<Eigen::VectorXd> OriFullPath_;
  vector<Eigen::VectorXd> FullPath_;
  vector<bool> waypoints_indicators_;
  unordered_map<vector<double>, bool, VectorHash> viewpoints_detector_;
  unordered_map<vector<double>, int, VectorHash> FullPathId_;
  /* joint refine data */
  vector<Eigen::Vector3d> connectJoints;
  vector<vector<Eigen::Vector3d>> innerVps;
  double searchRange;
};

public:
  hierarchical_coverage_planner();
  ~hierarchical_coverage_planner();
  /* Func */
  void init(ros::NodeHandle& nh);
  void HCPlan(ros::NodeHandle& nh);
  double CoverageEvaluation(vector<Eigen::VectorXd>& poseSet);
  void PinHoleCamera(Eigen::VectorXd& vp, vector<int>& covered_id, pcl::PointCloud<pcl::PointXYZ>::Ptr& checkCloud);
  /* Data */
  PathResult PR;
  voxel_map::VoxelMap voxelMap;
  /* Param */
  bool visFlag;
  /* Utils */
  unique_ptr<ROSA_main> skeleton_operator;
  /* Statistic */
  double hcoppCT;
  double HCOPPTime;
  double corridorProgress;
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> visible_group;
  double coverage;
  int viewpointNum;
  /* Evaluation */
  string fullcloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr Fullmodel;
  pcl::PointCloud<pcl::PointXYZ>::Ptr visibleFullmodel;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* Func */
  void viewpointGeneration();
  void CoveragePlan(bool turn);
  /* Params */
  double dist_vp, model_ds_size, safe_radius_vp;
  double fov_h, fov_w, fov_base, voxel_size;
  double cx, cy, fx, fy;
  double normal_step;
  string mode_;
  double safe_inner_coefficient_, avg_inner_dist_;
  int inner_count_;
  double cost_cn_, JointCoeff;
  bool zFlag;
  double GroundZPos, safeHeight, droneRadius;
  /* Data */
  string mesh;
  Eigen::Vector3d current_pos_;
  pcl::PointCloud<pcl::PointXYZ> uncovered_area;
  vector<Eigen::VectorXd> valid_viewpoints;
  vector<double> pitch_set;
  vector<bool> seg_visited_buffer_;
  pcl::KdTreeFLANN<pcl::PointXYZ> model_tree, OriModelTree, ROSATree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr rosaCloud;
  pcl::PointCloud<pcl::PointNormal>::Ptr all_safe_normal_vps;
  vector<Eigen::Vector3d> globalPath;
  /* Utils */
  shared_ptr<PlanningVisualization> vis_utils_;
  shared_ptr<SDFMap> HCMap;
  shared_ptr<ViewpointManager> viewpoint_manager_;
  unique_ptr<RayCaster> raycaster_, raycaster_rev;
  unique_ptr<PerceptionUtils> percep_utils_;
  unique_ptr<HCSolver> solver_;
  
  void PlanningVisCallback(const ros::TimerEvent& e);
  void cutting_plane_normals();
  void plane_query(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, Eigen::Vector3d& point_on_plane, Eigen::Vector3d& plane_normal, double thickness);
  void JointRefinePath(bool turn);
  void CameraModelProjection(Eigen::VectorXd& pose, Eigen::Vector3d& point, double& xRes, double& yRes, Eigen::Vector2d& leftdown, double& distance, Eigen::Vector2i& inCam);
  bool lightStateDet(Eigen::Vector3d& vpCandi, Eigen::Vector3d& vpCandiNormal);
  Eigen::Vector3d pyToVec(Eigen::Vector2d& pitch_yaw);
  void interFullPath(double dist_bound);
  /* Timer */
  ros::Timer planning_vis_timer_;
};

} // namespace predrecon

#endif