/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
                     Mingjie Zhang <zagerzhang at gmail dot com>, STAR Group, SYSU.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main algorithm of hierarchical coverage planning
 *                   in FC-Planner.
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

#include <hierarchical_coverage_planner/hcplanner.h>

namespace predrecon
{
  hierarchical_coverage_planner::hierarchical_coverage_planner()
  {
  }

  hierarchical_coverage_planner::~hierarchical_coverage_planner()
  {
  }

  void hierarchical_coverage_planner::init(ros::NodeHandle &nh)
  {
    // * Module Initialization
    skeleton_operator.reset(new ROSA_main);
    HCMap.reset(new SDFMap);
    percep_utils_.reset(new PerceptionUtils);
    solver_.reset(new HCSolver);
    vis_utils_.reset(new PlanningVisualization(nh));
    viewpoint_manager_.reset(new ViewpointManager);
    raycaster_.reset(new RayCaster);

    skeleton_operator->init(nh);
    percep_utils_->init(nh);
    viewpoint_manager_->init(nh);

    // * Visualization
    skeleton_operator->visFlag = false;
    visFlag = false;

    // * Params Initialization
    nh.param("hcplanner/mesh", mesh, string("null"));
    nh.param("hcplanner/model_downsample_size", model_ds_size, -1.0);
    nh.param("hcplanner/cx", cx, -1.0);
    nh.param("hcplanner/cy", cy, -1.0);
    nh.param("hcplanner/fx", fx, -1.0);
    nh.param("hcplanner/fy", fy, -1.0);
    nh.param("hcplanner/sample_step4normal", normal_step, -1.0);
    nh.param("hcplanner/exec_mode", mode_, string("null"));
    nh.param("hcplanner/safe_inner_dist_coeff", safe_inner_coefficient_, -1.0);
    nh.param("hcplanner/JointRadius", JointCoeff, -1.0);
    nh.param("hctraj/drone_radius", droneRadius, -1.0);
    nh.param("hcplanner/current_x_", current_pos_(0), -1.0);
    nh.param("hcplanner/current_y_", current_pos_(1), -1.0);
    nh.param("hcplanner/current_z_", current_pos_(2), -1.0);
    nh.param("viewpoint_manager/viewpoints_distance", dist_vp, -1.0);
    nh.param("viewpoint_manager/fov_h", fov_h, -1.0);
    nh.param("viewpoint_manager/fov_w", fov_w, -1.0);
    nh.param("viewpoint_manager/zGround", zFlag, false);
    nh.param("viewpoint_manager/GroundPos", GroundZPos, -1.0);
    nh.param("viewpoint_manager/safeHeight", safeHeight, -1.0);
    nh.param("viewpoint_manager/safe_radius", safe_radius_vp, -1.0);

    fov_base = min(fov_h, fov_w) * M_PI / 360.0;
    corridorProgress = 2.0 * dist_vp * tan(fov_base);

    // * Evaluation
    nh.param("hcplanner/fullcloud", fullcloud, string("null"));
    Fullmodel.reset(new pcl::PointCloud<pcl::PointXYZ>);
    visibleFullmodel.reset(new pcl::PointCloud<pcl::PointXYZ>);
    PR.occ_model.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fullcloud, *PR.occ_model);

    // * Mapping & Solver & Bidirectional Ray Casting (BiRC)
    HCMap->initHCMap(nh, PR.occ_model);
    solver_->init(nh, HCMap->hcmp_->resolution_, HCMap->hcmp_->map_origin_);
    raycaster_->setParams(HCMap->hcmp_->resolution_, HCMap->hcmp_->map_origin_);

    // * Timer
    planning_vis_timer_ = nh.createTimer(ros::Duration(0.5), &hierarchical_coverage_planner::PlanningVisCallback, this);

    ROS_INFO("\033[35m[Planner] Initialized! \033[32m");
  }

  void hierarchical_coverage_planner::HCPlan(ros::NodeHandle &nh)
  {
    auto plan_t1 = std::chrono::high_resolution_clock::now();

    // * Skeleton-based Space Decomposition
    skeleton_operator->main();
    rosaCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ vertex;
    for (int i = 0; i < (int)skeleton_operator->P.realVertices.rows(); ++i)
    {
      vertex.x = skeleton_operator->P.realVertices(i, 0);
      vertex.y = skeleton_operator->P.realVertices(i, 1);
      vertex.z = skeleton_operator->P.realVertices(i, 2);
      rosaCloud->points.push_back(vertex);
    }
    ROSATree.setInputCloud(rosaCloud);

    // * Uniform Downsampling for Acceleration
    PR.ori_model.reset(new pcl::PointCloud<pcl::PointXYZ>);
    PR.model.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> ds;

    map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> temp_seg;
    for (auto &pair : skeleton_operator->P.seg_clouds_scale)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*pair.second, *tempcloud);
      temp_seg[pair.first] = tempcloud;
    }

    // * Ground Constraint
    if (zFlag == true)
    {
      for (auto &seg_cloud : skeleton_operator->P.seg_clouds_scale)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto pt : seg_cloud.second->points)
        {
          if (pt.z > GroundZPos)
            groundCloud->points.push_back(pt);
        }
        seg_cloud.second = groundCloud;
      }
    }

    // * Initialize Mapping
    for (const auto &seg_cloud : skeleton_operator->P.seg_clouds_scale)
    {
      *PR.ori_model += *seg_cloud.second;
      ds.setInputCloud(seg_cloud.second);
      ds.setLeafSize(model_ds_size, model_ds_size, model_ds_size);
      ds.filter(*seg_cloud.second);
      pcl::PointCloud<pcl::PointXYZ>::Ptr tempSeg(new pcl::PointCloud<pcl::PointXYZ>);
      for (int i = 0; i < (int)seg_cloud.second->points.size(); ++i)
      {
        Eigen::Vector3d pt_vec;
        pt_vec << seg_cloud.second->points[i].x, seg_cloud.second->points[i].y, seg_cloud.second->points[i].z;
        Eigen::Vector3i pt_index;
        HCMap->posToIndex_hc(pt_vec, pt_index);
        if (HCMap->hcmd_->occupancy_buffer_hc_[HCMap->toAddress_hc(pt_index)] == 1)
          tempSeg->points.push_back(seg_cloud.second->points[i]);
      }
      *PR.model += *tempSeg;
    }
    ROS_INFO("\033[33m[Planner] input points size = %d. \033[32m", (int)PR.model->points.size());
    model_tree.setInputCloud(PR.model);
    OriModelTree.setInputCloud(PR.ori_model);

    cutting_plane_normals();

    // * Initialize VoxelMap for Safe Flight Corridor (SFCs)
    auto hcmap_t1 = std::chrono::high_resolution_clock::now();

    HCMap->InternalSpace(temp_seg, skeleton_operator->P.vertices_scale, skeleton_operator->P.segments);
    HCMap->OuterCheck(PR.outer_normals);

    double min_z_voxelMap;
    if (zFlag == true)
      min_z_voxelMap = GroundZPos;
    else
      min_z_voxelMap = HCMap->hcmp_->map_min_boundary_(2);

    Eigen::Vector3i xyz((HCMap->hcmp_->map_max_boundary_(0) - HCMap->hcmp_->map_min_boundary_(0)) / HCMap->hcmp_->resolution_,
                        (HCMap->hcmp_->map_max_boundary_(1) - HCMap->hcmp_->map_min_boundary_(1)) / HCMap->hcmp_->resolution_,
                        (HCMap->hcmp_->map_max_boundary_(2) - min_z_voxelMap) / HCMap->hcmp_->resolution_);
    Eigen::Vector3d offset(HCMap->hcmp_->map_min_boundary_(0), HCMap->hcmp_->map_min_boundary_(1), min_z_voxelMap);
    voxelMap = voxel_map::VoxelMap(xyz, offset, HCMap->hcmp_->resolution_);
    for (int i = 0; i < (int)PR.ori_model->points.size(); ++i)
      voxelMap.setOccupied(Eigen::Vector3d(PR.ori_model->points[i].x, PR.ori_model->points[i].y, PR.ori_model->points[i].z));
    voxelMap.dilate(std::ceil(droneRadius / voxelMap.getScale()));

    auto hcmap_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> hcmap_ms = hcmap_t2 - hcmap_t1;
    double hcmap_time = (double)hcmap_ms.count();
    ROS_INFO("\033[33m[Planner] global mapping time = %lf ms. \033[32m", hcmap_time);

    // * Initialize Path Solver
    solver_->setStart(current_pos_);
    solver_->setMap(HCMap);

    // * Skeleton-guided Viewpoint Generation
    auto vpg_t1 = std::chrono::high_resolution_clock::now();

    viewpointGeneration();

    auto vpg_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> vpg_ms = vpg_t2 - vpg_t1;
    double vpg_time = (double)vpg_ms.count();
    ROS_INFO("\033[33m[Planner] skeleton-guided viewpoint generation time = %lf ms. \033[32m", vpg_time);

    // * All Active Viewpoints
    vector<Eigen::VectorXd>().swap(valid_viewpoints);
    Eigen::VectorXd valid_vp;
    int sub_space_id = -1;
    map<int, vector<Eigen::VectorXd>> init_vp_pairs;
    for (auto vp : PR.vps_set_)
    {
      sub_space_id = vp.sub_id;
      valid_vp = vp.pose;
      init_vp_pairs[sub_space_id].push_back(valid_vp);
    }

    int new_id = 0, valid_id = 0;
    vector<int> validBranchID;
    for (const auto &p : init_vp_pairs)
    {
      if (p.second.size() > 0)
      {
        validBranchID.push_back(p.first);
        PR.final_sub_vps_pairs[valid_id] = p.second;
        valid_viewpoints.insert(valid_viewpoints.end(), p.second.begin(), p.second.end());
        valid_id++;
      }
      new_id++;
    }
    viewpointNum = (int)valid_viewpoints.size();

    vector<vector<int>> newBranches;
    for (int i = 0; i < (int)validBranchID.size(); ++i)
      newBranches.push_back(skeleton_operator->P.branches[validBranchID[i]]);
    skeleton_operator->P.branches.clear();
    skeleton_operator->P.branches = newBranches;

    // * Hierarchical Coverage Planning
    CoveragePlan(false);

    auto plan_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> plan_ms = plan_t2 - plan_t1;
    hcoppCT = (double)plan_ms.count();

    vector<Eigen::VectorXd> HCOPPPath(PR.FullPath_.begin() + 1, PR.FullPath_.end());
    ROS_INFO("\033[35m[Planner] --- <Planner finished> --- \033[35m");
    // * Coverage Evaluation
    auto cove_t1 = std::chrono::high_resolution_clock::now();

    *Fullmodel = *PR.occ_model;
    vector<bool> CoverState;
    CoverState.resize(Fullmodel->points.size(), false);
    for (auto v : HCOPPPath)
    {
      vector<int> tempCids;
      PinHoleCamera(v, tempCids, Fullmodel);
      for (auto x : tempCids)
        CoverState[x] = true;
    }
    int numTrue = 0;
    for (int s = 0; s < (int)CoverState.size(); ++s)
    {
      if (CoverState[s] == true)
      {
        numTrue++;
        visibleFullmodel->points.push_back(Fullmodel->points[s]);
      }
    }
    coverage = (double)numTrue / (double)Fullmodel->points.size();

    auto cove_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> cove_ms = cove_t2 - cove_t1;
    double cove_time = (double)cove_ms.count();
    ROS_INFO("\033[36m[CoverageAnalyzer] FC-Planner path coverage evaluation time = %lf s. \033[32m", cove_time / 1000.0);

    // * Only Visualize Intermediate State for Debugging
    if (mode_ == "debug")
    {
      pcl::PointXYZ ipt;
      for (int x = HCMap->hcmp_->box_min_(0) /* + 1 */; x < HCMap->hcmp_->box_max_(0); ++x)
        for (int y = HCMap->hcmp_->box_min_(1) /* + 1 */; y < HCMap->hcmp_->box_max_(1); ++y)
          for (int z = HCMap->hcmp_->box_min_(2) /* + 1 */; z < HCMap->hcmp_->box_max_(2); ++z)
          {
            if (HCMap->hcmd_->occupancy_buffer_internal_[HCMap->toAddress_hc(x, y, z)] == 1)
            {
              Eigen::Vector3d pos;
              HCMap->indexToPos_hc(Eigen::Vector3i(x, y, z), pos);
              ipt.x = pos(0);
              ipt.y = pos(1);
              ipt.z = pos(2);
              HCMap->hcmd_->internal_cloud.push_back(ipt);
            }
          }

      pcl::PointXYZ occpt;
      for (int x = HCMap->hcmp_->box_min_(0) /* + 1 */; x < HCMap->hcmp_->box_max_(0); ++x)
        for (int y = HCMap->hcmp_->box_min_(1) /* + 1 */; y < HCMap->hcmp_->box_max_(1); ++y)
          for (int z = HCMap->hcmp_->box_min_(2) /* + 1 */; z < HCMap->hcmp_->box_max_(2); ++z)
          {
            if (HCMap->hcmd_->occupancy_buffer_hc_[HCMap->toAddress_hc(x, y, z)] == 1)
            {
              Eigen::Vector3d pos;
              HCMap->indexToPos_hc(Eigen::Vector3i(x, y, z), pos);
              occpt.x = pos(0);
              occpt.y = pos(1);
              occpt.z = pos(2);
              HCMap->hcmd_->occ_cloud.push_back(occpt);
            }
          }
    }
  }

  void hierarchical_coverage_planner::CoveragePlan(bool turn)
  {
    auto cpt_t1 = std::chrono::high_resolution_clock::now();

    // * Global Sequence Planning
    auto global_t1 = std::chrono::high_resolution_clock::now();
    PR.global_seq_ = solver_->GlobalSubspaceSequence(PR.final_sub_vps_pairs);
    PR.sub_centroids_ = solver_->centroids;
    auto global_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> global_ms = global_t2 - global_t1;
    double global_time = (double)global_ms.count();
    ROS_INFO("\033[33m[Planner] global sequence planning time = %lf ms.\033[32m", global_time);

    // * Local Boundary Selection
    auto boundary_t1 = std::chrono::high_resolution_clock::now();
    PR.global_boundary_id_ = solver_->GlobalBoundaryPoints(PR.final_sub_vps_pairs, PR.global_seq_);
    auto boundary_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> boundary_ms = boundary_t2 - boundary_t1;
    double boundary_time = (double)boundary_ms.count();
    ROS_INFO("\033[33m[Planner] local boundary selection time = %lf ms.\033[32m", boundary_time);

    // * Parallel Local Path Planning
    auto plan_t1 = std::chrono::high_resolution_clock::now();
    tuple<map<int, vector<Eigen::VectorXd>>, map<int, vector<vector<Eigen::VectorXd>>>> LCPResults = solver_->LocalConditionalPath(PR.final_sub_vps_pairs, PR.global_boundary_id_, turn);
    PR.local_paths_viewpts_ = get<0>(LCPResults);
    PR.local_paths_waypts_ = get<1>(LCPResults);
    auto plan_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> plan_ms = plan_t2 - plan_t1;
    double local_time = (double)plan_ms.count();
    ROS_INFO("\033[33m[Planner] parallel local path planning time = %lf ms.\033[32m", local_time);

    // * Convert Global Path
    tuple<vector<Eigen::VectorXd>, vector<vector<Eigen::VectorXd>>> FullResults = solver_->CoverageFullPath(current_pos_, PR.global_seq_, PR.local_paths_viewpts_, PR.local_paths_waypts_);
    PR.full_viewpoints_ = get<0>(FullResults);
    PR.full_waypoints_ = get<1>(FullResults);

    for (int i = 0; i < (int)PR.full_viewpoints_.size(); ++i)
    {
      vector<double> vp_finder;
      for (int v = 0; v < (int)PR.full_viewpoints_[i].size(); ++v)
        vp_finder.push_back(PR.full_viewpoints_[i](v));
      PR.viewpoints_detector_[vp_finder] = true;

      if ((int)PR.FullPath_.size() > 0)
      {
        // Search collision-free path between endpoint of current fullpath and startpoint of current local path
        vector<Eigen::Vector3d> cf_path_;
        vector<Eigen::VectorXd> updated_cf_path_;
        cost_cn_ = solver_->search_Path(PR.FullPath_.back().head(3), PR.full_viewpoints_[i].head(3), cf_path_);
        if ((int)cf_path_.size() > 2)
        {
          solver_->AngleInterpolation(PR.FullPath_.back(), PR.full_viewpoints_[i], cf_path_, updated_cf_path_);
          Eigen::Vector3d lastPos = PR.FullPath_.back().head(3);
          for (auto pose : updated_cf_path_)
          {
            Eigen::Vector3d curPos = pose.head(3);
            if ((lastPos - curPos).norm() < 1e-3)
              continue;
            PR.FullPath_.push_back(pose);
          }
        }
      }

      PR.FullPath_.push_back(PR.full_viewpoints_[i]);
      if ((int)PR.full_waypoints_[i].size() > 0 && i < (int)PR.full_viewpoints_.size() - 1)
      {
        Eigen::VectorXd seg_start = PR.full_viewpoints_[i];
        Eigen::VectorXd seg_end = PR.full_viewpoints_[i + 1];
        vector<Eigen::VectorXd> pose_waypts_;
        vector<Eigen::VectorXd>().swap(pose_waypts_);
        if ((int)PR.full_waypoints_[i].size() > 0)
        {
          Eigen::Vector3d lastPos = PR.FullPath_.back().head(3);
          for (auto pose : PR.full_waypoints_[i])
          {
            Eigen::Vector3d curPos = pose.head(3);
            if ((lastPos - curPos).norm() < 1e-3)
              continue;
            PR.FullPath_.push_back(pose);
          }
        }
      }
    }

    for (int i = 0; i < (int)PR.FullPath_.size(); ++i)
    {
      vector<double> finder;
      for (int j = 0; j < (int)PR.FullPath_[i].size(); ++j)
        finder.push_back(PR.FullPath_[i](j));
      PR.FullPathId_[finder] = i;
    }

    // * Local Path Refinement
    auto refine_t1 = std::chrono::high_resolution_clock::now();
    JointRefinePath(false);
    auto refine_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> refine_ms = refine_t2 - refine_t1;
    double refine_time = (double)refine_ms.count();
    ROS_INFO("\033[33m[Planner] local path refinement time = %lf ms.\033[32m", refine_time);

    // * Determine Waypoints
    PR.waypoints_indicators_.clear();
    PR.waypoints_indicators_.resize(PR.FullPath_.size(), false);
    for (int i = 0; i < (int)PR.FullPath_.size(); ++i)
    {
      vector<double> v_finder;
      for (int j = 0; j < (int)PR.FullPath_[i].size(); ++j)
        v_finder.push_back(PR.FullPath_[i](j));
      if (PR.viewpoints_detector_.find(v_finder) == PR.viewpoints_detector_.end())
      {
        PR.waypoints_indicators_[i] = true;
      }
    }

    // * Interpolate Path within Corridor
    double dist_upper = 0.6 * corridorProgress;
    interFullPath(dist_upper);

    auto cpt_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> cpt_ms = cpt_t2 - cpt_t1;
    double cpt_time = (double)cpt_ms.count();
    ROS_INFO("\033[34m[Planner] Hierarchical Coverage Planning latency = %lf ms.\033[32m", cpt_time);
  }

  void hierarchical_coverage_planner::PlanningVisCallback(const ros::TimerEvent &e)
  {
    if (visFlag == true)
    {
      // * revised normals using internal space
      pcl::PointCloud<pcl::PointXYZ> scene_model;
      pcl::PointCloud<pcl::Normal> scene_normals;

      pcl::PointXYZ pt_;
      pcl::Normal n_;
      for (const auto &pt_normal : PR.pt_normal_pairs)
      {
        pt_.x = pt_normal.first(0);
        pt_.y = pt_normal.first(1);
        pt_.z = pt_normal.first(2);
        n_.normal_x = pt_normal.second(0);
        n_.normal_y = pt_normal.second(1);
        n_.normal_z = pt_normal.second(2);
        scene_model.points.push_back(pt_);
        scene_normals.points.push_back(n_);
      }

      // * initially sampled viewpoints
      vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vps;
      for (const auto &s_v : PR.sub_vps_inflate)
        vps.push_back(s_v.second);

      // * fov visualization of all active viewpoints
      vector<vector<Eigen::Vector3d>> total1, total2;
      vector<Eigen::Vector3d> l1, l2;
      Eigen::Vector3d pos;
      double pitch, yaw;
      for (int i = 0; i < (int)valid_viewpoints.size(); ++i)
      {

        pos(0) = valid_viewpoints[i](0);
        pos(1) = valid_viewpoints[i](1);
        pos(2) = valid_viewpoints[i](2);
        pitch = valid_viewpoints[i](3);
        yaw = valid_viewpoints[i](4);
        percep_utils_->setPose_PY(pos, pitch, yaw);
        percep_utils_->getFOV_PY(l1, l2);

        total1.push_back(l1);
        total2.push_back(l2);
      }

      // * updated viewpoints in each sub-space
      map<int, vector<vector<Eigen::Vector3d>>> sub_list1, sub_list2;
      map<int, vector<double>> sub_yaws;
      vector<Eigen::Vector3d> l1_, l2_;
      Eigen::Vector3d pos_;
      double pitch_, yaw_;
      int s_id;
      for (auto &sub_vps : PR.final_sub_vps_pairs)
      {
        s_id = sub_vps.first;
        for (int i = 0; i < (int)sub_vps.second.size(); ++i)
        {

          pos_(0) = sub_vps.second[i](0);
          pos_(1) = sub_vps.second[i](1);
          pos_(2) = sub_vps.second[i](2);
          pitch_ = sub_vps.second[i](3);
          yaw_ = sub_vps.second[i](4);
          percep_utils_->setPose_PY(pos_, pitch_, yaw_);
          percep_utils_->getFOV_PY(l1_, l2_);

          sub_list1[s_id].push_back(l1_);
          sub_list2[s_id].push_back(l2_);
          sub_yaws[s_id].push_back(yaw_);
        }
      }

      // * coverage path visualization
      vector<Eigen::VectorXd> vis_full_path(PR.FullPath_.begin() + 1, PR.FullPath_.end());

      // * publish visualization
      vis_utils_->publishMesh(mesh);
      vis_utils_->publishVisCloud(visibleFullmodel);
      vis_utils_->publishRevisedNormal(scene_model, scene_normals);
      vis_utils_->publishSegViewpoints(vps);
      vis_utils_->publishInternal(HCMap->hcmd_->internal_cloud);
      vis_utils_->publishOccupied(HCMap->hcmd_->occ_cloud);
      vis_utils_->publishUncovered(uncovered_area);
      vis_utils_->publishInitVps(all_safe_normal_vps);
      vis_utils_->publishFOV(total1, total2);
      vis_utils_->publishFinalFOV(sub_list1, sub_list2, sub_yaws);
      vis_utils_->publishGlobalSeq(current_pos_, PR.sub_centroids_, PR.global_seq_);
      vis_utils_->publishGlobalBoundary(current_pos_, PR.global_boundary_id_, PR.final_sub_vps_pairs, PR.global_seq_);
      vis_utils_->publishLocalPath(PR.local_paths_viewpts_);
      vis_utils_->publishHCOPPPath(vis_full_path);
      vis_utils_->publishJointSphere(PR.connectJoints, PR.searchRange, PR.innerVps);
    }
  }

  void hierarchical_coverage_planner::viewpointGeneration()
  {
    /* ---------- Viewpoints Sub-space Sampling ---------- */
    pcl::PointCloud<pcl::PointNormal>::Ptr seg_vps(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_vps(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr safe_vps_in_seg(new pcl::PointCloud<pcl::PointXYZ>);
    all_safe_normal_vps.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointXYZ vp;
    Eigen::Vector3d safe_check_pt;
    map<Eigen::Vector3d, int, Vector3dCompare> vp_search_sub_;
    map<int, vector<int>> temp_sub_vps_voxcount, before_sub_vps_voxcount;

    /* normal-based inflate sampling */
    int sub_id = 0, dist_count = 0, vp_count = 0, safe_vp_count = 0;
    double avg_dist = 0.0;
    Eigen::Vector3d p1, p2;
    vector<int> segments_id;
    Eigen::Vector3d pt_vec, normal_dir, vp_vec;
    pcl::PointXYZ pt_;
    pcl::PointNormal vp_fov_normal;

    vector<int> origin_sub_ids;
    for (auto sub_space : skeleton_operator->P.sub_space_scale)
    {
      seg_vps.reset(new pcl::PointCloud<pcl::PointNormal>);
      safe_vps_in_seg.reset(new pcl::PointCloud<pcl::PointXYZ>);
      /* find all segments in this sub-space */
      vector<int>().swap(segments_id);
      segments_id = skeleton_operator->P.branch_seg_pairs[sub_id];

      for (auto seg_id : segments_id)
      {
        for (int i = 0; i < (int)skeleton_operator->P.seg_clouds_scale[seg_id]->points.size(); ++i)
        {
          pt_ = skeleton_operator->P.seg_clouds_scale[seg_id]->points[i];
          pt_vec(0) = pt_.x;
          pt_vec(1) = pt_.y;
          pt_vec(2) = pt_.z;
          PR.pt_sub_pairs[pt_vec] = sub_id;
          // ! /* rosa orientation viewpoint generation */
          if (PR.pt_normal_pairs.find(pt_vec) != PR.pt_normal_pairs.end())
            normal_dir = PR.pt_normal_pairs.find(pt_vec)->second;
          else
            continue;

          vp.x = pt_.x + dist_vp * normal_dir(0);
          vp.y = pt_.y + dist_vp * normal_dir(1);
          vp.z = pt_.z + dist_vp * normal_dir(2);

          Eigen::Vector3d vp_det(vp.x, vp.y, vp.z);
          bool det = lightStateDet(vp_det, normal_dir);
          if (det == false)
          {
            continue;
          }

          if (zFlag == true)
          {
            if (pt_.z > skeleton_operator->groundHeight)
            {
              if (vp.z > GroundZPos + safeHeight)
              {
                vp_vec(0) = vp.x;
                vp_vec(1) = vp.y;
                vp_vec(2) = vp.z;
                basic_vps->points.push_back(vp);
                vp_search_sub_[vp_vec] = sub_id;

                vp_fov_normal.x = vp.x;
                vp_fov_normal.y = vp.y;
                vp_fov_normal.z = vp.z;
                vp_fov_normal.normal_x = -normal_dir(0);
                vp_fov_normal.normal_y = -normal_dir(1);
                vp_fov_normal.normal_z = -normal_dir(2);

                seg_vps->points.push_back(vp_fov_normal);
                PR.pt_vp_pairs[pt_vec] = vp_vec;
                PR.vp_pt_pairs[vp_vec] = pt_vec;
                p1(0) = pt_.x;
                p1(1) = pt_.y;
                p1(2) = pt_.z;
                p2(0) = vp.x;
                p2(1) = vp.y;
                p2(2) = vp.z;
                avg_dist += (p1 - p2).norm();
                dist_count++;
                PR.vp_seg_pairs[p2] = seg_id;
              }
              // for safety of viewpoints under ground constraint
              else
              {
                vp_vec(0) = vp.x;
                vp_vec(1) = vp.y;
                vp_vec(2) = GroundZPos + safeHeight;
                pcl::PointXYZ tmp_vp = vp; // zmj
                tmp_vp.z = vp_vec(2);
                basic_vps->points.push_back(tmp_vp);
                vp_search_sub_[vp_vec] = sub_id;

                vp_fov_normal.x = vp_vec(0);
                vp_fov_normal.y = vp_vec(1);
                vp_fov_normal.z = vp_vec(2);
                Eigen::Vector3d groundsite(pt_.x, pt_.y, pt_.z);
                Eigen::Vector3d new_dir = (groundsite - vp_vec).normalized();
                // new safe direction
                double z_new = sqrt(pow(new_dir(2), 2) * (pow(normal_dir(0), 2) + pow(normal_dir(1), 2)) / (pow(new_dir(0), 2) + pow(new_dir(1), 2)));
                z_new = new_dir(2) > 0 ? z_new : -z_new;
                vp_fov_normal.normal_x = -normal_dir(0);
                vp_fov_normal.normal_y = -normal_dir(1);
                vp_fov_normal.normal_z = z_new;

                seg_vps->points.push_back(vp_fov_normal);
                PR.pt_vp_pairs[pt_vec] = vp_vec;
                PR.vp_pt_pairs[vp_vec] = pt_vec;
                p1(0) = pt_.x;
                p1(1) = pt_.y;
                p1(2) = pt_.z;
                p2(0) = vp_vec(0);
                p2(1) = vp_vec(1);
                p2(2) = vp_vec(2);
                avg_dist += (p1 - p2).norm();
                dist_count++;
                PR.vp_seg_pairs[p2] = seg_id;
              }
            }
          }
          if (zFlag == false)
          {
            vp_vec(0) = vp.x;
            vp_vec(1) = vp.y;
            vp_vec(2) = vp.z;
            basic_vps->points.push_back(vp);
            vp_search_sub_[vp_vec] = sub_id;

            vp_fov_normal.x = vp.x;
            vp_fov_normal.y = vp.y;
            vp_fov_normal.z = vp.z;
            vp_fov_normal.normal_x = -normal_dir(0);
            vp_fov_normal.normal_y = -normal_dir(1);
            vp_fov_normal.normal_z = -normal_dir(2);

            seg_vps->points.push_back(vp_fov_normal);
            PR.pt_vp_pairs[pt_vec] = vp_vec;
            PR.vp_pt_pairs[vp_vec] = pt_vec;
            p1(0) = pt_.x;
            p1(1) = pt_.y;
            p1(2) = pt_.z;
            p2(0) = vp.x;
            p2(1) = vp.y;
            p2(2) = vp.z;
            avg_dist += (p1 - p2).norm();
            dist_count++;
            PR.vp_seg_pairs[p2] = seg_id;
          }
        }
      }

      /* safety check */
      pcl::PointXYZ safe_vp;
      Eigen::Vector3d filter_vp_, filter_dir_;
      for (auto vp : seg_vps->points)
      {
        bool safe_flag = true;
        for (int i = 0; vp.x - safe_radius_vp + i * HCMap->hcmp_->resolution_ < vp.x + safe_radius_vp; i = i + 2)
          for (int j = 0; vp.y - safe_radius_vp + j * HCMap->hcmp_->resolution_ < vp.y + safe_radius_vp; j = j + 2)
            for (int k = 0; vp.z - safe_radius_vp + k * HCMap->hcmp_->resolution_ < vp.z + safe_radius_vp; k = k + 2)
            {
              safe_check_pt(0) = vp.x - safe_radius_vp + i * HCMap->hcmp_->resolution_;
              safe_check_pt(1) = vp.y - safe_radius_vp + j * HCMap->hcmp_->resolution_;
              safe_check_pt(2) = vp.z - safe_radius_vp + k * HCMap->hcmp_->resolution_;
              safe_flag = HCMap->safety_check(safe_check_pt);
              if (safe_flag == false)
                break;
            }
        if (safe_flag == true)
        {
          filter_vp_(0) = vp.x;
          filter_vp_(1) = vp.y;
          filter_vp_(2) = vp.z;
          filter_dir_(0) = vp.normal_x;
          filter_dir_(1) = vp.normal_y;
          filter_dir_(2) = vp.normal_z;
          // if (filter_dir_.norm() < 1e-3)
          //   cout << "Error: vp normal direction is zero!" << endl;
          
          PR.vp_direction_pairs[filter_vp_] = filter_dir_;
          safe_vp.x = vp.x;
          safe_vp.y = vp.y;
          safe_vp.z = vp.z;
          safe_vps_in_seg->points.push_back(safe_vp);
          all_safe_normal_vps->points.push_back(vp);
          origin_sub_ids.push_back(sub_id);
        }
      }

      /* determine direction */
      Eigen::MatrixXd pose_set;
      Eigen::VectorXd pose;
      Eigen::Vector3d sampled_vp;

      pose_set.resize(safe_vps_in_seg->points.size(), 5);

      PR.sub_vps_inflate[sub_id] = safe_vps_in_seg;
      PR.sub_vps_pose[sub_id] = pose_set;
      sub_id++;
      vp_count += seg_vps->points.size();
      safe_vp_count += safe_vps_in_seg->points.size();
    }

    vector<SingleViewpoint> updated_vps;
    viewpoint_manager_->reset();
    viewpoint_manager_->setMapPointCloud(PR.occ_model);
    viewpoint_manager_->setModel(PR.model);
    viewpoint_manager_->setNormals(PR.pt_normal_pairs);
    viewpoint_manager_->setInitViewpoints(all_safe_normal_vps);
    viewpoint_manager_->updateViewpoints();
    viewpoint_manager_->getUpdatedViewpoints(updated_vps);
    viewpoint_manager_->getUncoveredModelCloud(uncovered_area);

    pcl::KdTreeFLANN<pcl::PointXYZ> basic_vps_tree_;
    basic_vps_tree_.setInputCloud(basic_vps);
    PR.vps_set_.clear();
    for (auto updated_vp : updated_vps)
    {
      Viewpoint final_vp;
      final_vp.vp_id = updated_vp.vp_id;
      final_vp.pose = updated_vp.pose;
      final_vp.vox_count = updated_vp.vox_count;
      if (updated_vp.vp_id < (int)all_safe_normal_vps->points.size())
      {
        final_vp.sub_id = origin_sub_ids[updated_vp.vp_id];
      }
      else
      {
        pcl::PointXYZ pt;
        pt.x = updated_vp.pose(0);
        pt.y = updated_vp.pose(1);
        pt.z = updated_vp.pose(2);
        std::vector<int> nearest;
        std::vector<float> k_sqr_distances;
        basic_vps_tree_.nearestKSearch(pt, 1, nearest, k_sqr_distances);
        Eigen::Vector3d NearestVec;
        NearestVec(0) = basic_vps->points[nearest[0]].x;
        NearestVec(1) = basic_vps->points[nearest[0]].y;
        NearestVec(2) = basic_vps->points[nearest[0]].z;
        if (vp_search_sub_.find(NearestVec) != vp_search_sub_.end())
          final_vp.sub_id = vp_search_sub_.find(NearestVec)->second;
      }
      PR.vps_set_.push_back(final_vp);
    }
  }

  // ! ------------------------------------- Utils -------------------------------------
  /* uniform path interpolation */
  void hierarchical_coverage_planner::interFullPath(double dist_bound)
  {
    vector<Eigen::VectorXd> temp_full_path;
    vector<int> waypt_ids;

    temp_full_path.push_back(PR.FullPath_[0]);
    int temp_id = 1;
    for (int i = 1; i < (int)PR.FullPath_.size(); ++i)
    {
      Eigen::Vector3d lastPos = PR.FullPath_[i - 1].head(3);
      Eigen::Vector3d currPos = PR.FullPath_[i].head(3);
      double dist = (lastPos - currPos).norm();
      if (dist > dist_bound)
      {
        int piece = ceil(dist / dist_bound);
        double inter_dist = dist / (double)piece;
        vector<Eigen::Vector3d> inter_waypts;
        vector<Eigen::VectorXd> updated_waypts;
        for (int j = 0; j < piece + 1; ++j)
        {
          Eigen::Vector3d interPos = lastPos + (double)j * inter_dist * (currPos - lastPos).normalized();
          inter_waypts.push_back(interPos);
        }
        solver_->AngleInterpolation(PR.FullPath_[i - 1], PR.FullPath_[i], inter_waypts, updated_waypts);

        for (auto x : updated_waypts)
        {
          Eigen::Vector3d updatedPos = x.head(3);
          if ((updatedPos - lastPos).norm() > 1e-3 && (updatedPos - currPos).norm() > 1e-3)
          {
            temp_full_path.push_back(x);
            waypt_ids.push_back(temp_id);
            temp_id++;
          }
        }
      }

      temp_full_path.push_back(PR.FullPath_[i]);
      if (PR.waypoints_indicators_[i] == true)
        waypt_ids.push_back(temp_id);
      temp_id++;
    }

    PR.FullPath_.clear();
    PR.FullPath_ = temp_full_path;
    PR.waypoints_indicators_.clear();
    PR.waypoints_indicators_.resize(PR.FullPath_.size(), false);
    for (int i = 0; i < (int)waypt_ids.size(); ++i)
      PR.waypoints_indicators_[waypt_ids[i]] = true;
  }

  /* State: true for outside, false for inside */
  bool hierarchical_coverage_planner::lightStateDet(Eigen::Vector3d &vpCandi, Eigen::Vector3d &vpCandiNormal)
  {
    bool det = false;

    Eigen::Vector3d vpTest = vpCandi + 0.0 * HCMap->hcmp_->resolution_ * vpCandiNormal;

    // find the nearest K rosa vertex
    int K = 1;
    vector<int> nearest_id(K);
    vector<float> nn_squared_distance(K);
    pcl::PointXYZ searchVP;
    searchVP.x = vpCandi(0);
    searchVP.y = vpCandi(1);
    searchVP.z = vpCandi(2);
    ROSATree.nearestKSearch(searchVP, K, nearest_id, nn_squared_distance);

        // convex detection: BiRC
    Eigen::Vector3i idxFwd, idxRev;
    vector<bool> checkFlags;
    bool fwdFlag, revFlag;
    for (int i = 0; i < K; ++i)
    {
      Eigen::Vector3d nearestROSAVertex;
      nearestROSAVertex << rosaCloud->points[nearest_id[i]].x, rosaCloud->points[nearest_id[i]].y, rosaCloud->points[nearest_id[i]].z;

      raycaster_->biInput(vpTest, nearestROSAVertex);
      raycaster_->biNextId(idxFwd, idxRev);
      int crossCount = 0;
      while (raycaster_->biNextId(idxFwd, idxRev))
      {
        fwdFlag = HCMap->occCheck(idxFwd);
        revFlag = HCMap->occCheck(idxRev);
        if (fwdFlag == false)
          crossCount++;
        if (revFlag == false)
          crossCount++;
      }
      fwdFlag = HCMap->occCheck(idxFwd);
      if (fwdFlag == false)
        crossCount++;
      // cross even number: inside, cross odd number: outside
      if (crossCount % 2 == 0)
        checkFlags.push_back(false);
      else
        checkFlags.push_back(true);
    }

    int detNum = 0;
    for (auto x : checkFlags)
    {
      if (x == true)
        detNum++;
    }

    if (detNum == K)
      det = true;

    return det;
  }

  Eigen::Vector3d hierarchical_coverage_planner::pyToVec(Eigen::Vector2d &pitch_yaw)
  {
    Eigen::Vector3d vec;
    vec(0) = cos(pitch_yaw(0)) * cos(pitch_yaw(1));
    vec(1) = cos(pitch_yaw(0)) * sin(pitch_yaw(1));
    vec(2) = sin(pitch_yaw(0));

    return vec;
  }

  void hierarchical_coverage_planner::cutting_plane_normals()
  {
    avg_inner_dist_ = 0.0;
    inner_count_ = 0;
    int seg_id;
    Eigen::Vector3d seg_start, seg_end, proj_pt, seg_dir;
    double seg_length;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;
    for (const auto &seg_cl : skeleton_operator->P.seg_clouds_scale)
    {
      seg_id = seg_cl.first;
      seg_start = skeleton_operator->P.vertices_scale.row(skeleton_operator->P.segments[seg_id][0]);
      seg_end = skeleton_operator->P.vertices_scale.row(skeleton_operator->P.segments[seg_id][1]);
      seg_dir = (seg_end - seg_start).normalized();
      seg_length = (seg_end - seg_start).norm();
      temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      for (int i = 0; i < (int)seg_cl.second->points.size(); i++)
      {
        auto cloud = seg_cl.second->points[i];
        Eigen::Vector3d pt_vec;
        pt_vec(0) = cloud.x;
        pt_vec(1) = cloud.y;
        pt_vec(2) = cloud.z;
        Eigen::Vector3i pt_index;
        HCMap->posToIndex_hc(pt_vec, pt_index);
        if (HCMap->hcmd_->occupancy_buffer_hc_[HCMap->toAddress_hc(pt_index)] == 1)
          temp_cloud->points.push_back(cloud);
      }
      skeleton_operator->P.seg_clouds_scale[seg_id] = temp_cloud;

      seg_visited_buffer_.clear();
      seg_visited_buffer_.resize(temp_cloud->points.size(), false);
      vector<Eigen::Vector3d> proj_points;
      for (int i = 0; normal_step * i < seg_length; ++i)
      {
        proj_pt = seg_start + normal_step * i * seg_dir;
        proj_points.push_back(proj_pt);
      }
      proj_points.push_back(seg_end);

      for (auto proj_pt : proj_points)
        plane_query(temp_cloud, proj_pt, seg_dir, 0.5 * normal_step);

      Eigen::Vector3d remain_normal;
      for (int j = 0; j < (int)seg_visited_buffer_.size(); ++j)
      {
        Eigen::Vector3d remain_pt;
        double dist = 0.0, dist_min = 100000.0;
        int distri_id = -1;
        if (seg_visited_buffer_[j] == false)
        {
          remain_pt << temp_cloud->points[j].x, temp_cloud->points[j].y, temp_cloud->points[j].z;
          for (int k = 0; k < (int)proj_points.size(); ++k)
          {
            dist = (proj_points[k] - remain_pt).norm();
            if (dist < dist_min)
            {
              dist_min = dist;
              distri_id = k;
            }
          }
          remain_normal = (remain_pt - proj_points[distri_id]).normalized();
          avg_inner_dist_ += (remain_pt - proj_points[distri_id]).norm();
          inner_count_++;
          PR.pt_normal_pairs[remain_pt] = remain_normal;
          PR.pt_proj_pairs[remain_pt] = proj_points[distri_id];
        }
      }
    }
    avg_inner_dist_ = avg_inner_dist_ / inner_count_;

    for (const auto &pt_normal : PR.pt_normal_pairs)
    {
      Eigen::VectorXd oPNormal;
      oPNormal.resize(6);
      oPNormal << pt_normal.first(0), pt_normal.first(1), pt_normal.first(2), pt_normal.second(0), pt_normal.second(1), pt_normal.second(2);
      
      PR.outer_normals.push_back(oPNormal);
    }
  }

  void hierarchical_coverage_planner::plane_query(pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, Eigen::Vector3d &point_on_plane, Eigen::Vector3d &plane_normal, double thickness)
  {
    Eigen::Vector3d pt_vec, pt_normal;
    for (int i = 0; i < (int)point_cloud->points.size(); ++i)
    {
      pt_vec << point_cloud->points[i].x, point_cloud->points[i].y, point_cloud->points[i].z;
      double distance_from_plane = (pt_vec - point_on_plane).dot(plane_normal);
      if (std::abs(distance_from_plane) <= thickness)
      {
        seg_visited_buffer_[i] = true;
        pt_normal = (pt_vec - point_on_plane).normalized();
        avg_inner_dist_ += (pt_vec - point_on_plane).norm();
        inner_count_++;
        PR.pt_normal_pairs[pt_vec] = pt_normal;
        PR.pt_proj_pairs[pt_vec] = point_on_plane;
      }
    }
  }

  /* find joint intersection and local path refinement */
  void hierarchical_coverage_planner::JointRefinePath(bool turn)
  {
    // ! Joint Sphere manner
    /* find joints for local refinement */
    vector<int> JointCandidates;
    vector<int> JointId;
    unordered_map<int, int> count_map;
    for (auto sub : skeleton_operator->P.branches)
    {
      for (auto ver_id : sub)
        JointCandidates.push_back(ver_id);
    }
    for (const auto &num : JointCandidates)
      count_map[num]++;
    for (const auto &pair : count_map)
    {
      if (pair.second >= 2)
        JointId.push_back(pair.first);
    }
    for (auto x : JointId)
    {
      Eigen::Vector3d JointPos;
      JointPos << skeleton_operator->P.vertices_scale.row(x)(0), skeleton_operator->P.vertices_scale.row(x)(1), skeleton_operator->P.vertices_scale.row(x)(2);
      PR.connectJoints.push_back(JointPos);
    }
    PR.searchRange = JointCoeff * (safe_inner_coefficient_ * avg_inner_dist_ + dist_vp);

    // * Local Refinement
    /* search refine candidates */
    vector<Eigen::VectorXd> RefinedResults;
    RefinedResults = solver_->LocalRefine(PR.connectJoints, PR.searchRange, PR.FullPath_, turn);

    double oriLength = 0.0;
    for (int j = 1; j < (int)PR.FullPath_.size() - 1; ++j)
      oriLength += (PR.FullPath_[j + 1].head(3) - PR.FullPath_[j].head(3)).norm();
    double refineLength = 0.0;
    for (int i = 1; i < (int)RefinedResults.size() - 1; ++i)
      refineLength += (RefinedResults[i + 1].head(3) - RefinedResults[i].head(3)).norm();
    ROS_INFO("\033[37m[LocalRefine] initial path length = %lf m, refined path length = %lf m.\033[32m", oriLength, refineLength);

    PR.OriFullPath_ = PR.FullPath_;
    PR.FullPath_ = RefinedResults;

    // ? visualize joint spheres
    for (int i = 0; i < (int)solver_->JointVps.size(); ++i)
    {
      vector<Eigen::Vector3d> TempVps;
      for (int j = 0; j < (int)solver_->JointVps[i].size(); ++j)
      {
        Eigen::Vector3d selectedVp = solver_->JointVps[i][j];
        TempVps.push_back(selectedVp);
      }
      PR.innerVps.push_back(TempVps);
    }
  }

  double hierarchical_coverage_planner::CoverageEvaluation(vector<Eigen::VectorXd> &poseSet)
  {
    auto ce_t1 = std::chrono::high_resolution_clock::now();

    double coverageRate = 0.0;
    vector<bool> cover_state;
    cover_state.resize(Fullmodel->points.size(), false);
    int xPixel = 2 * (int)cx, yPixel = 2 * (int)cy;
    double xRange = fx * tan(0.5 * fov_w * M_PI / 180.0) / 1000.0;
    double yRange = fy * tan(0.5 * fov_h * M_PI / 180.0) / 1000.0;
    Eigen::Vector2d ldc;
    ldc << -xRange, -yRange;
    double xRes = 2 * xRange / xPixel, yRes = 2 * yRange / yPixel;

    Eigen::Vector3d pt_vec;
    Eigen::Vector3i idx;
    Eigen::Vector2i camCod;
    double camDist;
    int xID, yID;
    // bool visible_flag = false;
    vector<int> allVisible;
    for (auto vp : poseSet)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr visibleCloud(new pcl::PointCloud<pcl::PointXYZ>);
      vector<int> visible_id_vp;
      Eigen::MatrixXd ImageStateTable(xPixel, yPixel);
      ImageStateTable.setZero();
      Eigen::MatrixXi ImagePointTable(xPixel, yPixel);
      Eigen::MatrixXd ImageDistTable(xPixel, yPixel);
      ImageDistTable.setOnes();
      ImageDistTable = 1000.0 * ImageDistTable;
      Eigen::Vector3d position = vp.head(3);
      percep_utils_->setPose_PY(position, vp(3), vp(4));
      for (int i = 0; i < (int)Fullmodel->points.size(); ++i)
      {
        pt_vec(0) = Fullmodel->points[i].x;
        pt_vec(1) = Fullmodel->points[i].y;
        pt_vec(2) = Fullmodel->points[i].z;
        if (percep_utils_->insideFOV(pt_vec) == true)
        {
          // ! /* Pinhole Camera Model Evaluation */
          CameraModelProjection(vp, pt_vec, xRes, yRes, ldc, camDist, camCod);
          xID = camCod(0);
          yID = camCod(1);
          if (xID < 0 || yID < 0)
            continue;
          if (ImageStateTable(xID, yID) == 0.0)
          {
            ImageStateTable(xID, yID) = 1.0;
            ImagePointTable(xID, yID) = i;
            ImageDistTable(xID, yID) = camDist;
          }
          else
          {
            if (camDist < ImageDistTable(xID, yID))
            {
              cover_state[ImagePointTable(xID, yID)] = false;
              ImagePointTable(xID, yID) = i;
              ImageDistTable(xID, yID) = camDist;
            }
          }
        }
      }
      for (int r = 0; r < xPixel; ++r)
        for (int c = 0; c < yPixel; ++c)
        {
          if (ImageStateTable(r, c) != 0.0)
            visible_id_vp.push_back(ImagePointTable(r, c));
        }

      set<int> visSet(visible_id_vp.begin(), visible_id_vp.end());
      for (const auto &visid : visSet)
      {
        if (cover_state[visid] == false)
          visibleCloud->points.push_back(Fullmodel->points[visid]);
      }
      visible_group.push_back(visibleCloud);
      for (const auto &upid : visSet)
        cover_state[upid] = true;

      allVisible.insert(allVisible.end(), visible_id_vp.begin(), visible_id_vp.end());
    }
    set<int> uniqueSet(allVisible.begin(), allVisible.end());

    int truenum = 0;
    truenum = (int)uniqueSet.size();
    coverageRate = (double)truenum / (double)cover_state.size();

    auto ce_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ce_ms = ce_t2 - ce_t1;
    double ce_time = (double)ce_ms.count();
    ROS_INFO("\033[36m[CoverageAnalyzer] FC-Planner trajectory coverage evaluation time = %lf s.\033[32m", ce_time / 1000.0);

    return coverageRate;
  }

  void hierarchical_coverage_planner::PinHoleCamera(Eigen::VectorXd &vp, vector<int> &covered_id,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &checkCloud)
  {
    int xPixel = 2 * (int)cx, yPixel = 2 * (int)cy;
    double xRange = fx * tan(0.5 * fov_w * M_PI / 180.0) / 1000.0;
    double yRange = fy * tan(0.5 * fov_h * M_PI / 180.0) / 1000.0;
    Eigen::Vector2d ldc;
    ldc << -xRange, -yRange;
    double xRes = 2 * xRange / xPixel, yRes = 2 * yRange / yPixel;

    Eigen::Vector3d pt_vec;
    Eigen::Vector3i idx;
    Eigen::Vector2i camCod;
    double camDist;
    int xID, yID;

    vector<int> visible_id_vp;

    Eigen::MatrixXd ImageStateTable(xPixel, yPixel);
    ImageStateTable.setZero();
    Eigen::MatrixXi ImagePointTable(xPixel, yPixel);
    Eigen::MatrixXd ImageDistTable(xPixel, yPixel);
    ImageDistTable.setOnes();
    ImageDistTable = 1000.0 * ImageDistTable;

    Eigen::Vector3d position = vp.head(3);
    percep_utils_->setPose_PY(position, vp(3), vp(4));
    for (int i = 0; i < (int)checkCloud->points.size(); ++i)
    {
      pt_vec(0) = checkCloud->points[i].x;
      pt_vec(1) = checkCloud->points[i].y;
      pt_vec(2) = checkCloud->points[i].z;
      if (percep_utils_->insideFOV(pt_vec) == true)
      {
        CameraModelProjection(vp, pt_vec, xRes, yRes, ldc, camDist, camCod);
        xID = camCod(0);
        yID = camCod(1);
        if (xID < 0 || yID < 0)
          continue;
        if (ImageStateTable(xID, yID) == 0.0)
        {
          ImageStateTable(xID, yID) = 1.0;
          ImagePointTable(xID, yID) = i;
          ImageDistTable(xID, yID) = camDist;
        }
        else
        {
          if (camDist < ImageDistTable(xID, yID))
          {
            ImagePointTable(xID, yID) = i;
            ImageDistTable(xID, yID) = camDist;
          }
        }
      }
    }

    for (int r = 0; r < xPixel; ++r)
      for (int c = 0; c < yPixel; ++c)
      {
        if (ImageStateTable(r, c) != 0.0)
          visible_id_vp.push_back(ImagePointTable(r, c));
      }

    set<int> visSet(visible_id_vp.begin(), visible_id_vp.end());
    for (const auto &visid : visSet)
      covered_id.push_back(visid);
  }

  void hierarchical_coverage_planner::CameraModelProjection(Eigen::VectorXd &pose, Eigen::Vector3d &point, double &xRes, double &yRes, Eigen::Vector2d &leftdown, double &distance, Eigen::Vector2i &inCam)
  {
    Eigen::Vector3d pos_ = pose.head(3);
    distance = (point - pos_).norm();
    double pitch = pose(3), yaw = pose(4);
    Eigen::Matrix3d R_y, R_p;
    R_y << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;
    R_p << cos(pitch), 0.0, -sin(pitch), 0.0, 1.0, 0.0, sin(pitch), 0.0, cos(pitch);
    Eigen::Vector3d pointCam = R_p.inverse() * R_y.inverse() * (point - pos_);

    double camX = fx * pointCam(1) / (1000.0 * pointCam(0)) - leftdown(0);
    double camY = fy * pointCam(2) / (1000.0 * pointCam(0)) - leftdown(1);

    inCam(0) = floor(camX / xRes);
    inCam(1) = floor(camY / yRes);
  }
} // namespace predrecon