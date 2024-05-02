/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
                     Mingjie Zhang <zagerzhang at gmail dot com>, STAR Group, SYSU.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of ViewpointManager class, which
 *                   implements iterative updates of viewpoint pose in FC-Planner.
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
#ifndef _VIEWPOINT_MANAGER_H_
#define _VIEWPOINT_MANAGER_H_

#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <unordered_map>
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

using namespace std;

class RayCaster;
namespace predrecon
{
  class SDFMap;
  class PerceptionUtils;

  struct Vector3dCompare0
  {
    bool operator()(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2) const
    {
      if (v1(0) != v2(0))
        return v1(0) < v2(0);
      if (v1(1) != v2(1))
        return v1(1) < v2(1);
      return v1(2) < v2(2);
    }
  };

  struct VectorHashEigenVector3d
  {
    std::size_t operator()(const Eigen::Vector3d &vec) const
    {
      std::size_t seed = 0;
      for (int i = 0; i < 3; ++i)
      {
        std::hash<double> hasher;
        seed ^= hasher(vec(i)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      return seed;
    }
  };

  struct SingleViewpoint
  {
    int vp_id;
    Eigen::VectorXd pose;
    int vox_count;
  };

  class ViewpointManager
  {
    struct TaskState
    {
      bool map_flag_;
      bool model_flag_;
      bool viewpoints_flag_;
      bool normal_flag_;

      void reset()
      {
        map_flag_ = false;
        model_flag_ = false;
        viewpoints_flag_ = false;
        normal_flag_ = false;
      }
      bool isReady()
      {
        if (!map_flag_)
        {
          ROS_ERROR("[ViewpointManager] Not ready: No input map!!");
          return false;
        }
        else if (!model_flag_)
        {
          ROS_ERROR("[ViewpointManager] Not ready: No input model!!");
          return false;
        }
        else if (!viewpoints_flag_)
        {
          ROS_ERROR("[ViewpointManager] Not ready: No input viewpoints!!");
          return false;
        }
        else if (!normal_flag_)
        {
          ROS_ERROR("[ViewpointManager] Not ready: No input normals!!");
          return false;
        }
        return true;
      }
    };

    struct ModelCloudInfo
    {
      vector<bool> cover_state;
      vector<int> cover_contrib;
      vector<int> contrib_id;
      map<Eigen::Vector3d, Eigen::Vector3d, Vector3dCompare0> pt_normal_pairs;

      void reset()
      {
        cover_state.clear();
        cover_contrib.clear();
        contrib_id.clear();
        pt_normal_pairs.clear();
      }
    };

    struct ViewpointsInfo
    {
      Eigen::MatrixXd vps_pose;
      vector<int> vps_voxcount; // contain the number of voxels covered by each viewpoint
      vector<int> vps_contri;
      int last_vps_num;

      void reset()
      {
        vps_pose.resize(0, 0);
        vps_voxcount.clear();
        vps_contri.clear();
        last_vps_num = 0;
      }
    };

    struct ViewpointsPrune
    {
      unordered_map<Eigen::Vector3d, int, VectorHashEigenVector3d> inverse_idx_;
      unordered_map<int, Eigen::VectorXd> idx_viewpoints_; // [vp_id]: viewpoint pose
      unordered_map<int, int> idx_ctrl_voxels_;            // [vp_id]: viewpoint voxel num
      unordered_map<int, bool> idx_live_state_;            // [vp_id]: if use this viewpoint finally?
      unordered_map<int, bool> idx_query_state_;           // [vp_id]: the query state of this viewpoint
      vector<SingleViewpoint> final_vps_;

      void reset()
      {
        inverse_idx_.clear();
        idx_viewpoints_.clear();
        idx_ctrl_voxels_.clear();
        idx_live_state_.clear();
        idx_query_state_.clear();
        final_vps_.clear();
      }
    };

  public:
    ViewpointManager();
    ~ViewpointManager();

    void init(ros::NodeHandle &nh);
    void reset();
    void setModel(pcl::PointCloud<pcl::PointXYZ>::Ptr input_model);
    void setMapPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_map);
    void setNormals(map<Eigen::Vector3d, Eigen::Vector3d, Vector3dCompare0> &pt_normal_pairs);
    void setInitViewpoints(pcl::PointCloud<pcl::PointNormal>::Ptr input_normal_vps);
    void updateViewpoints();

    void getUpdatedViewpoints(vector<SingleViewpoint> &viewpoints);
    void getUncoveredModelCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);

  private:
    Eigen::VectorXd getPose(int vp_id, const Vector3d &pos, const Vector3d &dir);
    void viewpointsPrune(Eigen::MatrixXd sub_pose, vector<int> sub_voxcount);
    void updatePoseGravitation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, vector<int> &inner_ids, const int &cur_idx);
    void updateViewpointInfo(Eigen::VectorXd &pose, const int &vp_id);

    bool viewpointSafetyCheck(pcl::PointXYZ &vp);
    bool nearCheck(Eigen::Vector3d &cur_position, int &vox_num);
    vector<pcl::PointNormal> uncNeighVps(pcl::PointXYZ oript, pcl::Normal normal);

    inline void warpAngle(double &angle);
    inline Eigen::Vector2d PitchYaw(Eigen::Vector3d &vec);
    inline Eigen::Vector3d pyToVec(Eigen::Vector2d &pitch_yaw);

    /* Param */
    ros::NodeHandle nh_;
    double visible_range;
    double pitch_upper, pitch_lower;
    double fov_h, fov_w;
    bool zFlag;
    double GroundZPos, safeHeight;
    double safe_radius_vp;
    bool pose_update;
    double dist_vp, fov_base;
    string attitude;
    double voxel_size;
    int max_iter_num;

    /* Utils */
    shared_ptr<SDFMap> HCMap;
    unique_ptr<RayCaster> raycaster_, raycaster_rev;
    unique_ptr<PerceptionUtils> percep_utils_;

    /* Data */
    pcl::KdTreeFLANN<pcl::PointXYZ> map_cloud_kdtree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_;
    ModelCloudInfo MCI;
    ViewpointsInfo VPI;
    ViewpointsPrune VPP;
    TaskState TS;
  };

  /* parse direction vector to Euler angle */
  inline Eigen::Vector2d ViewpointManager::PitchYaw(Eigen::Vector3d &vec)
  {
    Eigen::Vector2d PY;
    double pitch = std::asin(vec.z() / (vec.norm() + 1e-3)); // calculate pitch angle
    double yaw = std::atan2(vec.y(), vec.x());               // calculate yaw angle

    PY(0) = pitch;
    PY(1) = yaw;

    return PY;
  }

  /* convert Euler angle to direction vector */
  inline Eigen::Vector3d ViewpointManager::pyToVec(Eigen::Vector2d &pitch_yaw)
  {
    Eigen::Vector3d vec;
    vec(0) = cos(pitch_yaw(0)) * cos(pitch_yaw(1));
    vec(1) = cos(pitch_yaw(0)) * sin(pitch_yaw(1));
    vec(2) = sin(pitch_yaw(0));

    return vec;
  }

  inline void ViewpointManager::warpAngle(double &angle)
  {
    while (angle < -M_PI)
      angle += 2 * M_PI;
    while (angle > M_PI)
      angle -= 2 * M_PI;
  }
}

#endif