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
#ifndef _VIEWPOINT_MANAGER_NODE_H_
#define _VIEWPOINT_MANAGER_NODE_H_

#include <ros/ros.h>
#include <viewpoint_manager/viewpoint_manager.h>
#include <viewpoint_manager/backward.hpp>

namespace backward
{
  backward::SignalHandling sh;
}
using namespace std;
using namespace predrecon;

// param
const string frame_id_ = "world";
string model_path_, free_cloud_path_;
double downsampled_size_, downsampled_size_for_viewpoint_;
double dist_vp_;
double min_box_z_,max_box_z_;

pcl::PointCloud<pcl::PointXYZ>::Ptr model_;
pcl::PointCloud<pcl::PointXYZ>::Ptr free_model_;
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_model_;
pcl::PointCloud<pcl::PointXYZ>::Ptr vis_cloud_;
pcl::PointCloud<pcl::Normal>::Ptr downsampled_normals_;

// utils
shared_ptr<ViewpointManager> viewpoint_manager_;
shared_ptr<SDFMap> HCMap_;
shared_ptr<RayCaster> raycast_;
shared_ptr<PerceptionUtils> percep_utils_;

ros::Publisher occ_pub_, free_pub_, model_normals_pub_, updated_vps_pub_, updated_vps_fov_pub_;
ros::Publisher origin_vps_pub_, origin_vps_normals_pub_;

void inputModel();
void initUtils(ros::NodeHandle nh);
void modelProcessing();

bool viewpointSafetyCheck(Vector3d vp);
bool viewpointSafetyCheck(pcl::PointXYZ vp);
bool viewpointUsefulCheck(Vector3d pt, Vector3d vp);

void publishCloudMap(ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
void publishNormals(ros::Publisher &pub,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                    const pcl::PointCloud<pcl::Normal>::Ptr &normals);
void publishViewpoints(ros::Publisher &pub,
                       const pcl::PointCloud<pcl::PointNormal>::Ptr &viewpoints,
                       const double &scale, const Eigen::Vector4d &color);
void publishFOV(ros::Publisher &pub,
                const vector<vector<Eigen::Vector3d>> &list1, const vector<vector<Eigen::Vector3d>> &list2);
#endif