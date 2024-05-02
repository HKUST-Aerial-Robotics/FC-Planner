/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main file of volumetric mapping module, which
 *                   is mainly modified from Fast-Planner (https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
 *                   and PredRecon (https://github.com/HKUST-Aerial-Robotics/PredRecon).
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

#include <plan_env/sdf_map.h>

namespace predrecon {
SDFMap::SDFMap() {
}

SDFMap::~SDFMap() {
}

void SDFMap::initMap(ros::NodeHandle& nh) {
  mp_.reset(new MapParam);
  md_.reset(new MapData);
  mr_.reset(new MapROS);

  // Params of map properties
  double x_size, y_size, z_size;
  nh.param("sdf_map/resolution", mp_->resolution_, -1.0);
  nh.param("sdf_map/map_size_x", x_size, -1.0);
  nh.param("sdf_map/map_size_y", y_size, -1.0);
  nh.param("sdf_map/map_size_z", z_size, -1.0);
  nh.param("sdf_map/obstacles_inflation", mp_->obstacles_inflation_, -1.0);
  nh.param("sdf_map/local_bound_inflate", mp_->local_bound_inflate_, 1.0);
  nh.param("sdf_map/local_map_margin", mp_->local_map_margin_, 1);
  nh.param("sdf_map/ground_height", mp_->ground_height_, 1.0);
  nh.param("sdf_map/default_dist", mp_->default_dist_, 5.0);
  nh.param("sdf_map/optimistic", mp_->optimistic_, true);
  nh.param("sdf_map/signed_dist", mp_->signed_dist_, false);

  mp_->local_bound_inflate_ = max(mp_->resolution_, mp_->local_bound_inflate_);
  mp_->resolution_inv_ = 1 / mp_->resolution_;
  // mp_->map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_->ground_height_);
  mp_->map_origin_ = Eigen::Vector3d(-1.0, -y_size / 2.0, mp_->ground_height_);
  mp_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
  for (int i = 0; i < 3; ++i)
    mp_->map_voxel_num_(i) = ceil(mp_->map_size_(i) / mp_->resolution_);
  mp_->map_min_boundary_ = mp_->map_origin_;
  mp_->map_max_boundary_ = mp_->map_origin_ + mp_->map_size_;
  min_bound = mp_->map_min_boundary_;
  max_bound = mp_->map_max_boundary_;

  // Params of raycasting-based fusion
  nh.param("sdf_map/p_hit", mp_->p_hit_, 0.70);
  nh.param("sdf_map/p_miss", mp_->p_miss_, 0.35);
  nh.param("sdf_map/p_min", mp_->p_min_, 0.12);
  nh.param("sdf_map/p_max", mp_->p_max_, 0.97);
  nh.param("sdf_map/p_occ", mp_->p_occ_, 0.80);
  nh.param("sdf_map/max_ray_length", mp_->max_ray_length_, -0.1);
  nh.param("sdf_map/prediction_max_ray_length", mp_->max_ray_length_prediction, -0.1);
  nh.param("sdf_map/virtual_ceil_height", mp_->virtual_ceil_height_, -0.1);
  nh.param("sdf_map/view_pos_threshold", mp_->view_thre_, -0.1);

  auto logit = [](const double& x) { return log(x / (1 - x)); };
  mp_->prob_hit_log_ = logit(mp_->p_hit_);
  mp_->prob_miss_log_ = logit(mp_->p_miss_);
  mp_->clamp_min_log_ = logit(mp_->p_min_);
  mp_->clamp_max_log_ = logit(mp_->p_max_);
  mp_->min_occupancy_log_ = logit(mp_->p_occ_);
  mp_->unknown_flag_ = 0.01;
  cout << "hit: " << mp_->prob_hit_log_ << ", miss: " << mp_->prob_miss_log_
       << ", min: " << mp_->clamp_min_log_ << ", max: " << mp_->clamp_max_log_
       << ", thresh: " << mp_->min_occupancy_log_ << endl;

  // Initialize data buffer of map
  int buffer_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
  md_->occupancy_buffer_ = vector<double>(buffer_size, mp_->clamp_min_log_ - mp_->unknown_flag_);
  md_->occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_->occupancy_buffer_pred_ = vector<char>(buffer_size, 0);
  // recon_poses_idx_ = vector<int>(buffer_size, 0);
  // recon_states_ = vector<int>(buffer_size, 0);
  md_->distance_buffer_neg_ = vector<double>(buffer_size, mp_->default_dist_);
  md_->distance_buffer_ = vector<double>(buffer_size, mp_->default_dist_);
  md_->count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_->count_hit_ = vector<short>(buffer_size, 0);
  md_->count_miss_ = vector<short>(buffer_size, 0);
  md_->flag_rayend_ = vector<char>(buffer_size, -1);
  md_->flag_visited_ = vector<char>(buffer_size, -1);
  md_->tmp_buffer1_ = vector<double>(buffer_size, 0);
  md_->tmp_buffer2_ = vector<double>(buffer_size, 0);
  md_->raycast_num_ = 0;
  md_->reset_updated_box_ = true;
  md_->update_min_ = md_->update_max_ = Eigen::Vector3d(0, 0, 0);

  // Try retriving bounding box of map, set box to map size if not specified
  vector<string> axis = { "x", "y", "z" };
  for (int i = 0; i < 3; ++i) {
    nh.param("sdf_map/box_min_" + axis[i], mp_->box_mind_[i], mp_->map_min_boundary_[i]);
    nh.param("sdf_map/box_max_" + axis[i], mp_->box_maxd_[i], mp_->map_max_boundary_[i]);
  }
  posToIndex(mp_->box_mind_, mp_->box_min_);
  posToIndex(mp_->box_maxd_, mp_->box_max_);

  // Initialize ROS wrapper
  mr_->setMap(this);
  mr_->node_ = nh;
  mr_->init();

  caster_.reset(new RayCaster);
  caster_->setParams(mp_->resolution_, mp_->map_origin_);
}

void SDFMap::initHCMap(ros::NodeHandle& nh, pcl::PointCloud<pcl::PointXYZ>::Ptr& model)
{
  hcmp_.reset(new HCMapParam);
  hcmd_.reset(new HCMapData);
  mr_.reset(new MapROS);

  nh.param("hcmap/resolution", hcmp_->resolution_, -1.0);
  nh.param("hcmap/interval", hcmp_->proj_interval, -1.0);
  nh.param("hcmap/plane_thickness", hcmp_->thickness, -1.0);
  nh.param("hcmap/checkScale", hcmp_->checkScale, -1.0);
  nh.param("hcmap/checkSize", checkSize, -1);
  nh.param("hcmap/inflateVoxel", inflate_num, -1);
  nh.param("viewpoint_manager/visible_range", hcmp_->size_inflate, -1.0);
  nh.param("viewpoint_manager/zGround", zFlag, false);
  nh.param("viewpoint_manager/GroundPos", zPos, -1.0);

  occ_pub = nh.advertise<sensor_msgs::PointCloud2>("/hcmap/occ_map",10);
  free_pub = nh.advertise<sensor_msgs::PointCloud2>("/hcmap/free_map",10);
  
  /* find scene size */
  pcl::PointXYZ pt_min;
  pcl::PointXYZ pt_max;
  pcl::getMinMax3D(*model,pt_min,pt_max);
  // ROS_INFO("Map Minimum point: x=%f, y=%f, z=%f", pt_min.x, pt_min.y, pt_min.z);
  // ROS_INFO("Map Maximum point: x=%f, y=%f, z=%f", pt_max.x, pt_max.y, pt_max.z);
  double x_size, y_size, z_size;
  x_size = 4*hcmp_->size_inflate+pt_max.x - pt_min.x;
  y_size = 4*hcmp_->size_inflate+pt_max.y - pt_min.y;
  z_size = 4*hcmp_->size_inflate+pt_max.z - pt_min.z;

  hcmp_->resolution_inv_ = 1 / hcmp_->resolution_;
  hcmp_->map_origin_ = Eigen::Vector3d(pt_min.x-2*hcmp_->size_inflate, pt_min.y-2*hcmp_->size_inflate, pt_min.z-2*hcmp_->size_inflate);
  posToIndex_hc(hcmp_->map_origin_, hcmp_->map_origin_idx_);
  hcmp_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
  for (int i = 0; i < 3; ++i)
    hcmp_->map_voxel_num_(i) = ceil(hcmp_->map_size_(i) / hcmp_->resolution_);
  hcmp_->map_min_boundary_ = hcmp_->map_origin_;
  hcmp_->map_max_boundary_ = hcmp_->map_origin_ + hcmp_->map_size_;
  // if (zFlag == true)
  // {
  //   hcmp_->map_min_boundary_(2) = zPos;
  // }
  hcmp_->box_mind_ = hcmp_->map_min_boundary_;
  hcmp_->box_maxd_ = hcmp_->map_max_boundary_;

  posToIndex_hc(hcmp_->box_mind_, hcmp_->box_min_);
  posToIndex_hc(hcmp_->box_maxd_, hcmp_->box_max_);

  int buffer_size = hcmp_->map_voxel_num_(0) * hcmp_->map_voxel_num_(1) * hcmp_->map_voxel_num_(2);
  hcmd_->occupancy_buffer_hc_ = vector<char>(buffer_size, 0);
  hcmd_->occupancy_inflate_buffer_hc_ = vector<char>(buffer_size, 0);
  hcmd_->occupancy_buffer_internal_ = vector<char>(buffer_size, 0);

  internal_cast_.reset(new RayCaster);
  internal_cast_->setParams(hcmp_->resolution_, hcmp_->map_origin_);

  for (auto occ:model->points)
  {
    Eigen::Vector3d pt_occ;
    Eigen::Vector3i id_occ;
    int adr_occ;
    pt_occ << occ.x, occ.y, occ.z;
    posToIndex_hc(pt_occ, id_occ);
    adr_occ = toAddress_hc(id_occ);
    hcmd_->occupancy_buffer_hc_[adr_occ] = 1;
  }
}

void SDFMap::inputFreePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points)
{
  for (auto free:points->points)
  {
    Eigen::Vector3d pt_free;
    Eigen::Vector3i id_free;
    int adr_free;
    pt_free << free.x, free.y, free.z;
    posToIndex_hc(pt_free, id_free);
    adr_free = toAddress_hc(id_free);
    if(hcmd_->occupancy_buffer_hc_[adr_free] == 0)
      hcmd_->occupancy_buffer_hc_[adr_free] = 2;
  }
}

void SDFMap::publishMap()
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    Eigen::Vector3i min_idx, max_idx;
    posToIndex_hc(hcmp_->box_mind_, min_idx);
    posToIndex_hc(hcmp_->box_maxd_, max_idx);

    for (int x = min_idx[0]; x <= max_idx[0]; ++x)
      for (int y = min_idx[1]; y <= max_idx[1]; ++y)
        for (int z = min_idx[2]; z <= max_idx[2]; ++z)
        {
          if (hcmd_->occupancy_buffer_hc_[toAddress_hc(x, y, z)] == 1)
          {
            Eigen::Vector3d pos;
            indexToPos_hc(Eigen::Vector3i(x, y, z), pos);
            // if (pos(2) > visualization_truncate_height_)
            //   continue;
            // if (pos(2) < visualization_truncate_low_)
            //   continue;

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud1.push_back(pt);
          }

          if (hcmd_->occupancy_buffer_hc_[toAddress_hc(x, y, z)] == 2)
          {
            Eigen::Vector3d pos;
            indexToPos_hc(Eigen::Vector3i(x, y, z), pos);
            // if (pos(2) > visualization_truncate_height_)
            //   continue;
            // if (pos(2) < visualization_truncate_low_)
            //   continue;

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud2.push_back(pt);
          }
        }
    cloud1.width = cloud1.points.size();
    cloud1.height = 1;
    cloud1.is_dense = true;
    cloud1.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud1, cloud_msg);
    occ_pub.publish(cloud_msg);

    cloud2.width = cloud2.points.size();
    cloud2.height = 1;
    cloud2.is_dense = true;
    cloud2.header.frame_id = "world";
    pcl::toROSMsg(cloud2, cloud_msg);
    free_pub.publish(cloud_msg);
    }

void SDFMap::InternalSpace(map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& seg_cloud, Eigen::MatrixXd& vertices, map<int, vector<int>>& segments)
{ 
  Eigen::Vector3d pt_w;
  Eigen::Vector3i idx;
  int vox_adr, seg_id;
  Eigen::Vector3d seg_start, seg_end, proj_pt, seg_dir;
  double seg_length;
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;

  for (const auto& id_pts:seg_cloud)
  {
    seg_id = id_pts.first;
    seg_start = vertices.row(segments[seg_id][0]);
    seg_end = vertices.row(segments[seg_id][1]);
    seg_dir = (seg_end-seg_start).normalized();
    seg_length = (seg_end-seg_start).norm();
    temp_cloud = id_pts.second;
    
    for (auto pt:id_pts.second->points)
    {
      pt_w << pt.x, pt.y, pt.z;
      posToIndex_hc(pt_w, idx);
      vox_adr = toAddress_hc(idx);
      hcmd_->occupancy_buffer_hc_[vox_adr] = 1;
      hcmd_->occupancy_inflate_buffer_hc_[vox_adr] = 1;

      for (int i=-inflate_num; i<inflate_num+1; i=i+1)
        for (int j=-inflate_num; j<inflate_num+1; j=j+1)
          for (int k=-inflate_num; k<inflate_num+1; k=k+1)
          {
            if (i == 0 && j == 0 && k == 0) {
              continue;
            }
            Eigen::Vector3i neighbor;
            neighbor(0)=idx.x() + i; neighbor(1)=idx.y() + j; neighbor(2)=idx.z() + k;
            hcmd_->occupancy_inflate_buffer_hc_[toAddress_hc(neighbor)] = 1;
          }
      
      /* all projection method */
      // for (int i=0; hcmp_->proj_interval*i<seg_length; ++i)
      // {
      //   proj_pt = seg_start + hcmp_->proj_interval*i*seg_dir;
      //   internal_cast_->input(proj_pt, pt_w);
      //   internal_cast_->nextId(idx);
      //   while (internal_cast_->nextId(idx))
      //   {
      //     hcmd_->occupancy_buffer_internal_[toAddress_hc(idx)] = 1;
      //   }
      // }
    }
    
    /* cutting plane method */
    const std::vector<int> offsets {-1, 0, 1};
    hcmd_->seg_occ_visited_buffer_.clear();
    hcmd_->seg_occ_visited_buffer_.resize(temp_cloud->points.size(), false);
    vector<Eigen::Vector3d> proj_points;
    for (int i=0; hcmp_->proj_interval*i<seg_length; ++i)
    {
      proj_pt = seg_start + hcmp_->proj_interval*i*seg_dir;
      proj_points.push_back(proj_pt);
    }
    proj_points.push_back(seg_end);
    
    vector<Eigen::Vector3d> cut_plane;
    for (auto proj_pt:proj_points)
    {
      cut_plane = points_in_plane(temp_cloud, proj_pt, seg_dir, hcmp_->thickness);
      for (auto in_pt:cut_plane)
      {
        internal_cast_->input(proj_pt, in_pt);
        internal_cast_->nextId(idx);
        while (internal_cast_->nextId(idx))
        {
          hcmd_->occupancy_buffer_internal_[toAddress_hc(idx)] = 1;
          
          // Eigen::Vector3i neighbor;
          // for (const auto& x : offsets) 
          // {
          //   for (const auto& y : offsets) 
          //   {
          //     for (const auto& z : offsets) 
          //     {
          //       // Skip the center point (offsets of 0, 0, 0)
          //       if (x == 0 && y == 0 && z == 0) {
          //         continue;
          //       }
          //       // Compute the neighbor coordinates
          //       neighbor(0)=idx.x() + x; neighbor(1)=idx.y() + y; neighbor(2)=idx.z() + z;
          //       hcmd_->occupancy_buffer_internal_[toAddress_hc(neighbor)] = 1;
          //     }
          //   }
          // }

        }
      }
    }
    
    for (int j=0; j<(int)hcmd_->seg_occ_visited_buffer_.size(); ++j)
    {
      Eigen::Vector3d remain_pt;
      double dist = 0.0, dist_min = 100000.0;
      int distri_id = -1;

      if (hcmd_->seg_occ_visited_buffer_[j] == false)
      {
        remain_pt << temp_cloud->points[j].x, temp_cloud->points[j].y, temp_cloud->points[j].z;
        for (int k=0; k<(int)proj_points.size(); ++k)
        {
          dist = (proj_points[k] - remain_pt).norm();
          if (dist < dist_min)
          {
            dist_min = dist;
            distri_id = k;
          }
        }

        internal_cast_->input(proj_points[distri_id], remain_pt);
        internal_cast_->nextId(idx);
        while (internal_cast_->nextId(idx))
        {
          hcmd_->occupancy_buffer_internal_[toAddress_hc(idx)] = 1;

          // Eigen::Vector3i neighbor;
          // for (const auto& x : offsets) 
          // {
          //   for (const auto& y : offsets) 
          //   {
          //     for (const auto& z : offsets) 
          //     {
          //       // Skip the center point (offsets of 0, 0, 0)
          //       if (x == 0 && y == 0 && z == 0) {
          //         continue;
          //       }
          //       // Compute the neighbor coordinates
          //       neighbor(0)=idx.x() + x; neighbor(1)=idx.y() + y; neighbor(2)=idx.z() + z;
          //       hcmd_->occupancy_buffer_internal_[toAddress_hc(neighbor)] = 1;
          //     }
          //   }
          // }

        }
      }
    }

  }
  
  /* visualization results */
  // pcl::PointXYZ pt;
  // for (int x = hcmp_->box_min_(0) /* + 1 */; x < hcmp_->box_max_(0); ++x)
  //   for (int y = hcmp_->box_min_(1) /* + 1 */; y < hcmp_->box_max_(1); ++y)
  //     for (int z = hcmp_->box_min_(2) /* + 1 */; z < hcmp_->box_max_(2); ++z) {
  //       if (hcmd_->occupancy_buffer_hc_[toAddress_hc(x, y, z)] == 1) {
  //         Eigen::Vector3d pos;
  //         indexToPos_hc(Eigen::Vector3i(x, y, z), pos);
  //         pt.x = pos(0);
  //         pt.y = pos(1);
  //         pt.z = pos(2);
  //         hcmd_->occ_cloud.push_back(pt);
  //       }
  //     }
  
  // pcl::PointXYZ ipt;
  // for (int x = hcmp_->box_min_(0) /* + 1 */; x < hcmp_->box_max_(0); ++x)
  //   for (int y = hcmp_->box_min_(1) /* + 1 */; y < hcmp_->box_max_(1); ++y)
  //     for (int z = hcmp_->box_min_(2) /* + 1 */; z < hcmp_->box_max_(2); ++z) {
  //       if (hcmd_->occupancy_buffer_internal_[toAddress_hc(x, y, z)] == 1) {
  //         Eigen::Vector3d pos;
  //         indexToPos_hc(Eigen::Vector3i(x, y, z), pos);
  //         ipt.x = pos(0);
  //         ipt.y = pos(1);
  //         ipt.z = pos(2);
  //         hcmd_->internal_cloud.push_back(ipt);
  //       }
  //     }

}

void SDFMap::SetOcc(pcl::PointCloud<pcl::PointXYZ>::Ptr& OccModel)
{
  for (auto pt:OccModel->points)
  {
    Eigen::Vector3d pt_w;
    pt_w << pt.x, pt.y, pt.z;
    if (isInMap_hc(pt_w))
    {
      Eigen::Vector3i idx;
      int vox_adr;
      posToIndex_hc(pt_w, idx);
      vox_adr = toAddress_hc(idx);
      hcmd_->occupancy_buffer_hc_[vox_adr] = 1;
    }
  }
}

void SDFMap::OuterCheck(vector<Eigen::VectorXd>& outers)
{
  Eigen::Vector3d start, end;
  Eigen::Vector3i idx;
  for (auto o:outers)
  {
    start = o.head(3);
    end = o.head(3) + hcmp_->checkScale*o.tail(3);
    internal_cast_->input(start, end);
    internal_cast_->nextId(idx);
    while (internal_cast_->nextId(idx))
    {
      if (hcmd_->occupancy_buffer_internal_[toAddress_hc(idx)] == 1)
        hcmd_->occupancy_buffer_internal_[toAddress_hc(idx)] = 0;
    }
  }
}

vector<Eigen::Vector3d> SDFMap::points_in_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, Eigen::Vector3d& point_on_plane, Eigen::Vector3d& plane_normal, double thickness)
{
  vector<Eigen::Vector3d> points;
  Eigen::Vector3d pt_vec;
  for (int i=0; i<(int)point_cloud->points.size(); ++i)
  {
    pt_vec << point_cloud->points[i].x, point_cloud->points[i].y, point_cloud->points[i].z;
    double distance_from_plane = (pt_vec - point_on_plane).dot(plane_normal);
    if (std::abs(distance_from_plane) <= thickness)
    {
      points.push_back(pt_vec);
      hcmd_->seg_occ_visited_buffer_[i] = true;
    }
  }

  return points;
}

void SDFMap::resetBuffer() {
  resetBuffer(mp_->map_min_boundary_, mp_->map_max_boundary_);
  md_->local_bound_min_ = Eigen::Vector3i::Zero();
  md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::reset_PredStates()
{
  int size = md_->occupancy_buffer_pred_.size();
  // md_->occupancy_buffer_pred_ = vector<char>(size, 0);
  md_->occupancy_buffer_pred_.clear();
  md_->occupancy_buffer_pred_.resize(size, 0);
}

void SDFMap::resetBuffer(const Eigen::Vector3d& min_pos, const Eigen::Vector3d& max_pos) {
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        md_->distance_buffer_[toAddress(x, y, z)] = mp_->default_dist_;
      }
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_->map_voxel_num_(dim)];
  double z[mp_->map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q)
      k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d() {
  Eigen::Vector3i min_esdf = md_->local_bound_min_;
  Eigen::Vector3i max_esdf = md_->local_bound_max_;

  if (mp_->optimistic_) {
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return md_->occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                  0 :
                  std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
  } else {
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              int adr = toAddress(x, y, z);
              return (md_->occupancy_buffer_inflate_[adr] == 1 ||
                      md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3) ?
                  0 :
                  std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF(
          [&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
          [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
          max_esdf[1], 1);
    }
  for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF(
          [&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
          [&](int x, double val) {
            md_->distance_buffer_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
          },
          min_esdf[0], max_esdf[0], 0);
    }

  if (mp_->signed_dist_) {
    // Compute negative distance
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return md_->occupancy_buffer_inflate_
                          [x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) +
                           y * mp_->map_voxel_num_(2) + z] == 0 ?
                  0 :
                  std::numeric_limits<double>::max();
            },
            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF(
            [&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
            [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
            max_esdf[1], 1);
      }
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF(
            [&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
            [&](int x, double val) {
              md_->distance_buffer_neg_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
            },
            min_esdf[0], max_esdf[0], 0);
      }
    // Merge negative distance with positive
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
      for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
          int idx = toAddress(x, y, z);
          if (md_->distance_buffer_neg_[idx] > 0.0)
            md_->distance_buffer_[idx] += (-md_->distance_buffer_neg_[idx] + mp_->resolution_);
        }
  }
}

void SDFMap::setCacheOccupancy(const int& adr, const int& occ) {
  // Add to update list if first visited
  if (md_->count_hit_[adr] == 0 && md_->count_miss_[adr] == 0) md_->cache_voxel_.push(adr);

  if (occ == 0)
    md_->count_miss_[adr] = 1;
  else if (occ == 1)
    md_->count_hit_[adr] += 1;

  // md_->count_hit_and_miss_[adr] += 1;
  // if (occ == 1)
  //   md_->count_hit_[adr] += 1;
  // if (md_->count_hit_and_miss_[adr] == 1)
  //   md_->cache_voxel_.push(adr);
}

void SDFMap::inputPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
    const Eigen::Vector3d& camera_pos) {
  if (point_num == 0) return;
  md_->raycast_num_ += 1;

  Eigen::Vector3d update_min = camera_pos;
  Eigen::Vector3d update_max = camera_pos;
  if (md_->reset_updated_box_) {
    md_->update_min_ = camera_pos;
    md_->update_max_ = camera_pos;
    md_->reset_updated_box_ = false;
  }

  Eigen::Vector3d pt_w, tmp;
  Eigen::Vector3i idx;
  int vox_adr;
  double length;
  for (int i = 0; i < point_num; ++i) {
    auto& pt = points.points[i];
    pt_w << pt.x, pt.y, pt.z;
    int tmp_flag;
    // Set flag for projected point
    if (!isInMap(pt_w)) {
      // Find closest point in map and set free
      pt_w = closetPointInMap(pt_w, camera_pos);
      length = (pt_w - camera_pos).norm();
      if (length > mp_->max_ray_length_)
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
      if (pt_w[2] < 0.2) continue;
      tmp_flag = 0;
    } else {
      length = (pt_w - camera_pos).norm();
      if (length > mp_->max_ray_length_) {
        pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
        if (pt_w[2] < 0.2) continue;
        tmp_flag = 0;
      } else
        tmp_flag = 1;
    }
    posToIndex(pt_w, idx);
    vox_adr = toAddress(idx);
    setCacheOccupancy(vox_adr, tmp_flag);

    for (int k = 0; k < 3; ++k) {
      update_min[k] = min(update_min[k], pt_w[k]);
      update_max[k] = max(update_max[k], pt_w[k]);
    }
    // Raycasting between camera center and point
    if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
      continue;
    else if (md_->flag_rayend_[vox_adr] != md_->raycast_num_)
      md_->flag_rayend_[vox_adr] = md_->raycast_num_;

    caster_->input(camera_pos, pt_w);
    caster_->nextId(idx);
    while (caster_->nextId(idx))
      setCacheOccupancy(toAddress(idx), 0);
  }

  Eigen::Vector3d bound_inf(mp_->local_bound_inflate_, mp_->local_bound_inflate_, 0);
  posToIndex(update_max + bound_inf, md_->local_bound_max_);
  posToIndex(update_min - bound_inf, md_->local_bound_min_);
  boundIndex(md_->local_bound_min_);
  boundIndex(md_->local_bound_max_);
  mr_->local_updated_ = true;

  // Bounding box for subsequent updating
  for (int k = 0; k < 3; ++k) {
    md_->update_min_[k] = min(update_min[k], md_->update_min_[k]);
    md_->update_max_[k] = max(update_max[k], md_->update_max_[k]);
  }

  while (!md_->cache_voxel_.empty()) {
    int adr = md_->cache_voxel_.front();
    md_->cache_voxel_.pop();
    double log_odds_update =
        md_->count_hit_[adr] >= md_->count_miss_[adr] ? mp_->prob_hit_log_ : mp_->prob_miss_log_;
    md_->count_hit_[adr] = md_->count_miss_[adr] = 0;
    if (md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3)
      md_->occupancy_buffer_[adr] = mp_->min_occupancy_log_;
    // Hard Constrian
    else if (md_->occupancy_buffer_[adr] > mp_->min_occupancy_log_)
    {
      md_->occupancy_buffer_[adr] = md_->occupancy_buffer_[adr] + 1e-3;
    }
    else
      md_->occupancy_buffer_[adr] = std::min(
        std::max(md_->occupancy_buffer_[adr] + log_odds_update, mp_->clamp_min_log_),
        mp_->clamp_max_log_);
    // IF OCCUPIED, set observe pose state
    // if (md_->occupancy_buffer_[adr] > mp_->min_occupancy_log_)
    // {
    //   Eigen::Vector3i cam_idx;
    //   posToIndex(camera_pos, cam_idx);
    //   int cam_adr = toAddress(cam_idx);
    //   if (recon_states_[adr] == 0)
    //   {
    //     recon_poses_idx_[adr] = cam_adr;
    //     recon_states_[adr] += 1;
    //   }
    //   else
    //   {
    //     bool add_sign = true;
    //     Eigen::Vector3i cur_idx;
    //     inv_address(recon_poses_idx_[adr], cur_idx);
    //     double dist_tmp = (cur_idx-cam_idx).norm();
    //     if (dist_tmp < mp_->view_thre_)
    //         add_sign = false;
    //     if (add_sign == true)
    //     {
    //       recon_poses_idx_[adr] = cam_adr;
    //       recon_states_[adr] += 1;
    //     }
    //   }
    // }
  }
}

void SDFMap::inputPredictionCloud(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
                       const Eigen::Vector3d& viewpoint)
{
  if (point_num == 0) return;

  Eigen::Vector3d pt_w, tmp;
  Eigen::Vector3i idx;
  int vox_adr;
  double length;

  for (int i=0; i<point_num; ++i)
  {
    auto& pt = points.points[i];
    pt_w << pt.x, pt.y, pt.z;
    int tmp_flag;

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, viewpoint);
      length = (pt_w - viewpoint).norm();
      if (length > mp_->max_ray_length_prediction)
        pt_w = (pt_w - viewpoint) / length * mp_->max_ray_length_prediction + viewpoint;
      if (pt_w[2] < 0.05) continue;
      tmp_flag = 1;
    }
    else
    {
      length = (pt_w - viewpoint).norm();
      if (length > mp_->max_ray_length_prediction)
      {
        pt_w = (pt_w - viewpoint) / length * mp_->max_ray_length_prediction + viewpoint;
        if (pt_w[2] < 0.05) continue;
        tmp_flag = 1;
      }
      else
        tmp_flag = 0;
    }
    posToIndex(pt_w, idx);
    vox_adr = toAddress(idx);
    md_->occupancy_buffer_pred_[vox_adr] = tmp_flag;
    // raycast and inflate
    // int in_step = 1;
    Eigen::Vector3i inflate_;
    caster_->input(pt_w, viewpoint);
    caster_->nextId(idx);
    while (caster_->nextId(idx))
    {
      // for (int x = -in_step; x <= in_step; ++x)
      //   for (int y = -in_step; y <= in_step; ++y)
      //     for (int z = -in_step; z <= in_step; ++z)
      //     {
      //       inflate_ = Eigen::Vector3i(idx(0) + x, idx(1) + y, idx(2) + z);
      //       if (isInMap(inflate_))
      //         md_->occupancy_buffer_pred_[toAddress(inflate_)] = 1;
      //     }
      inflate_pred(idx);
    }
  }
}

Eigen::Vector3d
SDFMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_->map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_->map_min_boundary_ - camera_pt;
  double min_t = 1000000;
  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;
      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }
  return camera_pt + (min_t - 1e-3) * diff;
}

int SDFMap::get_qualified(const Eigen::Vector3i& id, const int& step)
{
  int quali = 0;
  vector<Eigen::Vector3i> nbr_pts;
  inflatePoint(id, step, nbr_pts);
  for (auto i:nbr_pts)
  {
    if (getOccupancy(i) == OCCUPIED)
    {
      quali = 1;
      break;
    }
  }

  return quali;
}

void SDFMap::clearAndInflateLocalMap() {
  // /*clear outside local*/
  // const int vec_margin = 5;

  // Eigen::Vector3i min_cut = md_->local_bound_min_ -
  //     Eigen::Vector3i(mp_->local_map_margin_, mp_->local_map_margin_,
  //     mp_->local_map_margin_);
  // Eigen::Vector3i max_cut = md_->local_bound_max_ +
  //     Eigen::Vector3i(mp_->local_map_margin_, mp_->local_map_margin_,
  //     mp_->local_map_margin_);
  // boundIndex(min_cut);
  // boundIndex(max_cut);

  // Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin,
  // vec_margin); Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); boundIndex(min_cut_m); boundIndex(max_cut_m);

  // // clear data outside the local range

  // for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
  //   for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

  //     for (int z = min_cut_m(2); z < min_cut(2); ++z) {
  //       int idx                       = toAddress(x, y, z);
  //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
  //       md_->distance_buffer_all_[idx] = 10000;
  //     }

  //     for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
  //       int idx                       = toAddress(x, y, z);
  //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
  //       md_->distance_buffer_all_[idx] = 10000;
  //     }
  //   }

  // for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
  //   for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

  //     for (int y = min_cut_m(1); y < min_cut(1); ++y) {
  //       int idx                       = toAddress(x, y, z);
  //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
  //       md_->distance_buffer_all_[idx] = 10000;
  //     }

  //     for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
  //       int idx                       = toAddress(x, y, z);
  //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
  //       md_->distance_buffer_all_[idx] = 10000;
  //     }
  //   }

  // for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
  //   for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {

  //     for (int x = min_cut_m(0); x < min_cut(0); ++x) {
  //       int idx                       = toAddress(x, y, z);
  //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
  //       md_->distance_buffer_all_[idx] = 10000;
  //     }

  //     for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
  //       int idx                       = toAddress(x, y, z);
  //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
  //       md_->distance_buffer_all_[idx] = 10000;
  //     }
  //   }

  // update inflated occupied cells
  // clean outdated occupancy

  int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);

  for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
    for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
      for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z) {
        md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate newest occpuied cells
  for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
    for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
      for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z) {
        int id1 = toAddress(x, y, z);
        if (md_->occupancy_buffer_[id1] > mp_->min_occupancy_log_) {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (auto inf_pt : inf_pts) {
            int idx_inf = toAddress(inf_pt);
            if (idx_inf >= 0 &&
                idx_inf <
                    mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2)) {
              md_->occupancy_buffer_inflate_[idx_inf] = 1;
            }
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_->virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_->virtual_ceil_height_ - mp_->map_origin_(2)) * mp_->resolution_inv_);
    for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
      for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y) {
        // md_->occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
        md_->occupancy_buffer_[toAddress(x, y, ceil_id)] = mp_->clamp_max_log_;
      }
  }
}

double SDFMap::getResolution() {
  return mp_->resolution_;
}

int SDFMap::getVoxelNum() {
  return mp_->map_voxel_num_[0] * mp_->map_voxel_num_[1] * mp_->map_voxel_num_[2];
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
  ori = mp_->map_origin_, size = mp_->map_size_;
}

void SDFMap::getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax) {
  bmin = mp_->box_mind_;
  bmax = mp_->box_maxd_;
}

void SDFMap::getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset) {
  bmin = md_->update_min_;
  bmax = md_->update_max_;
  if (reset) md_->reset_updated_box_ = true;
}

void SDFMap::getOccMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  pcl::PointXYZ pt;
  for (int x = mp_->box_min_(0); x < mp_->box_max_(0); ++x)
    for (int y = mp_->box_min_(1); y < mp_->box_max_(1); ++y)
      for (int z = mp_->box_min_(2); z < mp_->box_max_(2); ++z)
      {
        if (md_->occupancy_buffer_[toAddress(x, y, z)] > mp_->min_occupancy_log_)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2)>0.2)
          {
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
          }
          cloud->points.push_back(pt);
        }
      }
}

double SDFMap::getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) {
  if (!isInMap(pos)) {
    grad.setZero();
    return 0;
  }

  /* trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_->resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);
  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_->resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * mp_->resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_->resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= mp_->resolution_inv_;

  return dist;
}
}  // namespace fast_planner
// SDFMap
