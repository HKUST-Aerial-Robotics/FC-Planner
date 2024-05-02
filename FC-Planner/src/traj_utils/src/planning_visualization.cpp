/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main functions of visualization tools
 *                   for FC-Planner. 
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

#include <traj_utils/planning_visualization.h> 
#include <cstdlib>
#include <ctime>

using std::cout;
using std::endl;
namespace predrecon {
PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh) {
  node = nh;

  traj_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 100);
  pubs_.push_back(traj_pub_);

  topo_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/topo_path", 100);
  pubs_.push_back(topo_pub_);

  predict_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/prediction", 100);
  pubs_.push_back(predict_pub_);

  visib_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/"
                                                          "visib_constraint",
                                                          100);
  pubs_.push_back(visib_pub_);

  frontier_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/frontier", 10000);
  pubs_.push_back(frontier_pub_);

  yaw_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/yaw", 100);
  pubs_.push_back(yaw_pub_);

  viewpoint_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/viewpoints", 1000);
  pubs_.push_back(viewpoint_pub_);
  /* GlobalPlanner */
  pred_pub_ = node.advertise<sensor_msgs::PointCloud2>("/global_planning/pred_cloud", 10);
  localReg_pub_ = node.advertise<sensor_msgs::PointCloud2>("/global_planning/local_region", 10);
  global_pub_ = node.advertise<visualization_msgs::MarkerArray>("/global_planning/global_tour", 10);
  vpg_pub_ = node.advertise<visualization_msgs::MarkerArray>("/global_planning/vp_global", 1);
  internal_pub_ =  node.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
  global_dir_pub_ = node.advertise<visualization_msgs::Marker>("/global_planning/global_dir", 10);
  global_c_pub_ = node.advertise<sensor_msgs::PointCloud2>("/global_planning/cluster", 1);
  global_n_pub_ = node.advertise<visualization_msgs::MarkerArray>("/global_planning/normals", 1);

  /* LocalPlanner */
  local_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/local_tour", 10);
  localob_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/localob_tour", 10);
  localVP_pub_ = node.advertise<visualization_msgs::MarkerArray>("/local_planning/vp_local", 10);

  /* ROSA */
  pcloud_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_vis/input_cloud", 10);
  mesh_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_vis/input_mesh", 1);
  normal_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/input_normal", 10);
  rosa_orientation_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/rosa_orientation", 10);
  drosa_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_vis/drosa_pts", 10);
  le_pts_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_vis/le_pts", 10);
  le_lines_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_vis/le_lines", 10);
  rr_pts_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_vis/rr_pts", 10);
  rr_lines_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_vis/rr_lines", 10);
  decomp_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/branches", 10);
  branch_start_end_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/branches_start_end", 10);
  branch_dir_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/branches_dir", 10);
  cut_plane_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_vis/cut_plane", 10);
  cut_pt_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_vis/cut_pt", 10);
  sub_space_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_vis/sub_space", 1);
  sub_endpts_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/sub_endpts", 1);
  vertex_ID_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_vis/vertex_ID", 1);

  /* ROSA Debug Vis */
  checkPoint_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_debug/checkPoint", 1);
  checkNeigh_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_debug/checkNeigh", 1);
  checkCPdir_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_debug/CPDirection", 1);
  checkRP_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_debug/checkRP", 1);
  checkCPpts_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_debug/CPPoints", 1);
  checkCPptsCluster_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_debug/CPPointsCluster", 1);
  checkBranch_pub_ = node.advertise<visualization_msgs::MarkerArray>("/rosa_debug/checkBranches", 10);
  checkAdj_pub_ = node.advertise<visualization_msgs::Marker>("/rosa_debug/adj", 1);

  /* ROS Opt */
  optArea_pub_ = node.advertise<sensor_msgs::PointCloud2>("/rosa_opt/opt_area", 1);

  /* HCOPP */
  init_vps_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/init_vps", 1);
  sub_vps_hull_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/vps_hull", 1);
  before_opt_vp_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/before_vps", 1);
  after_opt_vp_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/after_vps", 1); 
  hcopp_viewpoints_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/seg_viewpoints", 1);
  hcopp_occ_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/occupied", 1);
  hcopp_internal_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/internal", 1);
  hcopp_fov_pub_ = node.advertise<visualization_msgs::Marker>("/hcopp/fov_set", 1);
  hcopp_uncovered_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/uncovered_area", 1);
  hcopp_validvp_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/valid_vp", 1);
  hcopp_correctnormal_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/correctNormals", 10);
  hcopp_sub_finalvps_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/sub_finalvps", 1);
  hcopp_vps_drone_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/finalvps_drones", 1);
  hcopp_globalseq_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/global_seq", 1);
  hcopp_globalboundary_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/global_boundary", 1);
  hcopp_local_path_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/local_paths", 1);
  hcopp_full_path_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/HCOPP_Path", 1);
  fullatsp_full_path_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/FullATSP_Path", 1);
  fullgdcpca_full_path_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/FullGDCPCA_Path", 1);
  pca_vec_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/PCA_Vec", 1);
  cylinder_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/fit_cylinder_", 1);
  posi_traj_pub_ = node.advertise<quadrotor_msgs::PolynomialTraj>("/fc_planner/position_traj", 1);
  pitch_traj_pub_ = node.advertise<quadrotor_msgs::PolynomialTraj>("/fc_planner/pitch_traj", 1);
  yaw_traj_pub_ = node.advertise<quadrotor_msgs::PolynomialTraj>("/fc_planner/yaw_traj", 1);
  jointSphere_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/JointSphere", 1);
  hcoppYaw_pub_ = node.advertise<visualization_msgs::MarkerArray>("/hcopp/yaw_traj_", 1);
  pathVisible_pub_ = node.advertise<sensor_msgs::PointCloud2>("/hcopp/vis_path_cloud_", 1);

  /* Updates Visualization */
  currentPose_pub_ = node.advertise<visualization_msgs::MarkerArray>("/SGVG/cur_vps_", 1);
  currentVoxels_pub_ = node.advertise<sensor_msgs::PointCloud2>("/SGVG/cur_vox_", 1);

  /* Flight */
  drawFoV_pub_ = node.advertise<visualization_msgs::Marker>("/fc_planner/cmd_fov", 10);
  drone_pub_ = node.advertise<visualization_msgs::Marker>("/fc_planner/drone", 10);
  traveltraj_pub_ = node.advertise<nav_msgs::Path>("/fc_planner/travel_traj", 1, true);
  visible_pub_ = node.advertise<sensor_msgs::PointCloud2>("/fc_planner/vis_points", 1);
  nh.param("hcopp/droneMesh", droneMesh, std::string("null"));

  last_topo_path1_num_ = 0;
  last_topo_path2_num_ = 0;
  last_bspline_phase1_num_ = 0;
  last_bspline_phase2_num_ = 0;
  last_frontier_num_ = 0;
}

void PlanningVisualization::publishSurface(const pcl::PointCloud<pcl::PointXYZ>& input_cloud)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = input_cloud;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  pcloud_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishVisCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  for (auto p:input_cloud->points)
    cloud_pred.points.push_back(p);

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  pathVisible_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishMesh(std::string& mesh)
{
  visualization_msgs::Marker meshModel;
  meshModel.header.frame_id = "world";
  meshModel.header.stamp = ros::Time::now();
  meshModel.id = 0;
  meshModel.ns = "mesh";
  meshModel.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshModel.color.r = 128.0/256.0;
  meshModel.color.g = 128.0/256.0;
  meshModel.color.b = 128.0/256.0;
  meshModel.color.a = 1.0;
  meshModel.scale.x = 1.0;
  meshModel.scale.y = 1.0;
  meshModel.scale.z = 1.0;
  meshModel.pose.orientation.w = 1.0;
  meshModel.mesh_resource = "file://";
  meshModel.mesh_resource += mesh;
  meshModel.mesh_use_embedded_materials = true;

  mesh_pub_.publish(meshModel);
}

void PlanningVisualization::publishSurfaceNormal(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals)
{
  visualization_msgs::MarkerArray pcloud_normals;
  int counter = 0;
  double scale = 3.0;
  for (int i=0; i<(int)input_cloud.points.size(); ++i)
  {
    visualization_msgs::Marker nm;
    nm.header.frame_id = "world";
    nm.header.stamp = ros::Time::now();
    nm.id = counter;
    nm.type = visualization_msgs::Marker::ARROW;
    nm.action = visualization_msgs::Marker::ADD;

    nm.pose.orientation.w = 1.0;
    nm.scale.x = 0.2;
    nm.scale.y = 0.3;
    nm.scale.z = 0.2;

    geometry_msgs::Point pt_;
    pt_.x = input_cloud.points[i].x;
    pt_.y = input_cloud.points[i].y;
    pt_.z = input_cloud.points[i].z;
    nm.points.push_back(pt_);

    pt_.x = input_cloud.points[i].x + scale*normals.points[i].normal_x;
    pt_.y = input_cloud.points[i].y + scale*normals.points[i].normal_y;
    pt_.z = input_cloud.points[i].z + scale*normals.points[i].normal_z;
    nm.points.push_back(pt_);

    nm.color.r = 0.1;
    nm.color.g = 0.2;
    nm.color.b = 0.7;
    nm.color.a = 1.0;
    
    pcloud_normals.markers.push_back(nm);
    counter++;
  }

  normal_pub_.publish(pcloud_normals);
}

void PlanningVisualization::publishROSAOrientation(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals)
{
  visualization_msgs::MarkerArray pcloud_normals;
  int counter = 0;
  double scale = 3.0;
  for (int i=0; i<(int)input_cloud.points.size(); ++i)
  {
    visualization_msgs::Marker nm;
    nm.header.frame_id = "world";
    nm.header.stamp = ros::Time::now();
    nm.id = counter;
    nm.type = visualization_msgs::Marker::ARROW;
    nm.action = visualization_msgs::Marker::ADD;

    nm.pose.orientation.w = 1.0;
    nm.scale.x = 0.2;
    nm.scale.y = 0.3;
    nm.scale.z = 0.2;

    geometry_msgs::Point pt_;
    pt_.x = input_cloud.points[i].x;
    pt_.y = input_cloud.points[i].y;
    pt_.z = input_cloud.points[i].z;
    nm.points.push_back(pt_);

    pt_.x = input_cloud.points[i].x + scale*normals.points[i].normal_x;
    pt_.y = input_cloud.points[i].y + scale*normals.points[i].normal_y;
    pt_.z = input_cloud.points[i].z + scale*normals.points[i].normal_z;
    nm.points.push_back(pt_);

    nm.color.r = 0.2;
    nm.color.g = 0.7;
    nm.color.b = 0.1;
    nm.color.a = 1.0;
    
    pcloud_normals.markers.push_back(nm);
    counter++;
  }

  rosa_orientation_pub_.publish(pcloud_normals);
}

void PlanningVisualization::publish_dROSA(const pcl::PointCloud<pcl::PointXYZ>& local_region)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = local_region;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  drosa_pub_.publish(cloud_msg);
}

void PlanningVisualization::publish_lineextract_vis(Eigen::MatrixXd& skelver, Eigen::MatrixXi& skeladj)
{
  pcl::PointCloud<pcl::PointXYZ> rosa_le_pts;
  pcl::PointXYZ pt;

  for (int i=0; i<skelver.rows(); ++i)
  {
    if (skelver(i,0) < -1e5+1) continue;
    pt.x = skelver(i,0); pt.y = skelver(i,1); pt.z = skelver(i,2);
    rosa_le_pts.points.push_back(pt);
  }

  rosa_le_pts.width = rosa_le_pts.points.size();
  rosa_le_pts.height = 1;
  rosa_le_pts.is_dense = true;
  rosa_le_pts.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(rosa_le_pts, cloud_msg);

  int counter_vpg = 0;

  visualization_msgs::Marker lines;
  lines.header.frame_id = "world";
  lines.header.stamp = ros::Time::now();
  lines.id = counter_vpg;
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;

  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.01;

  lines.color.r = 0.2;
  lines.color.g = 0.8;
  lines.color.b = 0.4;
  lines.color.a = 1.0;
  
  geometry_msgs::Point p1, p2;
  for (int i=0; i<skeladj.rows(); ++i)
  {
    for (int j=0; j<skeladj.cols(); ++j)
    {
      if (skeladj(i,j) == 1)
      {
        p1.x = skelver(i,0); p1.y = skelver(i,1); p1.z = skelver(i,2);
        p2.x = skelver(j,0); p2.y = skelver(j,1); p2.z = skelver(j,2);
        lines.points.push_back(p1);
        lines.points.push_back(p2);
      }
    }
  }

  /* Publish */
  le_pts_pub_.publish(cloud_msg);
  le_lines_pub_.publish(lines);
}

void PlanningVisualization::publish_recenter_vis(Eigen::MatrixXd& skelver, Eigen::MatrixXi& skeladj, Eigen::MatrixXd& realVertices)
{
  pcl::PointCloud<pcl::PointXYZ> rosa_le_pts;
  visualization_msgs::MarkerArray vID;
  pcl::PointXYZ pt;

  for (int i=0; i<realVertices.rows(); ++i)
  {
    if (realVertices(i,0) < -1e5+1) continue;
    pt.x = realVertices(i,0); pt.y = realVertices(i,1); pt.z = realVertices(i,2);

    rosa_le_pts.points.push_back(pt);
  }

  int count = 0;
  for (int j=0; j<(int)skelver.rows(); ++j)
  {
    visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = "world";
    textMarker.header.stamp = ros::Time::now();
    textMarker.ns = "vertex_ID";
    textMarker.id = count;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.action = visualization_msgs::Marker::ADD;

    textMarker.pose.position.x = skelver(j,0);
    textMarker.pose.position.y = skelver(j,1);
    textMarker.pose.position.z = skelver(j,2);
    textMarker.pose.orientation.x = 0.0;
    textMarker.pose.orientation.y = 0.0;
    textMarker.pose.orientation.z = 0.0;
    textMarker.pose.orientation.w = 1.0;
    textMarker.scale.x = 2.0;
    textMarker.scale.y = 2.0;
    textMarker.scale.z = 2.0;
    textMarker.color.r = 0.0;
    textMarker.color.g = 0.0;
    textMarker.color.b = 0.0;
    textMarker.color.a = 1.0;
    textMarker.text = std::to_string(j);

    vID.markers.push_back(textMarker);
    count++;
  }

  rosa_le_pts.width = rosa_le_pts.points.size();
  rosa_le_pts.height = 1;
  rosa_le_pts.is_dense = true;
  rosa_le_pts.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(rosa_le_pts, cloud_msg);

  int counter_vpg = 0;

  visualization_msgs::Marker lines;
  lines.header.frame_id = "world";
  lines.header.stamp = ros::Time::now();
  lines.id = counter_vpg;
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;

  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.3;

  lines.color.r = 0.2;
  lines.color.g = 0.8;
  lines.color.b = 0.4;
  lines.color.a = 1.0;
  
  geometry_msgs::Point p1, p2;
  for (int i=0; i<skeladj.rows(); ++i)
  {
    {
      p1.x = skelver(skeladj(i,0),0); p1.y = skelver(skeladj(i,0),1); p1.z = skelver(skeladj(i,0),2);
      p2.x = skelver(skeladj(i,1),0); p2.y = skelver(skeladj(i,1),1); p2.z = skelver(skeladj(i,1),2);
      lines.points.push_back(p1);
      lines.points.push_back(p2);
    }
  }

  /* Publish */
  rr_pts_pub_.publish(cloud_msg);
  rr_lines_pub_.publish(lines);
  vertex_ID_pub_.publish(vID);
}

void PlanningVisualization::publish_decomposition(Eigen::MatrixXd& nodes, vector<vector<int>>& branches, vector<Eigen::Vector3d>& dirs, vector<Eigen::Vector3d>& centroids)
{
  srand(time(NULL)); 
  visualization_msgs::MarkerArray b_set, bse_set, bdir_set;
  int counter = 0;
  for (int i=0; i<(int)branches.size(); ++i)
  {
    visualization_msgs::Marker segment;
    segment.header.frame_id = "world";
    segment.header.stamp = ros::Time::now();
    segment.id = counter;

    segment.type = visualization_msgs::Marker::LINE_LIST;
    segment.action = visualization_msgs::Marker::ADD;

    segment.pose.orientation.w = 1.0;
    segment.scale.x = 0.3;

    segment.color.r = (double)rand() / (double)RAND_MAX;
    segment.color.g = (double)rand() / (double)RAND_MAX;
    segment.color.b = (double)rand() / (double)RAND_MAX;
    segment.color.a = 1.0;

    geometry_msgs::Point p1, p2;
    for (int j=0; j<(int)branches[i].size()-1; ++j)
    {
      p1.x = nodes(branches[i][j],0); p1.y = nodes(branches[i][j],1); p1.z = nodes(branches[i][j],2); 
      p2.x = nodes(branches[i][j+1],0); p2.y = nodes(branches[i][j+1],1); p2.z = nodes(branches[i][j+1],2);
      segment.points.push_back(p1);
      segment.points.push_back(p2);
    }

    visualization_msgs::Marker begin;
    begin.header.frame_id = "world";
    begin.header.stamp = ros::Time::now();
    begin.id = counter;
    begin.type = visualization_msgs::Marker::CUBE;
    begin.color.r = segment.color.r;
    begin.color.g = segment.color.g;
    begin.color.b = segment.color.b;
    begin.color.a = 0.3;
    begin.scale.x = 1.3;
    begin.scale.y = 1.3;
    begin.scale.z = 1.3;
    begin.pose.orientation.w = 1.0;
    begin.pose.position.x = nodes(branches[i][0],0);
    begin.pose.position.y = nodes(branches[i][0],1);
    begin.pose.position.z = nodes(branches[i][0],2);

    visualization_msgs::Marker end;
    end.header.frame_id = "world";
    end.header.stamp = ros::Time::now();
    end.id = counter+20;
    end.type = visualization_msgs::Marker::SPHERE;
    end.color.r = segment.color.r;
    end.color.g = segment.color.g;
    end.color.b = segment.color.b;
    end.color.a = 1.0;
    end.scale.x = 1.1;
    end.scale.y = 1.1;
    end.scale.z = 1.1;
    end.pose.orientation.w = 1.0;
    end.pose.position.x = nodes(branches[i][(int)branches[i].size()-1],0);
    end.pose.position.y = nodes(branches[i][(int)branches[i].size()-1],1);
    end.pose.position.z = nodes(branches[i][(int)branches[i].size()-1],2);

    double scale = (nodes.row(branches[i][0]) - nodes.row(branches[i][(int)branches[i].size()-1])).norm();
    visualization_msgs::Marker nm;
    nm.header.frame_id = "world";
    nm.header.stamp = ros::Time::now();
    nm.id = counter;
    nm.type = visualization_msgs::Marker::ARROW;
    nm.pose.orientation.w = 1.0;
    nm.scale.x = 0.2;
    nm.scale.y = 0.3;
    nm.scale.z = 0.2;
    geometry_msgs::Point pt_;
    pt_.x = centroids[i](0) - 0.5*scale*dirs[i](0);
    pt_.y = centroids[i](1) - 0.5*scale*dirs[i](1);
    pt_.z = centroids[i](2) - 0.5*scale*dirs[i](2);
    nm.points.push_back(pt_);

    pt_.x = centroids[i](0) + 0.5*scale*dirs[i](0);
    pt_.y = centroids[i](1) + 0.5*scale*dirs[i](1);
    pt_.z = centroids[i](2) + 0.5*scale*dirs[i](2);
    nm.points.push_back(pt_);

    nm.color.r = 0.9;
    nm.color.g = 0.2;
    nm.color.b = 0.1;
    nm.color.a = 1.0;

    b_set.markers.push_back(segment);
    bse_set.markers.push_back(begin);
    bse_set.markers.push_back(end);
    bdir_set.markers.push_back(nm);
    counter++;
  }

  decomp_pub_.publish(b_set);
  branch_start_end_pub_.publish(bse_set);
  branch_dir_pub_.publish(bdir_set);
}

void PlanningVisualization::publishCutPlane(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, Eigen::Vector3d& p, Eigen::Vector3d& v)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = input_cloud;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  cut_plane_pub_.publish(cloud_msg);

  visualization_msgs::Marker lines;
  lines.header.frame_id = "world";
  lines.header.stamp = ros::Time::now();
  lines.id = 0;
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;

  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.02;

  lines.color.r = 0.2;
  lines.color.g = 0.8;
  lines.color.b = 0.4;
  lines.color.a = 1.0;

  geometry_msgs::Point p1, p2;
  p1.x = p(0); p1.y = p(1); p1.z = p(2); 
  p2.x = p(0)+0.1*v(0); p2.y = p(1)+0.1*v(1); p2.z = p(2)+0.1*v(2); 
  lines.points.push_back(p1);
  lines.points.push_back(p2);

  cut_pt_pub_.publish(lines);
}

void PlanningVisualization::publishSubSpace(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& space)
{
  srand((int)time(0));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB p;
  int red, blue, green;
  for (int i=0; i<(int)space.size(); ++i)
  {
    red = rand()%255;
    blue = rand()%255;
    green = rand()%255;
    for (int j=0; j<(int)space[i]->points.size(); ++j)
    {
      p.x = space[i]->points[j].x;
      p.y = space[i]->points[j].y;
      p.z = space[i]->points[j].z;
      // color
      p.r = red;
      p.g = green;
      p.b = blue;
      colored_pcl_ptr->points.push_back(p);
    }
  }

  colored_pcl_ptr->width = colored_pcl_ptr->points.size();
  colored_pcl_ptr->height = 1;
  colored_pcl_ptr->is_dense = true;
  colored_pcl_ptr->header.frame_id = "world";

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*colored_pcl_ptr, cloud_msg);
  sub_space_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishSubEndpts(map<int, vector<Eigen::Vector3d>>& endpts)
{
  srand((int)time(0));
  int counter = 0;
  visualization_msgs::MarkerArray sub_endpts;
  for (const auto& pair:endpts)
  {     
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "sub_space_endpt";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 1.2;
    mk.scale.y = 1.2;
    mk.scale.z = 1.2;
    mk.pose.orientation.w = 1.0;

    mk.pose.position.x = pair.second[0](0);
    mk.pose.position.y = pair.second[0](1);
    mk.pose.position.z = pair.second[0](2);
    sub_endpts.markers.push_back(mk);
    counter++;
    
    mk.id = counter;
    mk.pose.position.x = pair.second[1](0);
    mk.pose.position.y = pair.second[1](1);
    mk.pose.position.z = pair.second[1](2);
    sub_endpts.markers.push_back(mk);
    counter++;
  }

  sub_endpts_pub_.publish(sub_endpts);
}

void PlanningVisualization::publishSegViewpoints(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& seg_vps)
{
  srand((int)time(0));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB p;
  int red, blue, green;

  for (int i=0; i<(int)seg_vps.size(); ++i)
  {
    red = rand()%255;
    blue = rand()%255;
    green = rand()%255;
    for (int j=0; j<(int)seg_vps[i]->points.size(); ++j)
    {
      p.x = seg_vps[i]->points[j].x;
      p.y = seg_vps[i]->points[j].y;
      p.z = seg_vps[i]->points[j].z;
      // color
      p.r = red;
      p.g = green;
      p.b = blue;
      colored_pcl_ptr->points.push_back(p);
    }
  }

  colored_pcl_ptr->width = colored_pcl_ptr->points.size();
  colored_pcl_ptr->height = 1;
  colored_pcl_ptr->is_dense = true;
  colored_pcl_ptr->header.frame_id = "world";

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*colored_pcl_ptr, cloud_msg);
  hcopp_viewpoints_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishOccupied(pcl::PointCloud<pcl::PointXYZ>& occupied)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = occupied;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  hcopp_occ_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishInternal(pcl::PointCloud<pcl::PointXYZ>& internal)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = internal;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  hcopp_internal_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishFOV(const vector<vector<Eigen::Vector3d>>& list1, const vector<vector<Eigen::Vector3d>>& list2)
{
  visualization_msgs::MarkerArray vp_set;
  int counter = 0;
  for (int j=0; j<(int)list1.size(); ++j)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 1.0;
    mk.color.g = 0.1;
    mk.color.b = 0.1;
    mk.color.a = 0.85;
    mk.scale.x = 0.18;
    mk.scale.y = 0.18;
    mk.scale.z = 0.18;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(list1[j].size()); ++i) {
      pt.x = list1[j][i](0);
      pt.y = list1[j][i](1);
      pt.z = list1[j][i](2);
      mk.points.push_back(pt);

      pt.x = list2[j][i](0);
      pt.y = list2[j][i](1);
      pt.z = list2[j][i](2);
      mk.points.push_back(pt);
    }
    
    vp_set.markers.push_back(mk);
    counter++;
  }

  hcopp_validvp_pub_.publish(vp_set);
}

void PlanningVisualization::publishUncovered(pcl::PointCloud<pcl::PointXYZ>& uncovered)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = uncovered;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  hcopp_uncovered_pub_.publish(cloud_msg);
}

void PlanningVisualization::publishRevisedNormal(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const pcl::PointCloud<pcl::Normal>& normals)
{
  visualization_msgs::MarkerArray pcloud_normals;
  int counter = 0;
  double scale = 5.0;
  for (int i=0; i<(int)input_cloud.points.size(); ++i)
  {
    visualization_msgs::Marker nm;
    nm.header.frame_id = "world";
    nm.header.stamp = ros::Time::now();
    nm.id = counter;
    nm.type = visualization_msgs::Marker::ARROW;
    nm.action = visualization_msgs::Marker::ADD;

    nm.pose.orientation.w = 1.0;
    nm.scale.x = 0.2;
    nm.scale.y = 0.3;
    nm.scale.z = 0.2;

    geometry_msgs::Point pt_;
    pt_.x = input_cloud.points[i].x;
    pt_.y = input_cloud.points[i].y;
    pt_.z = input_cloud.points[i].z;
    nm.points.push_back(pt_);

    pt_.x = input_cloud.points[i].x + scale*normals.points[i].normal_x;
    pt_.y = input_cloud.points[i].y + scale*normals.points[i].normal_y;
    pt_.z = input_cloud.points[i].z + scale*normals.points[i].normal_z;
    nm.points.push_back(pt_);

    nm.color.r = 0.1;
    nm.color.g = 0.2;
    nm.color.b = 0.7;
    nm.color.a = 1.0;
    
    pcloud_normals.markers.push_back(nm);
    counter++;
  }

  hcopp_correctnormal_pub_.publish(pcloud_normals);
}

void PlanningVisualization::publishFinalFOV(map<int, vector<vector<Eigen::Vector3d>>>& list1, map<int, vector<vector<Eigen::Vector3d>>>& list2, map<int, vector<double>>& yaws)
{
  srand((int)time(0));
  int red, blue, green;
  visualization_msgs::MarkerArray final_vps;
  visualization_msgs::MarkerArray vps_drones;
  int counter = 0;
  int sub_id;
  for (auto& sub_fov:list1)
  {
    sub_id = sub_fov.first;
    red = rand()%255;
    blue = rand()%255;
    green = rand()%255;

    for (int j=0; j<(int)sub_fov.second.size(); ++j)
    {
      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = counter;
      mk.ns = "current_pose";
      mk.type = visualization_msgs::Marker::LINE_LIST;
      mk.color.r = red;
      mk.color.g = blue;
      mk.color.b = green;
      mk.color.a = 1.0;
      mk.scale.x = 0.15;
      mk.scale.y = 0.15;
      mk.scale.z = 0.15;
      mk.pose.orientation.w = 1.0;

      visualization_msgs::Marker meshROS;
      meshROS.header.frame_id = "world";
      meshROS.header.stamp = ros::Time::now();
      meshROS.id = counter;
      meshROS.ns = "drones_mesh";
      meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
      meshROS.color.r = 0.8;
      meshROS.color.g = 0.0;
      meshROS.color.b = 0.5;
      meshROS.color.a = 1.0;
      meshROS.scale.x = 8.0;
      meshROS.scale.y = 8.0;
      meshROS.scale.z = 9.0;
      meshROS.pose.orientation.w = 1.0;
      meshROS.mesh_resource = "file://";
      meshROS.mesh_resource += droneMesh;

      double yaw = yaws[sub_id][j];
      double yaw_mesh = yaw;
      meshROS.pose.orientation.x = 0.0;
      meshROS.pose.orientation.y = 0.0;
      meshROS.pose.orientation.z = sin(0.5*yaw_mesh);
      meshROS.pose.orientation.w = cos(0.5*yaw_mesh);
      meshROS.pose.position.x = list1[sub_id][j][0](0);
      meshROS.pose.position.y = list1[sub_id][j][0](1);
      meshROS.pose.position.z = list1[sub_id][j][0](2);

      geometry_msgs::Point pt;
      for (int i = 0; i < int(sub_fov.second[j].size()); ++i) {
        pt.x = list1[sub_id][j][i](0);
        pt.y = list1[sub_id][j][i](1);
        pt.z = list1[sub_id][j][i](2);
        mk.points.push_back(pt);

        pt.x = list2[sub_id][j][i](0);
        pt.y = list2[sub_id][j][i](1);
        pt.z = list2[sub_id][j][i](2);
        mk.points.push_back(pt);
      }

      vps_drones.markers.push_back(meshROS);
      final_vps.markers.push_back(mk);
      counter++;
    }
  }

  hcopp_sub_finalvps_pub_.publish(final_vps);
  hcopp_vps_drone_pub_.publish(vps_drones);
}

void PlanningVisualization::publishGlobalSeq(Eigen::Vector3d& start_, vector<Eigen::Vector3d>& sub_rep, vector<int>& global_seq)
{
  vector<Eigen::Vector3d> total_site_;
  total_site_.push_back(start_);
  total_site_.insert(total_site_.begin()+1, sub_rep.begin(), sub_rep.end());
  vector<int> total_seq_;
  total_seq_.push_back(0);
  for (auto x:global_seq)
    total_seq_.push_back(x+1);

  visualization_msgs::MarkerArray global_results;
  int counter = 0;

  visualization_msgs::Marker begin;
  begin.header.frame_id = "world";
  begin.header.stamp = ros::Time::now();
  begin.id = counter;
  begin.ns = "current_pose";
  begin.type = visualization_msgs::Marker::SPHERE;
  begin.color.r = 1.0;
  begin.color.g = 0.0;
  begin.color.b = 0.0;
  begin.color.a = 1.0;
  begin.scale.x = 1.5;
  begin.scale.y = 1.5;
  begin.scale.z = 1.5;
  begin.pose.orientation.w = 1.0;
  begin.pose.position.x = start_(0);
  begin.pose.position.y = start_(1);
  begin.pose.position.z = start_(2);

  global_results.markers.push_back(begin);
  counter++;

  for (int i=0; i<(int)sub_rep.size(); ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.color.r = 0.0;
    mk.color.g = 0.0;
    mk.color.b = 1.0;
    mk.color.a = 1.0;
    mk.scale.x = 1.5;
    mk.scale.y = 1.5;
    mk.scale.z = 1.5;
    mk.pose.orientation.w = 1.0;
    mk.pose.position.x = sub_rep[i](0);
    mk.pose.position.y = sub_rep[i](1);
    mk.pose.position.z = sub_rep[i](2);

    global_results.markers.push_back(mk);
    counter++;
  }

  for (int j=0; j<(int)total_seq_.size()-1; ++j)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 1.5;
    mk.scale.y = 1.5;
    mk.scale.z = 1.5;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    pt.x = total_site_[total_seq_[j]](0);
    pt.y = total_site_[total_seq_[j]](1);
    pt.z = total_site_[total_seq_[j]](2);
    mk.points.push_back(pt);

    pt.x = total_site_[total_seq_[j+1]](0);
    pt.y = total_site_[total_seq_[j+1]](1);
    pt.z = total_site_[total_seq_[j+1]](2);
    mk.points.push_back(pt);

    global_results.markers.push_back(mk);
    counter++;
  }

  hcopp_globalseq_pub_.publish(global_results);
}

void PlanningVisualization::publishGlobalBoundary(Eigen::Vector3d& start_, map<int, vector<int>>& boundary_id_, map<int, vector<Eigen::VectorXd>>& sub_vps, vector<int>& global_seq)
{
  vector<Eigen::Vector3d> boundaries;
  boundaries.push_back(start_);
  Eigen::Vector3d b_s, b_e;
  for (auto id:global_seq)
  {
    if ((int)boundary_id_.find(id)->second.size() == 2)
    {
      b_s(0) = sub_vps.find(id)->second[boundary_id_.find(id)->second[0]](0);
      b_s(1) = sub_vps.find(id)->second[boundary_id_.find(id)->second[0]](1);
      b_s(2) = sub_vps.find(id)->second[boundary_id_.find(id)->second[0]](2);
      boundaries.push_back(b_s);
      b_e(0) = sub_vps.find(id)->second[boundary_id_.find(id)->second[1]](0);
      b_e(1) = sub_vps.find(id)->second[boundary_id_.find(id)->second[1]](1);
      b_e(2) = sub_vps.find(id)->second[boundary_id_.find(id)->second[1]](2);
      boundaries.push_back(b_e);
    }
    else
    {
      b_s(0) = sub_vps.find(id)->second[boundary_id_.find(id)->second[0]](0);
      b_s(1) = sub_vps.find(id)->second[boundary_id_.find(id)->second[0]](1);
      b_s(2) = sub_vps.find(id)->second[boundary_id_.find(id)->second[0]](2);
      boundaries.push_back(b_s);
    }
  }
  
  visualization_msgs::MarkerArray boundary_results;
  int counter = 0;
  for (int i=0; i<(int)boundaries.size(); ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 1.0;
    mk.color.a = 1.0;
    mk.scale.x = 1.2;
    mk.scale.y = 1.2;
    mk.scale.z = 1.2;
    mk.pose.orientation.w = 1.0;
    mk.pose.position.x = boundaries[i](0);
    mk.pose.position.y = boundaries[i](1);
    mk.pose.position.z = boundaries[i](2);

    boundary_results.markers.push_back(mk);
    counter++;
  }

  for (int j=0; j<(int)boundaries.size()-1; ++j)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 1.0;
    mk.scale.y = 1.0;
    mk.scale.z = 1.0;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    pt.x = boundaries[j](0);
    pt.y = boundaries[j](1);
    pt.z = boundaries[j](2);
    mk.points.push_back(pt);

    pt.x = boundaries[j+1](0);
    pt.y = boundaries[j+1](1);
    pt.z = boundaries[j+1](2);
    mk.points.push_back(pt);

    boundary_results.markers.push_back(mk);
    counter++;
  }

  hcopp_globalboundary_pub_.publish(boundary_results);
}

void PlanningVisualization::publishLocalPath(map<int, vector<Eigen::VectorXd>>& sub_paths_)
{
  srand((int)time(0));
  int red, blue, green;
  vector<Eigen::VectorXd> vps_;
  int counter = 0;
  visualization_msgs::MarkerArray local_results;
  for (const auto& pair:sub_paths_)
  {
    vps_ = pair.second;
    red = rand()%255;
    blue = rand()%255;
    green = rand()%255;

    for (int i=0; i<(int)vps_.size()-1; ++i)
    {
      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = counter;
      mk.ns = "current_pose";
      mk.type = visualization_msgs::Marker::LINE_LIST;
      mk.color.r = red;
      mk.color.g = blue;
      mk.color.b = green;
      mk.color.a = 1.0;
      mk.scale.x = 1.0;
      mk.scale.y = 1.0;
      mk.scale.z = 1.0;
      mk.pose.orientation.w = 1.0;

      geometry_msgs::Point pt;
      pt.x = vps_[i](0);
      pt.y = vps_[i](1);
      pt.z = vps_[i](2);
      mk.points.push_back(pt);

      pt.x = vps_[i+1](0);
      pt.y = vps_[i+1](1);
      pt.z = vps_[i+1](2);
      mk.points.push_back(pt);

      local_results.markers.push_back(mk);
      counter++;
    }
  }

  hcopp_local_path_pub_.publish(local_results);
}

void PlanningVisualization::publishHCOPPPath(vector<Eigen::VectorXd>& fullpath_)
{
  // srand((int)time(0));
  // int red, blue, green;
  // red = rand()%255;
  // green = rand()%255;
  // blue = rand()%255;
  int counter = 0;
  visualization_msgs::MarkerArray hcopp_results;
  for (int i=0; i<(int)fullpath_.size()-1; ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 0.5;
    mk.scale.y = 0.5;
    mk.scale.z = 0.5;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    pt.x = fullpath_[i](0);
    pt.y = fullpath_[i](1);
    pt.z = fullpath_[i](2);
    mk.points.push_back(pt);

    pt.x = fullpath_[i+1](0);
    pt.y = fullpath_[i+1](1);
    pt.z = fullpath_[i+1](2);
    mk.points.push_back(pt);

    hcopp_results.markers.push_back(mk);
    counter++;
  }

  hcopp_full_path_pub_.publish(hcopp_results);
}

void PlanningVisualization::publishFullATSPPath(vector<Eigen::VectorXd>& fullpath_)
{
  // srand((int)time(0));
  // int red, blue, green;
  // red = rand()%255;
  // green = rand()%255;
  // blue = rand()%255;
  int counter = 0;
  visualization_msgs::MarkerArray fullatsp_results;
  for (int i=0; i<(int)fullpath_.size()-1; ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 0.0;
    mk.color.g = 0.0;
    mk.color.b = 1.0;
    mk.color.a = 1.0;
    mk.scale.x = 0.5;
    mk.scale.y = 0.5;
    mk.scale.z = 0.5;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    pt.x = fullpath_[i](0);
    pt.y = fullpath_[i](1);
    pt.z = fullpath_[i](2);
    mk.points.push_back(pt);

    pt.x = fullpath_[i+1](0);
    pt.y = fullpath_[i+1](1);
    pt.z = fullpath_[i+1](2);
    mk.points.push_back(pt);

    fullatsp_results.markers.push_back(mk);
    counter++;
  }

  fullatsp_full_path_pub_.publish(fullatsp_results);
}

void PlanningVisualization::publishFullGDCPCAPath(vector<Eigen::VectorXd>& fullpath_)
{
  int counter = 0;
  visualization_msgs::MarkerArray fullgdcpca_results;
  for (int i=0; i<(int)fullpath_.size()-1; ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_pose";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 0.0;
    mk.color.g = 1.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 0.5;
    mk.scale.y = 0.5;
    mk.scale.z = 0.5;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    pt.x = fullpath_[i](0);
    pt.y = fullpath_[i](1);
    pt.z = fullpath_[i](2);
    mk.points.push_back(pt);

    pt.x = fullpath_[i+1](0);
    pt.y = fullpath_[i+1](1);
    pt.z = fullpath_[i+1](2);
    mk.points.push_back(pt);

    fullgdcpca_results.markers.push_back(mk);
    counter++;
  }

  fullgdcpca_full_path_pub_.publish(fullgdcpca_results);
}

void PlanningVisualization::publishPCAVec(vector<Eigen::Vector3d>& sub_center, map<int, Eigen::Matrix3d>& sub_pcavec)
{
  visualization_msgs::MarkerArray pca_results;
  int sub_id, counter = 0;
  vector<double> red = {1.0, 0.0, 0.0};
  vector<double> green = {0.0, 1.0, 0.0};
  vector<double> blue = {0.0, 0.0, 1.0};
  double scale = 5.0;

  for (auto& pair:sub_pcavec)
  {
    sub_id = pair.first;
    Eigen::Vector3d center_ = sub_center[sub_id];
    for (int i=0; i<3; ++i)
    {
      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = counter;
      mk.ns = "pca_vec";
      mk.type = visualization_msgs::Marker::LINE_LIST;
      mk.color.r = red[i];
      mk.color.g = green[i];
      mk.color.b = blue[i];
      mk.color.a = 1.0;
      mk.scale.x = 0.5;
      mk.scale.y = 0.5;
      mk.scale.z = 0.5;
      mk.pose.orientation.w = 1.0;

      geometry_msgs::Point pt;
      pt.x = center_(0);
      pt.y = center_(1);
      pt.z = center_(2);
      mk.points.push_back(pt);

      pt.x = center_(0) + scale*pair.second(i,0);
      pt.y = center_(1) + scale*pair.second(i,1);
      pt.z = center_(2) + scale*pair.second(i,2);
      mk.points.push_back(pt);

      pca_results.markers.push_back(mk);
      counter++;
    }
  }
  
  pca_vec_pub_.publish(pca_results);
}

void PlanningVisualization::publishVPOpt(pcl::PointCloud<pcl::PointNormal>::Ptr& before_, pcl::PointCloud<pcl::PointNormal>::Ptr& after_)
{
  pcl::PointCloud<pcl::PointXYZ> before_cloud;
  pcl::PointXYZ before_pt;
  for (int i=0; i<(int)before_->points.size(); ++i)
  {
    before_pt.x = before_->points[i].x; 
    before_pt.y = before_->points[i].y; 
    before_pt.z = before_->points[i].z;
    before_cloud.points.push_back(before_pt); 
  }
  before_cloud.width = before_cloud.points.size();
  before_cloud.height = 1;
  before_cloud.is_dense = true;
  before_cloud.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 before_msg;
  pcl::toROSMsg(before_cloud, before_msg);
  before_opt_vp_pub_.publish(before_msg);

  pcl::PointCloud<pcl::PointXYZ> after_cloud;
  pcl::PointXYZ after_pt;
  for (int i=0; i<(int)after_->points.size(); ++i)
  {
    after_pt.x = after_->points[i].x; 
    after_pt.y = after_->points[i].y; 
    after_pt.z = after_->points[i].z;
    after_cloud.points.push_back(after_pt); 
  }
  after_cloud.width = after_cloud.points.size();
  after_cloud.height = 1;
  after_cloud.is_dense = true;
  after_cloud.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 after_msg;
  pcl::toROSMsg(after_cloud, after_msg);
  after_opt_vp_pub_.publish(after_msg);
}

void PlanningVisualization::publishFitCylinder(map<int, vector<double>>& cylinder_param)
{
  visualization_msgs::MarkerArray cylinder_set;
  int counter = 0;
  srand((int)time(0));
  Eigen::Vector3d rot_vec;

  for (auto& pair:cylinder_param)
  { 
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "fit_cylinder";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 0.5;
    mk.scale.y = 0.5;
    mk.scale.z = 0.5;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    pt.x = pair.second[9];
    pt.y = pair.second[10];
    pt.z = pair.second[11];
    mk.points.push_back(pt);

    pt.x = pair.second[0];
    pt.y = pair.second[1];
    pt.z = pair.second[2];
    mk.points.push_back(pt);

    pt.x = pair.second[0];
    pt.y = pair.second[1];
    pt.z = pair.second[2];
    mk.points.push_back(pt);

    pt.x = pair.second[6];
    pt.y = pair.second[7];
    pt.z = pair.second[8];
    mk.points.push_back(pt);

    cylinder_set.markers.push_back(mk);
    counter++;
  }

  cylinder_pub_.publish(cylinder_set);
}

void PlanningVisualization::publishHCOPPTraj(quadrotor_msgs::PolynomialTraj& posi, quadrotor_msgs::PolynomialTraj& pitch, quadrotor_msgs::PolynomialTraj& yaw)
{
  posi_traj_pub_.publish(posi);
  pitch_traj_pub_.publish(pitch);
  yaw_traj_pub_.publish(yaw);
}

void PlanningVisualization::publishJointSphere(vector<Eigen::Vector3d>& joints, double& radius, vector<vector<Eigen::Vector3d>>& InnerVps)
{
  visualization_msgs::MarkerArray JointSpheres;
  int counter = 0, vpscount = 0;

  for (int i=0; i<(int)joints.size(); ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "JointSphere";
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.color.r = 1.0;
    mk.color.g = 0.5;
    mk.color.b = 0.0;
    mk.color.a = 0.5;
    mk.scale.x = 2*radius;
    mk.scale.y = 2*radius;
    mk.scale.z = 2*radius;
    mk.pose.orientation.w = 1.0;
    mk.pose.position.x = joints[i](0);
    mk.pose.position.y = joints[i](1);
    mk.pose.position.z = joints[i](2);

    JointSpheres.markers.push_back(mk);
    counter++;

    for (int j=0; j<(int)InnerVps[i].size(); ++j)
    {
      visualization_msgs::Marker vp;
      vp.header.frame_id = "world";
      vp.header.stamp = ros::Time::now();
      vp.id = vpscount;
      vp.ns = "JointVp";
      vp.type = visualization_msgs::Marker::CUBE;
      vp.color.r = 0.0;
      vp.color.g = 0.0;
      vp.color.b = 1.0;
      vp.color.a = 1.0;
      vp.scale.x = 1.5;
      vp.scale.y = 1.5;
      vp.scale.z = 1.5;
      vp.pose.orientation.w = 1.0;
      vp.pose.position.x = InnerVps[i][j](0);
      vp.pose.position.y = InnerVps[i][j](1);
      vp.pose.position.z = InnerVps[i][j](2);

      JointSpheres.markers.push_back(vp);
      vpscount++;
    }
  }

  jointSphere_pub_.publish(JointSpheres);
}

void PlanningVisualization::publishYawTraj(vector<Eigen::Vector3d>& waypt, vector<double>& yaw)
{
  visualization_msgs::MarkerArray yaw_traj;
  int counter = 0;
  double scale = 3.0;
  for (int i=0; i<(int)yaw.size(); ++i)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "yaw_traj";
    mk.type = visualization_msgs::Marker::ARROW;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.2;
    mk.scale.y = 0.4;
    mk.scale.z = 0.3;
    mk.color.r = 0.0;
    mk.color.g = 0.0;
    mk.color.b = 1.0;
    mk.color.a = 1.0;

    geometry_msgs::Point pt_;
    pt_.x = waypt[i](0);
    pt_.y = waypt[i](1);
    pt_.z = waypt[i](2);
    mk.points.push_back(pt_);

    pt_.x = waypt[i](0) + scale*cos(yaw[i]);
    pt_.y = waypt[i](1) + scale*sin(yaw[i]);
    pt_.z = waypt[i](2);
    mk.points.push_back(pt_);

    yaw_traj.markers.push_back(mk);
    counter++;
  }

  hcoppYaw_pub_.publish(yaw_traj);
}

void PlanningVisualization::publishCurrentFoV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2, const double& yaw)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_fov";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.color.r = 0.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.2;
  mk.scale.y = 0.2;
  mk.scale.z = 0.2;
  mk.pose.orientation.w = 1.0;

  visualization_msgs::Marker meshROS;
  meshROS.header.frame_id = "world";
  meshROS.header.stamp = ros::Time::now();
  meshROS.id = 0;
  meshROS.ns = "current_mesh";
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.color.a = 1.0;
  meshROS.scale.x = 5.0;
  meshROS.scale.y = 5.0;
  meshROS.scale.z = 6.0;
  meshROS.pose.orientation.w = 1.0;
  meshROS.mesh_resource = "file://";
  meshROS.mesh_resource += droneMesh;
  meshROS.mesh_use_embedded_materials = true;

  meshROS.action = visualization_msgs::Marker::DELETE;
  drone_pub_.publish(meshROS);
  mk.action = visualization_msgs::Marker::DELETE;
  drawFoV_pub_.publish(mk);

  if (list1.size() == 0) return;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) 
  {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::Marker::ADD;
  drawFoV_pub_.publish(mk);

  // double yaw_mesh = yaw + 45.0 * M_PI / 180.0;
  double yaw_mesh = yaw;
  meshROS.pose.orientation.x = 0.0;
  meshROS.pose.orientation.y = 0.0;
  meshROS.pose.orientation.z = sin(0.5*yaw_mesh);
  meshROS.pose.orientation.w = cos(0.5*yaw_mesh);
  meshROS.pose.position.x = list1[0](0);
  meshROS.pose.position.y = list1[0](1);
  meshROS.pose.position.z = list1[0](2);
  meshROS.action = visualization_msgs::Marker::ADD;
  drone_pub_.publish(meshROS);
}

void PlanningVisualization::publishTravelTraj(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id)
{
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "world";
  // path_msg.header.seq = id;

  for (const auto& point : path)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = point(0);
    pose.pose.position.y = point(1);
    pose.pose.position.z = point(2);

    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    pose.header.stamp=ros::Time::now();;
    pose.header.frame_id="world";

    path_msg.poses.push_back(pose);
  }

  traveltraj_pub_.publish(path_msg);
}

void PlanningVisualization::publishVisiblePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& currentCloud, int id)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = *currentCloud;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  visible_pub_.publish(cloud_msg);

  // sensor_msgs::PointCloud2 emptyPointCloudMsg;
  // emptyPointCloudMsg.header.frame_id = "world";
  // visible_pub_.publish(emptyPointCloudMsg);
}

void PlanningVisualization::publishCheckNeigh(Eigen::Vector3d& checkPoint, const pcl::PointCloud<pcl::PointXYZ>& checkNeigh, Eigen::MatrixXi& edgeMat)
{
  double radius = 1.5;
  
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 1;
  mk.ns = "rosa_debug_ckpt";
  mk.type = visualization_msgs::Marker::SPHERE;
  mk.color.r = 0.0;
  mk.color.g = 0.0;
  mk.color.b = 1.0;
  mk.color.a = 0.8;
  mk.scale.x = radius;
  mk.scale.y = radius;
  mk.scale.z = radius;
  mk.pose.orientation.w = 1.0;
  mk.pose.position.x = checkPoint(0);
  mk.pose.position.y = checkPoint(1);
  mk.pose.position.z = checkPoint(2);

  checkPoint_pub_.publish(mk);

  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = checkNeigh;
  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  checkNeigh_pub_.publish(cloud_msg);

  int counter_vpg = 0;
  visualization_msgs::Marker lines;
  lines.header.frame_id = "world";
  lines.header.stamp = ros::Time::now();
  lines.id = counter_vpg;
  lines.ns = "rosa_debug_adj";
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;

  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.05;

  lines.color.r = 0.2;
  lines.color.g = 0.0;
  lines.color.b = 0.8;
  lines.color.a = 0.8;

  geometry_msgs::Point p1, p2;
  for (int i=0; i<(int)edgeMat.rows(); ++i)
  {
    for (int j=0; j<(int)edgeMat.cols(); ++j)
    {
      if (edgeMat(i,j) == 1 && i >= j)
      {
        p1.x = checkNeigh.points[i].x; p1.y = checkNeigh.points[i].y; p1.z = checkNeigh.points[i].z;
        p2.x = checkNeigh.points[j].x; p2.y = checkNeigh.points[j].y; p2.z = checkNeigh.points[j].z;
        lines.points.push_back(p1);
        lines.points.push_back(p2);
      }
    }
  }

  checkAdj_pub_.publish(lines);
}

void PlanningVisualization::publishCheckCP(Eigen::Vector3d& CPPoint, Eigen::Vector3d& CPDir, Eigen::Vector3d& checkRP, const pcl::PointCloud<pcl::PointXYZ>& CPPts, const pcl::PointCloud<pcl::PointXYZ>& CPPtsCluster)
{
  double scale = 3.0;
  visualization_msgs::Marker nm;
  nm.header.frame_id = "world";
  nm.header.stamp = ros::Time::now();
  nm.id = 1;
  nm.ns = "rosa_debug_CPdir";
  nm.type = visualization_msgs::Marker::ARROW;

  nm.pose.orientation.w = 1.0;
  nm.scale.x = 0.2;
  nm.scale.y = 0.3;
  nm.scale.z = 0.2;

  geometry_msgs::Point pt_;
  pt_.x = CPPoint(0);
  pt_.y = CPPoint(1);
  pt_.z = CPPoint(2);
  nm.points.push_back(pt_);

  pt_.x = CPPoint(0) + scale*CPDir(0);
  pt_.y = CPPoint(1) + scale*CPDir(1);
  pt_.z = CPPoint(2) + scale*CPDir(2);
  nm.points.push_back(pt_);

  nm.color.r = 0.7;
  nm.color.g = 0.2;
  nm.color.b = 0.4;
  nm.color.a = 1.0;
  checkCPdir_pub_.publish(nm);

  double radius = 1.5;
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 1;
  mk.ns = "rosa_debug_ckRP";
  mk.type = visualization_msgs::Marker::CUBE;
  mk.color.r = 1.0;
  mk.color.g = 0.5;
  mk.color.b = 0.2;
  mk.color.a = 0.8;
  mk.scale.x = radius;
  mk.scale.y = radius;
  mk.scale.z = radius;
  mk.pose.orientation.w = 1.0;
  mk.pose.position.x = checkRP(0);
  mk.pose.position.y = checkRP(1);
  mk.pose.position.z = checkRP(2);
  checkRP_pub_.publish(mk);

  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = CPPts;
  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);
  checkCPpts_pub_.publish(cloud_msg);

  pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
  cloud_cluster = CPPtsCluster;
  cloud_cluster.width = cloud_cluster.points.size();
  cloud_cluster.height = 1;
  cloud_cluster.is_dense = true;
  cloud_cluster.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg_cluster;
  pcl::toROSMsg(cloud_cluster, cloud_msg_cluster);
  checkCPptsCluster_pub_.publish(cloud_msg_cluster);
}

void PlanningVisualization::publishUpdatesPose(pcl::PointCloud<pcl::PointXYZ>& visCloud, vector<vector<Eigen::Vector3d>>& list1, vector<vector<Eigen::Vector3d>>& list2, vector<double>& yaws)
{
  visualization_msgs::MarkerArray vps;
  int counter = 0;
  for (int j=0; j<(int)list1.size(); ++j)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "current_vps";
    mk.type = visualization_msgs::Marker::LINE_LIST;
    mk.color.r = 0.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;
    mk.pose.orientation.w = 1.0;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(list1[j].size()); ++i) 
    {
      pt.x = list1[j][i](0);
      pt.y = list1[j][i](1);
      pt.z = list1[j][i](2);
      mk.points.push_back(pt);

      pt.x = list2[j][i](0);
      pt.y = list2[j][i](1);
      pt.z = list2[j][i](2);
      mk.points.push_back(pt);
    }

    vps.markers.push_back(mk);
    counter++;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud_pred;
  cloud_pred = visCloud;

  cloud_pred.width = cloud_pred.points.size();
  cloud_pred.height = 1;
  cloud_pred.is_dense = true;
  cloud_pred.header.frame_id = "world";
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_pred, cloud_msg);

  currentPose_pub_.publish(vps);
  currentVoxels_pub_.publish(cloud_msg);

  ros::Duration(3.0).sleep();
}

void PlanningVisualization::publishVpsCHull(std::map<int, std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>>& vpHull, vector<Eigen::Vector3d>& hamiPath)
{
  visualization_msgs::MarkerArray vpHulls;
  srand((int)time(0));
  int red, blue, green;
  int counter = 0;
  for (const auto& pair:vpHull)
  {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> hull = pair.second;
    red = rand()%255;
    blue = rand()%255;
    green = rand()%255;

    if (hull.size() == 0) continue;
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "vps_hull";
    marker.id = counter;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    for (const auto& edge : hull)
    {
      geometry_msgs::Point p1, p2;
      p1.x = edge.first[0]; p1.y = edge.first[1]; p1.z = edge.first[2];
      p2.x = edge.second[0]; p2.y = edge.second[1]; p2.z = edge.second[2];
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.color.a = 0.5;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    // vpHulls.markers.push_back(marker);
    counter++;
  }

  visualization_msgs::Marker path;
  path.header.frame_id = "world";
  path.header.stamp = ros::Time();
  path.ns = "hamiPath";
  path.id = 0;
  path.type = visualization_msgs::Marker::LINE_LIST;

  for (int i=0; i<(int)hamiPath.size()-1; ++i)
  {
    geometry_msgs::Point p1, p2;
    p1.x = hamiPath[i](0); p1.y = hamiPath[i](1); p1.z = hamiPath[i](2);
    p2.x = hamiPath[i+1](0); p2.y = hamiPath[i+1](1); p2.z = hamiPath[i+1](2);
    path.points.push_back(p1);
    path.points.push_back(p2);
  }

  path.pose.orientation.w = 1.0;
  path.scale.x = 0.5;
  path.color.a = 1.0;
  path.color.r = 1.0;
  path.color.g = 0.0;
  path.color.b = 0.0;
  vpHulls.markers.push_back(path);

  sub_vps_hull_pub_.publish(vpHulls);
}

void PlanningVisualization::publishInitVps(pcl::PointCloud<pcl::PointNormal>::Ptr& init_vps)
{
  visualization_msgs::MarkerArray init_vps_markers;
  int counter = 0;
  double scale = 3.0;

  for (int i=0; i<(int)init_vps->points.size(); ++i)
  {
    visualization_msgs::Marker nm;
    nm.header.frame_id = "world";
    nm.header.stamp = ros::Time::now();
    nm.id = counter;
    nm.ns = "init_vps_dir";
    nm.type = visualization_msgs::Marker::ARROW;
    nm.action = visualization_msgs::Marker::ADD;

    nm.pose.orientation.w = 1.0;
    nm.scale.x = 0.2;
    nm.scale.y = 0.3;
    nm.scale.z = 0.2;

    geometry_msgs::Point pt_;
    pt_.x = init_vps->points[i].x;
    pt_.y = init_vps->points[i].y;
    pt_.z = init_vps->points[i].z;
    nm.points.push_back(pt_);

    pt_.x = init_vps->points[i].x + scale*init_vps->points[i].normal_x;
    pt_.y = init_vps->points[i].y + scale*init_vps->points[i].normal_y;
    pt_.z = init_vps->points[i].z + scale*init_vps->points[i].normal_z;

    nm.points.push_back(pt_);

    nm.color.r = 0.0;
    nm.color.g = 0.4;
    nm.color.b = 0.9;
    nm.color.a = 0.8;

    init_vps_markers.markers.push_back(nm);
    counter++;

    visualization_msgs::Marker pos;
    pos.header.frame_id = "world";
    pos.header.stamp = ros::Time::now();
    pos.id = counter;
    pos.ns = "init_vps_pos";
    pos.type = visualization_msgs::Marker::SPHERE;
    pos.color.r = 0.0;
    pos.color.g = 0.0;
    pos.color.b = 0.0;
    pos.color.a = 1.0;
    pos.scale.x = 0.5;
    pos.scale.y = 0.5;
    pos.scale.z = 0.5;
    pos.pose.orientation.w = 1.0;
    pos.pose.position.x = init_vps->points[i].x;
    pos.pose.position.y = init_vps->points[i].y;
    pos.pose.position.z = init_vps->points[i].z;

    init_vps_markers.markers.push_back(pos);
    counter++;
  }

  init_vps_pub_.publish(init_vps_markers);
}

void PlanningVisualization::publishOptArea(vector<pcl::PointCloud<pcl::PointXYZ>>& optArea)
{
  srand((int)time(0));
  pcl::PointCloud<pcl::PointXYZRGB> colorCloud;
  pcl::PointXYZRGB p;
  int red, blue, green;
  for (int i=0; i<(int)optArea.size(); ++i)
  {
    red = rand()%255;
    blue = rand()%255;
    green = rand()%255;
    for (int j=0; j<(int)optArea[i].points.size(); ++j)
    {
      p.x = optArea[i].points[j].x;
      p.y = optArea[i].points[j].y;
      p.z = optArea[i].points[j].z;
      // color
      p.r = red;
      p.g = green;
      p.b = blue;
      colorCloud.points.push_back(p);
    }
  }

  colorCloud.width = colorCloud.points.size();
  colorCloud.height = 1;
  colorCloud.is_dense = true;
  colorCloud.header.frame_id = "world";

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(colorCloud, cloud_msg);
  optArea_pub_.publish(cloud_msg);
}

}
