/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main algorithm of the skeleton extraction,
 *                   decomposition, and space allocation in FC-Planner. In skeleton 
 *                   extraction part, we refer to the paper "Curve Skeleton  
 *                   Extraction from Incomplete Point Cloud" <https://dl.acm.org/
 *                   doi/pdf/10.1145/1531326.1531377>.
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

#include <rosa/rosa_main.h>

namespace predrecon
{ 
  void ROSA_main::init(ros::NodeHandle& nh)
  {
    /* Params */
    nh.param("rosa_main/pcd", input_pcd, string("null"));
    nh.param("rosa_main/radius", Radius, -1.0);
    nh.param("rosa_main/th_mah", th_mah, -1.0);
    nh.param("rosa_main/delta", delta, -1.0);
    nh.param("rosa_main/num_drosa", numiter_drosa, -1);
    nh.param("rosa_main/num_dcrosa", numiter_dcrosa, -1);
    nh.param("rosa_main/k_KNN", k_KNN, -1);
    nh.param("rosa_main/sample_r", sample_radius, -1.0);
    nh.param("rosa_main/alpha", alpha_recenter, -1.0);
    nh.param("rosa_main/pt_downsample_size", pt_downsample_voxel_size, -1.0);
    nh.param("rosa_main/estimation_number", ne_KNN, -1);
    nh.param("rosa_main/upper_bound_angle_inner_decomp", angle_upper, -1.0);
    nh.param("rosa_main/upper_bound_length_inner_decomp", length_upper, -1.0);
    nh.param("rosa_main/Prune", prune_flag, false);
    nh.param("rosa_main/lower_bound_length", length_lower, -1.0);
    nh.param("rosa_main/lower_bound_prune_angle", prune_lower, -1.0);
    nh.param("rosa_main/upper_bound_original_points_num", ori_num, -1);
    nh.param("rosa_main/Ground", ground, false);
    nh.param("rosa_main/estimation_num", estNum, -1);
    /* Visualization */
    vis_utils_.reset(new PlanningVisualization(nh));
    visFlag = true;
    vis_timer_ = nh.createTimer(ros::Duration(0.1), &ROSA_main::VisCallback, this);
    /* Initialization */
    pcd_size_ = 0;
    ROS_INFO("\033[34m[SSD] Initialized!\033[34m");
  }
  /* ROSA main function */
  void ROSA_main::main()
  {
    pcloud_read_off();
    auto rosa_t1 = std::chrono::high_resolution_clock::now();
    normalize();

    MAHADJ.resize(pcd_size_, pcd_size_);
    MAHADJ.setZero();
    pset.resize(pcd_size_, 3);
    vset.resize(pcd_size_, 3);
    vvar.resize(pcd_size_, 1);
    P.datas = new double[pcd_size_*3]();
    for (int idx=0; idx<pcd_size_; idx++)
    {  P.datas[idx] = P.pts_->points[idx].x; P.datas[idx+pcd_size_] = P.pts_->points[idx].y; P.datas[idx+2*pcd_size_] = P.pts_->points[idx].z;} 
    
    pcloud_adj_matrix_mahalanobis(Radius);
    rosa_drosa();
    rosa_dcrosa();
    rosa_lineextract();
    rosa_recenter();
    graph_decomposition();
    inner_decomposition();
    branch_merge();
    if (prune_flag == true)
      prune_branches();

    for (auto x:P.branches)
    {
      cout << "{";
      for (auto y:x)
        cout << y << " ";
      cout << "}" << endl;
    }
    /* Invert Normalization */
    P.scale = norm_scale; 
    P.center(0) = centroid(0); P.center(1) = centroid(1); P.center(2) = centroid(2); 
    restore_scale();
    /* distribute original pcloud */
    distribute_ori_cloud();
    cal_inner_dist();
    /* store real scale graph */
    storeRealGraph();

    auto rosa_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> rosa_ms = rosa_t2 - rosa_t1;
    double rt = (double)rosa_ms.count();
    ROS_INFO("\033[34m[SSD] SSD latency = %lf ms.\033[34m", rt);
    ROS_INFO("\033[35m[SSD] --- <SSD finished> --- \033[35m");
  }

  void ROSA_main::VisCallback(const ros::TimerEvent& e)
  {
    if (visFlag == true)
    {
    /* Vis */
    pcl::PointCloud<pcl::PointXYZ> pcloud, calcloud, drosa, cutplane;
    pcl::PointCloud<pcl::Normal> normals, orientations;
    pcl::PointXYZ pt_, cutpt_; pcl::Normal n_, o_;
    for (int i=0; i<(int)P.ori_pts_->points.size(); ++i)
    {
      pt_.x = P.ori_pts_->points[i].x;
      pt_.y = P.ori_pts_->points[i].y;
      pt_.z = P.ori_pts_->points[i].z;
      pcloud.points.push_back(pt_);
    }
    for (int j=0; j<(int)P.pts_->points.size(); ++j)
    {
      pt_.x = P.pts_->points[j].x*P.scale + P.center(0);
      pt_.y = P.pts_->points[j].y*P.scale + P.center(1);
      pt_.z = P.pts_->points[j].z*P.scale + P.center(2);
      calcloud.points.push_back(pt_);
    }
    for (int k=0; k<(int)P.normals_->points.size(); ++k)
    {
      n_.normal_x = P.normals_->points[k].normal_x;
      n_.normal_y = P.normals_->points[k].normal_y;
      n_.normal_z = P.normals_->points[k].normal_z;
      o_.normal_x = vset(k,0);
      o_.normal_y = vset(k,1);
      o_.normal_z = vset(k,2);
      normals.points.push_back(n_);
      orientations.points.push_back(o_);
    }
    for (int j=0; j<pset.rows(); ++j)
    {
      pt_.x = pset(j,0)*P.scale + P.center(0);
      pt_.y = pset(j,1)*P.scale + P.center(1);
      pt_.z = pset(j,2)*P.scale + P.center(2);
      drosa.push_back(pt_);
    }

    // ! Debug
    pcl::PointCloud<pcl::PointXYZ> vertex_debug;
    pcl::PointXYZ vertex_debug_pt;
    for (int i=0; i<(int)skeleton_ver_cloud->size(); ++i)
    {
      vertex_debug_pt.x = skeleton_ver_cloud->points[i].x*P.scale + P.center(0);
      vertex_debug_pt.y = skeleton_ver_cloud->points[i].y*P.scale + P.center(1);
      vertex_debug_pt.z = skeleton_ver_cloud->points[i].z*P.scale + P.center(2);
      vertex_debug.points.push_back(vertex_debug_pt);
    }
    Eigen::Vector3d debug_cutpt;
    debug_cutpt << vertex_debug.points[0].x, vertex_debug.points[0].y, vertex_debug.points[0].z;
    vis_utils_->publishCheckNeigh(debug_cutpt, vertex_debug, adj_before_collapse);
    // ! Debug

    vis_utils_->publishSurface(pcloud);
    vis_utils_->publishSurfaceNormal(calcloud, normals);
    vis_utils_->publishROSAOrientation(calcloud, orientations);
    vis_utils_->publish_dROSA(drosa);
    vis_utils_->publish_lineextract_vis(P.skelver, P.skeladj);
    vis_utils_->publish_recenter_vis(P.vertices_scale, P.edges, P.realVertices);
    vis_utils_->publishSubSpace(P.sub_space_scale);
    vis_utils_->publish_decomposition(P.vertices_scale, P.branches, P.mainDirBranches, P.centroidBranches);
    }
  }
  /* read point cloud input */
  void ROSA_main::pcloud_read_off()
  {
    P.pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    P.normals_.reset(new pcl::PointCloud<pcl::Normal>);

    pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd, *P.pts_);

    pcd_size_ = P.pts_->points.size();
    P.pts_mat.resize(pcd_size_, 3);
    P.nrs_mat.resize(pcd_size_, 3);
    for (int i=0; i<pcd_size_; ++i)
    {
      P.pts_mat(i,0) = P.pts_->points[i].x; P.pts_mat(i,1) = P.pts_->points[i].y; P.pts_mat(i,2) = P.pts_->points[i].z;
    }

    if (ground == true)
    {
      P.ground_pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointXYZ min;
      pcl::PointXYZ max;
      pcl::getMinMax3D(*P.pts_,min,max);
      groundHeight = min.z;
      
      // Snow Fall for pseudo ground
      pcl::PointXYZ SnowFallPt;
      for (auto p:P.pts_->points)
      {
        SnowFallPt.x = p.x; SnowFallPt.y = p.y; SnowFallPt.z = groundHeight-0.01;
        P.ground_pts_->points.push_back(SnowFallPt);
      }

      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(P.ground_pts_);
      vg.setLeafSize(pt_downsample_voxel_size, pt_downsample_voxel_size, pt_downsample_voxel_size);
      vg.filter(*P.ground_pts_);
      
      int size = P.pts_->points.size();
      int dsize = floor(0.1*(double)size);
      pcl::RandomSample<pcl::PointXYZ> rs;
      rs.setInputCloud(P.ground_pts_);
      rs.setSample(dsize);
      rs.filter(*P.ground_pts_);
    }
  }
  /* calculate mahalanobis length */
  double ROSA_main::mahalanobis_leth(pcl::PointXYZ& p1, pcl::Normal& v1, pcl::PointXYZ& p2, pcl::Normal& v2, double& r)
  {
    double Fs = 2.0, k = 0.0, dist, vec_dot, w;
    Eigen::Vector3d p1_, p2_, v1_, v2_;
    p1_ << p1.x, p1.y, p1.z; p2_ << p2.x, p2.y, p2.z;
    v1_ << v1.normal_x, v1.normal_y, v1.normal_z; v2_ << v2.normal_x, v2.normal_y, v2.normal_z;

    dist = (p1_-p2_ + Fs*((p1_-p2_).dot(v1_))*v1_).norm();
    dist = dist/r;
    if (dist <= 1)
      k=2*pow(dist,3)-3*pow(dist,2)+1;
    
    vec_dot = v1_.dot(v2_);
    w = k*pow(max(0.0, vec_dot),2);
    
    return w;
  }
  /* calculate mahalanobis adj matrix */
  void ROSA_main::pcloud_adj_matrix_mahalanobis(double& r_range)
  {
    auto adj_t1 = std::chrono::high_resolution_clock::now();

    P.neighs.clear();
    P.neighs.resize(MAHADJ.rows());

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(P.pts_);
    /* temp variable */
    pcl::PointXYZ search_point, p1, p2;
    pcl::Normal v1, v2;
    vector<int> indxs;
	  vector<float> radius_squared_distance;
    double weight1, weight2, weight;
    vector<vector<int>> pt_neighs_idx;
    for (int i=0; i<pcd_size_; ++i)
    {
      vector<int>().swap(indxs);
      vector<float>().swap(radius_squared_distance);
      p1 = P.pts_->points[i];
      v1 = P.normals_->points[i];
      search_point = P.pts_->points[i];
      tree.radiusSearch(search_point, r_range, indxs, radius_squared_distance);
      
      vector<int> temp_neighs;

      for (int j=0; j<(int)indxs.size(); ++j)
      {
        p2 = P.pts_->points[indxs[j]];
        v2 = P.normals_->points[indxs[j]];
        weight1 = mahalanobis_leth(p1, v1, p2, v2, r_range);
        weight2 = mahalanobis_leth(p2, v2, p1, v1, r_range);
        weight = min(weight1, weight2);
        if (weight > th_mah)
        {
          temp_neighs.push_back(indxs[j]);
        }
      }

      P.neighs[i] = temp_neighs;
    }
    auto adj_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> adj_ms = adj_t2 - adj_t1;
    double adj_time = (double)adj_ms.count();
    ROS_INFO("\033[32m[SSD] calculate adj matrix time = %lf ms.\033[32m", adj_time);
  }
  /* ROSA 1st calculation */
  void ROSA_main::rosa_drosa()
  {
    auto drosa_t1 = std::chrono::high_resolution_clock::now();

    Extra_Del ed_;
    rosa_initialize(P.pts_, P.normals_);

    vector<vector<int>>().swap(P.surf_neighs);
    vector<int> temp_surf(k_KNN);
    vector<float> nn_squared_distance(k_KNN);
    pcl::KdTreeFLANN<pcl::PointXYZ> surf_kdtree;
    surf_kdtree.setInputCloud(P.pts_);
    pcl::PointXYZ search_point_surf;
    for (int i=0; i<pcd_size_; ++i)
    {
      vector<int>().swap(temp_surf);
      vector<float>().swap(nn_squared_distance);
      search_point_surf = P.pts_->points[i];
      surf_kdtree.nearestKSearch(search_point_surf, k_KNN, temp_surf, nn_squared_distance);
      P.surf_neighs.push_back(temp_surf);
    }

    Eigen::Vector3d var_p, var_v, new_v;
    Eigen::MatrixXd indxs, extract_normals;
    for (int n=0; n<numiter_drosa; ++n)
    {
      Eigen::MatrixXd vnew = Eigen::MatrixXd::Zero(pcd_size_, 3);
      for (int pidx=0; pidx<pcd_size_; ++pidx)
      {
        var_p = pset.row(pidx); var_v = vset.row(pidx);
        indxs = rosa_compute_active_samples(pidx, var_p, var_v);
        extract_normals = ed_.rows_ext_M(indxs, P.nrs_mat);
        vnew.row(pidx) = compute_symmetrynormal(extract_normals).transpose(); new_v = vnew.row(pidx);

        if (extract_normals.rows() > 0)
          vvar(pidx,0) = symmnormal_variance(new_v, extract_normals);
        else
          vvar(pidx,0) = 0.0;
      }
      Eigen::MatrixXd offset(vvar.rows(), vvar.cols()); offset.setOnes(); offset = 0.00001*offset;
      vvar = (vvar.cwiseAbs2().cwiseAbs2()+offset).cwiseInverse();
      vset = vnew;
      /* smoothing */
      vector<int> surf_;
      Eigen::MatrixXi snidxs; 
      Eigen::MatrixXd snidxs_d, vset_ex, vvar_ex;
      for (int i=0; i<1; ++i)
      {
        for (int p=0; p<pcd_size_; ++p)
        {
          vector<int>().swap(surf_);
          surf_ = P.surf_neighs[p];
          snidxs.resize(surf_.size(), 1);
          snidxs = Eigen::Map<Eigen::MatrixXi>(surf_.data(), surf_.size(), 1);
          snidxs_d = snidxs.cast<double>(); vset_ex = ed_.rows_ext_M(snidxs_d, vset); vvar_ex = ed_.rows_ext_M(snidxs_d, vvar);
          vset.row(p) = symmnormal_smooth(vset_ex, vvar_ex);
        }
        vnew = vset;
      }
    }
    /* --- compute positions of ROSA --- */
    vector<int> poorIdx;
    pcl::PointCloud<pcl::PointXYZ>::Ptr goodPts (new pcl::PointCloud<pcl::PointXYZ>);
    map<Eigen::Vector3d, Eigen::Vector3d, Vector3dCompare> goodPtsPset;
    Eigen::Vector3d var_p_p, var_v_p, centroid;
    Eigen::MatrixXd indxs_p, extract_pts, extract_nrs;
    for (int pIdx=0; pIdx<pcd_size_; ++pIdx)
    {
      var_p_p = pset.row(pIdx); var_v_p = vset.row(pIdx);
      indxs_p = rosa_compute_active_samples(pIdx, var_p_p, var_v_p);

      //* Update Neighbors
      vector<int> temp_neigh;
      for (int p=0; p<(int)indxs_p.rows(); ++p)
        temp_neigh.push_back(indxs_p(p,0));
      P.neighs_new.push_back(temp_neigh);

      extract_pts = ed_.rows_ext_M(indxs_p, P.pts_mat); extract_nrs = ed_.rows_ext_M(indxs_p, P.nrs_mat);
      centroid = closest_projection_point(extract_pts, extract_nrs);
      if (abs(centroid(0)) < 1 && abs(centroid(1)) < 1 && abs(centroid(2)) < 1)
      {
        pset.row(pIdx) = centroid;
        pcl::PointXYZ goodPoint;
        Eigen::Vector3d goodPointP;
        goodPoint = P.pts_->points[pIdx];
        goodPointP(0) = P.pts_->points[pIdx].x; goodPointP(1) = P.pts_->points[pIdx].y; goodPointP(2) = P.pts_->points[pIdx].z;
        goodPts->points.push_back(goodPoint);
        goodPtsPset[goodPointP] = centroid;
      }
      else
      {
        poorIdx.push_back(pIdx);
      } 
    }
    
    rosa_tree.setInputCloud(goodPts);
    for (int pp=0; pp<(int)poorIdx.size(); ++pp)
    {
      int pair = 1;
      pcl::PointXYZ search_point;
      search_point.x = P.pts_->points[poorIdx[pp]].x; 
      search_point.y = P.pts_->points[poorIdx[pp]].y; 
      search_point.z = P.pts_->points[poorIdx[pp]].z;
      vector<int> pair_id(pair);
      vector<float> nn_squared_distance(pair);
      rosa_tree.nearestKSearch(search_point, pair, pair_id, nn_squared_distance);
      Eigen::Vector3d pairpos;
      pairpos(0) = goodPts->points[pair_id[0]].x;
      pairpos(1) = goodPts->points[pair_id[0]].y;
      pairpos(2) = goodPts->points[pair_id[0]].z;
      Eigen::Vector3d goodrp = goodPtsPset.find(pairpos)->second;
      pset.row(poorIdx[pp]) = goodrp;
    }

    dpset = pset;

    auto drosa_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> drosa_ms = drosa_t2 - drosa_t1;
    double drosa_time = (double)drosa_ms.count();
    ROS_INFO("\033[32m[SSD] drosa estimation time = %lf ms.\033[32m", drosa_time);
  }
  /* ROSA 2nd calculation */
  void ROSA_main::rosa_dcrosa()
  { 
    auto dcrosa_t1 = std::chrono::high_resolution_clock::now();
    
    Extra_Del ed_dc;
    for (int n=0; n<numiter_dcrosa; ++n)
    {
      Eigen::MatrixXi int_nidxs; 
      Eigen::MatrixXd newpset, indxs, extract_neighs; newpset.resize(pcd_size_, 3);
      for (int i=0; i<pcd_size_; ++i)
      {
        if (P.neighs[i].size() > 0)
        {
          int_nidxs = Eigen::Map<Eigen::MatrixXi>(P.neighs[i].data(), P.neighs[i].size(), 1);
          indxs = int_nidxs.cast<double>();
          extract_neighs = ed_dc.rows_ext_M(indxs, pset);
          newpset.row(i) = extract_neighs.colwise().mean();
        }
        else
        {
          newpset.row(i) = pset.row(i);
        }
      }
      pset = newpset;

      // ! /* shrinking */
      pcl::PointCloud<pcl::PointXYZ>::Ptr pset_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pset_cloud->width = pset.rows(); 
      pset_cloud->height = 1;
      pset_cloud->points.resize(pset_cloud->width * pset_cloud->height);
      for (size_t i = 0; i < pset_cloud->points.size(); ++i)
      {
        pset_cloud->points[i].x = pset(i, 0);
        pset_cloud->points[i].y = pset(i, 1);
        pset_cloud->points[i].z = pset(i, 2);
      }

      pcl::KdTreeFLANN<pcl::PointXYZ> pset_tree;
      pset_tree.setInputCloud(pset_cloud);
      
      // calculate confidence
      Eigen::VectorXd conf = Eigen::VectorXd::Zero(pset.rows());
      Eigen::MatrixXd newpset2;
      newpset2.resize(pcd_size_, 3);
      newpset2 = pset;
      double CONFIDENCE_TH = 0.5;

      for (int i=0; i<(int)pset.rows(); ++i)
      {
        vector<int> pointIdxNKNSearch(k_KNN);
        vector<float> pointNKNSquaredDistance(k_KNN);
        pset_tree.nearestKSearch(pset_cloud->points[i], k_KNN, pointIdxNKNSearch, pointNKNSquaredDistance);

        Eigen::MatrixXd neighbors(k_KNN, 3);
        for (int j=0; j<k_KNN; ++j)
          neighbors.row(j) = pset.row(pointIdxNKNSearch[j]);

        Eigen::Vector3d local_mean = neighbors.colwise().mean();
        neighbors.rowwise() -= local_mean.transpose();

        BDCSVD<Eigen::MatrixXd> svd(neighbors, ComputeThinU | ComputeThinV);
        conf(i) = svd.singularValues()(0) / svd.singularValues().sum();
        // compute linear projection
        if (conf(i) < CONFIDENCE_TH) continue;

        newpset2.row(i) = svd.matrixU().col(0).transpose() * ( svd.matrixU().col(0) * (pset.row(i) - local_mean.transpose()) ) + local_mean.transpose();
      }

      pset = newpset2;
      // ! /* shrinking */
    }

    auto dcrosa_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> dcrosa_ms = dcrosa_t2 - dcrosa_t1;
    double dcrosa_time = (double)dcrosa_ms.count();
    ROS_INFO("\033[32m[SSD] dcrosa estimation time = %lf ms.\033[32m", dcrosa_time);
  }
  /* graph extract */
  void ROSA_main::rosa_lineextract()
  {
    auto line_t1 = std::chrono::high_resolution_clock::now();

    Extra_Del ed_le;
    int outlier = 2;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pset_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ pset_pt;
    Eigen::MatrixXi bad_sample = Eigen::MatrixXi::Zero(pcd_size_, 1);
    vector<int> indxs;
	  vector<float> radius_squared_distance;
    pcl::PointXYZ search_point;
    Eigen::MatrixXi int_nidxs; Eigen::MatrixXd nIdxs, extract_corresp;

    for (int i=0; i<pcd_size_; ++i)
    {
      if ((int)P.neighs[i].size() <= outlier)
        bad_sample(i,0) = 1;
    }
    for (int j=0; j<pset.rows(); ++j)
    {
      pset_pt.x = pset(j,0); pset_pt.y = pset(j,1); pset_pt.z = pset(j,2); 
      pset_cloud->points.push_back(pset_pt);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(pset_cloud);
    P.skelver.resize(0,3);
    Eigen::MatrixXd mindst = Eigen::MatrixXd::Constant(pcd_size_,1,std::numeric_limits<double>::quiet_NaN());
    P.corresp = Eigen::MatrixXd::Constant(pcd_size_, 1, -1);
    for (int k=0; k<pcd_size_; ++k)
    {
      if (P.corresp(k,0) != -1) continue;

      mindst(k,0) = 1e8;
      while (!((P.corresp.array() != -1).all()))
      {
        int maxIdx = argmax_eigen(mindst);
        if (mindst(maxIdx,0) == 0) break;
        if (!std::isnan(mindst(maxIdx,0)) && mindst(maxIdx,0) == 0) break;

        vector<int>().swap(indxs);
        vector<float>().swap(radius_squared_distance);
        search_point.x = pset(maxIdx,0); search_point.y = pset(maxIdx,1); search_point.z = pset(maxIdx,2);
        tree.radiusSearch(search_point, sample_radius, indxs, radius_squared_distance);

        int_nidxs = Eigen::Map<Eigen::MatrixXi>(indxs.data(), indxs.size(), 1);
        nIdxs = int_nidxs.cast<double>();
        extract_corresp = ed_le.rows_ext_M(nIdxs, P.corresp);
        if ((extract_corresp.array() != -1).all())
        {
          mindst(maxIdx,0) = 0; continue;
        }

        P.skelver.conservativeResize(P.skelver.rows()+1, P.skelver.cols());
        P.skelver.row(P.skelver.rows()-1) = pset.row(maxIdx);
        for (int z=0; z<(int)indxs.size(); ++z)
        {
          if (std::isnan(mindst(indxs[z],0)) || mindst(indxs[z],0) > radius_squared_distance[z])
          {
            mindst(indxs[z],0) = radius_squared_distance[z];
            P.corresp(indxs[z], 0) = P.skelver.rows()-1;
          }
        }
      }
    }

    skeleton_ver_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ pt_vertex;
    for (int r=0; r<(int)P.skelver.rows(); ++r)
    {
      pt_vertex.x = P.skelver(r,0); pt_vertex.y = P.skelver(r,1); pt_vertex.z = P.skelver(r,2);
      skeleton_ver_cloud->points.push_back(pt_vertex);
    }
    
    int dim = P.skelver.rows();

    Eigen::MatrixXi Adj; Adj = Eigen::MatrixXi::Zero(dim, dim);
    vector<int> temp_surf(k_KNN);
    vector<int> good_neighs;

    for (int pIdx=0; pIdx<pcd_size_; ++pIdx)
    {
      if ((int)P.neighs[pIdx].size() <= outlier) continue;

      temp_surf.clear();
      good_neighs.clear();
      temp_surf = P.surf_neighs[pIdx];
      for (int ne=0; ne<(int)temp_surf.size(); ++ne)
      {
        if (bad_sample(temp_surf[ne],0) == 0)
          good_neighs.push_back(temp_surf[ne]);
      }

      if (P.corresp(pIdx,0) == -1) continue;
      for (int nidx=0; nidx<(int)good_neighs.size(); ++nidx)
      {
        if (P.corresp(good_neighs[nidx],0) == -1) continue;
        Adj((int)P.corresp(pIdx,0), (int)P.corresp(good_neighs[nidx],0)) = 1;
        Adj((int)P.corresp(good_neighs[nidx],0), (int)P.corresp(pIdx,0)) = 1;
      }
    }

    adj_before_collapse.resize(Adj.rows(), Adj.cols());
    adj_before_collapse = Adj;

    /* edge collapse */
    vector<int> ec_neighs; 
    Eigen::MatrixXd edge_rows; edge_rows.resize(2,3);
    while (1)
    {
      int tricount = 0;
      Eigen::MatrixXi skeds; skeds.resize(0,2);
      Eigen::MatrixXd skcst; skcst.resize(0,1);
      for (int i=0; i<P.skelver.rows(); ++i)
      {
        ec_neighs.clear();
        for(int col=0; col < Adj.cols(); ++col) 
        {
          if (Adj(i,col) == 1 && col>i) 
		        ec_neighs.push_back(col); 
        }
        std::sort(ec_neighs.begin(), ec_neighs.end());

        for (int j=0; j<(int)ec_neighs.size(); ++j)
        {
          for (int k=j+1; k<(int)ec_neighs.size(); ++k)
          {
            if (Adj(ec_neighs[j], ec_neighs[k]) == 1)
            {
              tricount++;
              skeds.conservativeResize(skeds.rows()+1, skeds.cols()); 
              skeds(skeds.rows()-1, 0)=i; 
              skeds(skeds.rows()-1, 1)=ec_neighs[j];

              skcst.conservativeResize(skcst.rows()+1, skcst.cols()); 
              skcst(skcst.rows()-1, 0)=(P.skelver.row(i)-P.skelver.row(ec_neighs[j])).norm();
              
              skeds.conservativeResize(skeds.rows()+1, skeds.cols()); 
              skeds(skeds.rows()-1, 0)=ec_neighs[j];
              skeds(skeds.rows()-1, 1)=ec_neighs[k];
              
              skcst.conservativeResize(skcst.rows()+1, skcst.cols()); 
              skcst(skcst.rows()-1, 0)=(P.skelver.row(ec_neighs[j])-P.skelver.row(ec_neighs[k])).norm();
              
              skeds.conservativeResize(skeds.rows()+1, skeds.cols()); 
              skeds(skeds.rows()-1, 0)=ec_neighs[k]; 
              skeds(skeds.rows()-1, 1)=i;
              
              skcst.conservativeResize(skcst.rows()+1, skcst.cols()); 
              skcst(skcst.rows()-1, 0)=(P.skelver.row(ec_neighs[k])-P.skelver.row(i)).norm();
            }
          }
        }
      }

      if (tricount == 0) break;

      Eigen::MatrixXd::Index minRow, minCol;
      skcst.minCoeff(&minRow,&minCol);
      int idx = minRow;
      Eigen::Vector2i edge = skeds.row(idx);
      
      edge_rows.row(0) = P.skelver.row(edge(0)); edge_rows.row(1) = P.skelver.row(edge(1));
      P.skelver.row(edge(0)) = edge_rows.colwise().mean();
      P.skelver.row(edge(1)).setConstant(std::numeric_limits<double>::quiet_NaN());

      for (int k=0; k<Adj.rows(); ++k)
      {
        if (Adj(edge(1),k) == 1)
        {
          Adj(edge(0),k) = 1; Adj(k,edge(0)) = 1;
        }
      }

      Adj.row(edge(1)) = Eigen::MatrixXi::Zero(1,Adj.cols());
      Adj.col(edge(1)) = Eigen::MatrixXi::Zero(Adj.rows(),1);
      for (int r=0; r<P.corresp.rows(); ++r)
      {
        if (P.corresp(r,0) == (double)edge(1)) P.corresp(r,0) = (double)edge(0);
      }
    }

    P.skeladj = Adj;

    auto line_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> line_ms = line_t2 - line_t1;
    double line_time = (double)line_ms.count();
    ROS_INFO("\033[32m[SSD] line extraction time = %lf ms.\033[32m", line_time);
  }
  /* recentering */
  void ROSA_main::rosa_recenter()
  {
    auto recenter_t1 = std::chrono::high_resolution_clock::now();
    
    Extra_Del ed_rr;
    vector<int> deleted_vertice_idxs, idxs;
    Eigen::MatrixXi ne_idxs, d_idxs; Eigen::MatrixXd ne_idxs_d, d_idxs_d, extract_pts, extract_nrs;
    Eigen::Vector3d proj_center, eucl_center, fuse_center;

    Eigen::MatrixXd temp_skelver, temp_skeladj_d, temp_skeladj_d2;
    
    for (int i=0; i<P.skelver.rows(); ++i)
    {
      idxs.clear();
      for (int j=0; j<P.corresp.rows(); ++j)
      {
        if (P.corresp(j,0) == (double)i) idxs.push_back(j);
      }

      if (idxs.size() < 3) deleted_vertice_idxs.push_back(i);
      else
      {
        ne_idxs = Eigen::Map<Eigen::MatrixXi>(idxs.data(), idxs.size(), 1);
        ne_idxs_d = ne_idxs.cast<double>();
        extract_pts = ed_rr.rows_ext_M(ne_idxs_d, P.pts_mat); extract_nrs = ed_rr.rows_ext_M(ne_idxs_d, P.nrs_mat);
        proj_center = closest_projection_point(extract_pts, extract_nrs);
        if (abs(proj_center(0)) < 1 && abs(proj_center(1)) < 1 && abs(proj_center(2)) < 1)
        {
          eucl_center = extract_pts.colwise().mean();
          fuse_center = alpha_recenter*proj_center + (1-alpha_recenter)*eucl_center;
          P.skelver(i,0) = fuse_center(0); P.skelver(i,1) = fuse_center(1); P.skelver(i,2) = fuse_center(2);
        }
      }
    }
    int del_size = deleted_vertice_idxs.size();
    d_idxs = Eigen::Map<Eigen::MatrixXi>(deleted_vertice_idxs.data(), deleted_vertice_idxs.size(), 1);
    d_idxs_d = d_idxs.cast<double>();
    temp_skeladj_d = P.skeladj.cast<double>();
    Eigen::MatrixXd fill = Eigen::MatrixXd::Zero(del_size, P.skeladj.cols());

    temp_skelver = ed_rr.rows_del_M(d_idxs_d, P.skelver);
    temp_skeladj_d = ed_rr.rows_del_M(d_idxs_d, temp_skeladj_d);
    temp_skeladj_d2.resize(P.skeladj.cols(), P.skeladj.cols());
    temp_skeladj_d2 << temp_skeladj_d, fill;
    temp_skeladj_d2 = ed_rr.cols_del_M(d_idxs_d, temp_skeladj_d2);
    P.skelver = temp_skelver; P.skeladj = temp_skeladj_d2.block(0,0,temp_skeladj_d2.cols(),temp_skeladj_d2.cols()).cast<int>();
    /* save the graph */
    P.vertices = P.skelver;
    P.edges.resize(0,2);
    for (int i=0; i<P.skeladj.rows(); ++i)
    {
      for (int j=0; j<P.skeladj.cols(); ++j)
      {
        if (P.skeladj(i,j) == 1 && i < j)
        {
          P.edges.conservativeResize(P.edges.rows()+1,P.edges.cols());
          P.edges(P.edges.rows()-1, 0) = i; P.edges(P.edges.rows()-1, 1) = j;
        }
      }
    }

    auto recenter_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> recenter_ms = recenter_t2 - recenter_t1;
    double recenter_time = (double)recenter_ms.count();
    ROS_INFO("\033[32m[SSD] recentering time = %lf ms.\033[32m", recenter_time);
  }
  /* find joints and graph decomposition */
  void ROSA_main::graph_decomposition()
  {
    auto graph_t1 = std::chrono::high_resolution_clock::now();
    
    deque<int>().swap(P.joint);
    P.degrees.resize(P.skeladj.rows(), 1);
    int deg; Eigen::VectorXi temp_row; 
    list<int> temp_nodes;

    for (int i=0; i<P.skeladj.rows(); ++i)
    {
      list<int>().swap(temp_nodes);
      deg = 0;
      for (int j=0; j<P.skeladj.cols(); ++j)
      {
        if (P.skeladj(i,j) == 1 && i!=j)
        {
          temp_nodes.push_back(j);
          deg++;
        }
      }
      P.graph.push_back(temp_nodes);
      P.degrees(i,0) = deg;
      if (deg >=3)
        P.joint.push_back(i);
    }
    if ((int)P.joint.size() == 0)
    {
      for (int i=0; i<P.skeladj.rows(); ++i)
      {
        if (P.degrees(i,0) == 1)
        {
          P.joint.push_back(i);
          break;
        }
      }
    }
    /* Tree + DFS+decomposition */
    vector<vector<int>>().swap(P.branches);
    P.visited.resize(P.graph.size(), false);
    
    for (int j=0; j<(int)P.joint.size(); ++j)
      dfs(P.joint[j]);
    
    vector<vector<int>> temp_branches;
    for (int i=0; i<(int)P.branches.size(); ++i)
    {
      if ((int)P.branches[i].size() == 2 && P.branches[i].front() == P.branches[i].back())
        continue;
      else
        temp_branches.push_back(P.branches[i]);
    }

    vector<vector<int>>().swap(P.branches);
    P.branches = temp_branches;
    
    auto graph_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> graph_ms = graph_t2 - graph_t1;
    double graph_time = (double)graph_ms.count();
    ROS_INFO("\033[32m[SSD] graph decomposition time = %lf ms.\033[32m", graph_time);
  }
  /* branch inner sub-decomposition */
  void ROSA_main::inner_decomposition()
  {
    auto inner_t1 = std::chrono::high_resolution_clock::now();
    
    vector<vector<int>> temp_branches;
    vector<vector<int>> inner_decomp;

    for (int i=0; i<(int)P.branches.size(); ++i)
    {
      if (P.branches[i].size() <= 2)
        temp_branches.push_back(P.branches[i]);
      else
      {
        inner_decomp = divide_branch(P.branches[i]);
        for (int j=0; j<(int)inner_decomp.size(); ++j)
          temp_branches.push_back(inner_decomp[j]);
      }
    }

    P.branches.clear();
    P.branches = temp_branches;

    vector<vector<int>> temp_branches_inner;
    for (int i=0; i<(int)P.branches.size(); ++i)
    {
      if ((int)P.branches[i].size() == 2 && P.branches[i].front() == P.branches[i].back())
        continue;
      else
        temp_branches_inner.push_back(P.branches[i]);
    }

    P.branches.clear();
    P.branches = temp_branches_inner;

    auto inner_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> inner_ms = inner_t2 - inner_t1;
    double inner_time = (double)inner_ms.count();
    ROS_INFO("\033[32m[SSD] inner decomposition time = %lf ms.\033[32m", inner_time);
  }
  /* merge these individual branches with small length */
  void ROSA_main::branch_merge()
  {
    auto bm_t1 = std::chrono::high_resolution_clock::now();
    
    P.visited.clear();
    P.visited.resize(P.branches.size(), false);
    vector<vector<int>> temp_branches; vector<int> merge_one;
    Eigen::Vector3d start, end;
    double norm;

    for (int i=0; i<(int)P.branches.size(); ++i)
    {
      start = P.vertices.row(P.branches[i].front());
      end = P.vertices.row(P.branches[i].back());
      norm = (start-end).norm();

      if (P.degrees(P.branches[i].front(),0)==1 && P.degrees(P.branches[i].back(),0)==1)
      {
        continue;
      }

      if (P.branches[i].size() == 2 && (P.degrees(P.branches[i].front()) == 1 || P.degrees(P.branches[i].back()) == 1) && norm <= length_lower)
      {
        P.visited[i] = true;
        cout << "2_element_Merge_id:" << i << endl;
        merge_one = merge_branch(P.branches[i]);
        temp_branches.push_back(merge_one);
        continue;
      }

      if (norm > length_lower)
        continue;
      else
      {
        if (P.degrees(P.branches[i].front()) > 1 && P.degrees(P.branches[i].back()) > 1)
        {
          ROS_WARN("Without Merging Intermediate Branch...");
          continue;
        }
        else
        {
          P.visited[i] = true;
          cout << "Merge_id:" << i << endl;
          merge_one = merge_branch(P.branches[i]);
          temp_branches.push_back(merge_one);
        }
      }
    }

    for (int j=0; j<(int)P.branches.size(); ++j)
    {
      if (P.visited[j] == false)
        temp_branches.push_back(P.branches[j]);
    }
    
    P.branches = temp_branches;

    vector<vector<int>> temp_branches_merge;
    for (int i=0; i<(int)P.branches.size(); ++i)
    {
      if ((int)P.branches[i].size() == 2 && P.branches[i].front() == P.branches[i].back())
        continue;
      else
        temp_branches_merge.push_back(P.branches[i]);
    }

    vector<vector<int>>().swap(P.branches);
    P.branches = temp_branches_merge;
    /* update graph */
    P.segments.clear();
    P.branch_seg_pairs.clear();
    seg_count=0;
    vector<int> indexer;
    vector<int> seg_set;
    for (auto x:P.branches)
    {
      vector<int>().swap(seg_set);
      for (int i=0; i<(int)x.size()-1; ++i)
      {
        vector<int>().swap(indexer);
        indexer.push_back(x[i]); indexer.push_back(x[i+1]);
        P.segments[seg_count] = indexer;
        seg_set.push_back(seg_count);
        seg_count++;
      }
      P.branch_seg_pairs.push_back(seg_set);
    }
    cout << "branch_merge->segments: " << seg_count << endl;
    cout << "branch_merge->pairs: " << P.branch_seg_pairs.size() << endl;

    auto bm_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> bm_ms = bm_t2 - bm_t1;
    double bm_time = (double)bm_ms.count();
    ROS_INFO("\033[32m[SSD] branch merge time = %lf ms.\033[32m", bm_time);
  }
  /* prune small unnecessary branches */
  void ROSA_main::prune_branches()
  {
    vector<vector<int>> updated_branches;
    bool prune_flag = false;
    for (auto x:P.branches)
    {
      if (x.size() == 2 && (P.degrees(x.front()) == 1 || P.degrees(x.back()) == 1))
      {
        prune_flag = prune(x);
        if (prune_flag)
        {
          continue;
        }
        else
          updated_branches.push_back(x);
        
      }
      else
        updated_branches.push_back(x);
    }

    P.branches = updated_branches;
    /* update graph */
    P.segments.clear();
    P.branch_seg_pairs.clear();
    seg_count=0;
    vector<int> indexer;
    vector<int> seg_set;
    for (auto x:P.branches)
    {
      vector<int>().swap(seg_set);
      for (int i=0; i<(int)x.size()-1; ++i)
      {
        vector<int>().swap(indexer);
        indexer.push_back(x[i]); indexer.push_back(x[i+1]);
        P.segments[seg_count] = indexer;
        seg_set.push_back(seg_count);
        seg_count++;
      }
      P.branch_seg_pairs.push_back(seg_set);
    }
    cout << "prune_branch->segments: " << seg_count << endl;
    cout << "prune_branch->pairs: " << P.branch_seg_pairs.size() << endl;
  }
  /* restore the original scale of input pcloud */
  void ROSA_main::restore_scale()
  {
    auto rs_t1 = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_sub_space (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_segment (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ ss_pt, sc_pt;

    for (auto ss:P.sub_space)
    {
      temp_sub_space.reset(new pcl::PointCloud<pcl::PointXYZ>);
      for (auto pt:ss->points)
      {
        ss_pt.x = pt.x*P.scale + P.center(0);
        ss_pt.y = pt.y*P.scale + P.center(1);
        ss_pt.z = pt.z*P.scale + P.center(2);
        temp_sub_space->points.push_back(ss_pt);
      }
      P.sub_space_scale.push_back(temp_sub_space);
    }
    
    Eigen::VectorXd vertex; vertex.resize(P.vertices.cols());
    P.vertices_scale.resize(P.vertices.rows(), P.vertices.cols());
    for (int k=0; k<(int)P.vertices_scale.rows(); ++k)
    {
      vertex(0) = P.vertices.row(k)(0)*P.scale + P.center(0);
      vertex(1) = P.vertices.row(k)(1)*P.scale + P.center(1);
      vertex(2) = P.vertices.row(k)(2)*P.scale + P.center(2);
      P.vertices_scale.row(k) = vertex;
    }

    /* update mainDir and centroid */
    P.mainDirBranches.clear();
    P.centroidBranches.clear();
    for (int i=0; i<(int)P.branches.size(); ++i)
    {
      vector<int> start_end_; start_end_.resize(2);
      start_end_[0] = P.branches[i].front(); start_end_[1] = P.branches[i].back();
      P.startendBranches.push_back(start_end_);
      Eigen::MatrixXd temp_vertices;
      temp_vertices.resize(P.branches[i].size(), P.vertices_scale.cols());
      for (int j=0; j<(int)P.branches[i].size(); ++j)
        temp_vertices.row(j) = P.vertices_scale.row(P.branches[i][j]);
      /* centroid & main direction */
      Eigen::Vector3d centroid, main_dir;
      centroid = temp_vertices.colwise().mean();
      if (temp_vertices.rows() < 3)
        main_dir = (temp_vertices.row(1) - temp_vertices.row(0)).normalized();
      else
        main_dir = PCA(temp_vertices);
      P.mainDirBranches.push_back(main_dir);
      P.centroidBranches.push_back(centroid);
    }

    auto rs_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> rs_ms = rs_t2 - rs_t1;
    double rs_time = (double)rs_ms.count();
    ROS_INFO("\033[32m[SSD] restore scale time = %lf ms.\033[32m", rs_time);
  }
  /* calculate inner dist in physical world */
  void ROSA_main::cal_inner_dist()
  {
    auto cid_t1 = std::chrono::high_resolution_clock::now();
    
    int seg_id = -1;
    Eigen::Vector3d start, end, line_vec, in_pt;

    for (const auto& seg_pts:P.seg_clouds_scale)
    {
      double max_dist = -1.0;
      double min_dist = numeric_limits<double>::max();
      double avg_dist = 0.0, dist = 0.0;

      seg_id = seg_pts.first;
      start = P.scale * P.vertices.row(P.segments[seg_id][0]);
      end = P.scale * P.vertices.row(P.segments[seg_id][1]);
      line_vec = (end - start).normalized();

      for (int i=0; i<(int)seg_pts.second->points.size(); ++i)
      {
        in_pt(0) = seg_pts.second->points[i].x; in_pt(1) = seg_pts.second->points[i].y; in_pt(2) = seg_pts.second->points[i].z;
        dist = distance_point_line(in_pt, start, line_vec);
        avg_dist += dist;
        max_dist = max(max_dist, dist);
        min_dist = min(min_dist, dist);
      }
      avg_dist = avg_dist/seg_pts.second->points.size();
      P.inner_dist_set[seg_id] = avg_dist;
    }

    auto cid_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> cid_ms = cid_t2 - cid_t1;
    double cid_time = (double)cid_ms.count();
    ROS_INFO("\033[32m[SSD] calculate inner dist time = %lf ms.\033[32m", cid_time);
  }
  /* distribute original point cloud into each sub-space and segment */
  void ROSA_main::distribute_ori_cloud()
  {
    auto doc_t1 = std::chrono::high_resolution_clock::now();

    /* construct pt_seg pairs Vector3d --> seg_id */
    int seg_id = -1;
    Eigen::Vector3d temp_pt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ search_point;

    /* skeleton discreted points */
    map<int, int> skeletonSegID;
    Eigen::Vector3d start, end;
    int sk_id = 0;
    for (int j=0; j<seg_count; ++j)
    {
      seg_id = j;
      pcl::PointCloud<pcl::PointXYZ>::Ptr zerocloud (new pcl::PointCloud<pcl::PointXYZ>);
      P.seg_clouds_scale[seg_id] = zerocloud;
      start = P.vertices_scale.row(P.segments[j][0]); end = P.vertices_scale.row(P.segments[j][1]);
      Eigen::Vector3d dir = end-start;
      double norm = dir.norm();
      Eigen::Vector3d p_cut, v_cut;
      v_cut = dir.normalized();
      int num = norm/delta;
      
      Eigen::Vector3d temp_inpt;
      pcl::PointXYZ pt;
      for (int i=0; i<(num+1); i=i+1)
      {
        temp_inpt = start + i*delta*v_cut;
        skeletonSegID[sk_id] = seg_id;
        pt.x = temp_inpt(0); pt.y = temp_inpt(1); pt.z = temp_inpt(2);
        target->points.push_back(pt);
        sk_id++;
      }
    }
    /* random down sample */
    if (ground == true)
      *P.ori_pts_ = *P.ori_pts_ + *P.ground_pts_;
    if ((int)P.ori_pts_->points.size() > ori_num)
    {
      pcl::RandomSample<pcl::PointXYZ> rs;
      rs.setInputCloud(P.ori_pts_);
      rs.setSample(ori_num);
      rs.filter(*P.ori_pts_);
    }
    /* nearest distribution principle */
    pcl::KdTreeFLANN<pcl::PointXYZ> distri_tree;
    distri_tree.setInputCloud(target);
    vector<int> nearest(1);
    vector<float> nn_squared_distance(1);
    // Eigen::Vector3d find_target;
    int affi_id = -1;

    for (auto ori_pt:P.ori_pts_->points)
    {
      vector<int>().swap(nearest);
      vector<float>().swap(nn_squared_distance);
      search_point = ori_pt;
      distri_tree.nearestKSearch(search_point, 1, nearest, nn_squared_distance);
      affi_id = skeletonSegID.find(nearest[0])->second;
      /* add point into each segment */
      P.seg_clouds_scale[affi_id]->points.push_back(search_point);
    }
    /* Update sub_space_scale */
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>().swap(P.sub_space_scale);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sub_space_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (auto set:P.branch_seg_pairs)
    {
      sub_space_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      for (auto seg_id:set)
      {
        *sub_space_cloud += *P.seg_clouds_scale.find(seg_id)->second;
      }
      P.sub_space_scale.push_back(sub_space_cloud);
    }

    auto doc_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> doc_ms = doc_t2 - doc_t1;
    double doc_time = (double)doc_ms.count();
    ROS_INFO("\033[32m[SSD] distribute original cloud time = %lf ms.\033[32m", doc_time);
  }
  /* store real scale ROSA graph */
  void ROSA_main::storeRealGraph()
  {
    auto srg_t1 = std::chrono::high_resolution_clock::now();    

    P.outputVertices = P.vertices_scale;
    P.outputEdges = P.edges;
    
    vector<int> effectiveIdxs;
    for (int i=0; i<(int)P.outputEdges.rows(); ++i)
    {
      for (int j=0; j<(int)P.outputEdges.cols(); ++j)
        effectiveIdxs.push_back(P.outputEdges(i,j));
    }
    set<int> st(effectiveIdxs.begin(), effectiveIdxs.end());
    Eigen::MatrixXd temp_vertices; 
    temp_vertices.resize(st.size(), P.outputVertices.cols());

    vector<int> newVerticesIdxs;
    for (const auto& value : st)
      newVerticesIdxs.push_back(value);
    for (int k=0; k<(int)newVerticesIdxs.size(); ++k)
      temp_vertices.row(k) = P.outputVertices.row(newVerticesIdxs[k]);
    
    P.realVertices = temp_vertices;

    auto srg_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> srg_ms = srg_t2 - srg_t1;
    double srg_time = (double)srg_ms.count();
    ROS_INFO("\033[32m[SSD] store real graph time = %lf ms.\033[32m", srg_time);
  }

  void ROSA_main::rosa_initialize(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals)
  {
    int psize = (int)cloud->points.size();
    for (int i=0; i<psize; ++i)
    {
      pset(i,0) = cloud->points[i].x; pset(i,1) = cloud->points[i].y; pset(i,2) = cloud->points[i].z;
    }
    Eigen::Matrix3d M;
    Eigen::Vector3d normal_v;
    for (int j=0; j<psize; ++j)
    {
      normal_v(0) = normals->points[j].normal_x; normal_v(1) = normals->points[j].normal_y; normal_v(2) = normals->points[j].normal_z;
      M = create_orthonormal_frame(normal_v);
      vset.row(j) = M.row(1);
    }
  }

  void ROSA_main::pcloud_isoncut(Eigen::Vector3d& p_cut, Eigen::Vector3d& v_cut, vector<int>& isoncut, double*& datas, int& size)
  {
    DataWrapper data;
    data.factory(datas, size);
    vector<double> p(3); p[0] = p_cut(0); p[1] = p_cut(1); p[2] = p_cut(2);
    vector<double> n(3); n[0] = v_cut(0); n[1] = v_cut(1); n[2] = v_cut(2);
    distance_query(data, p, n, delta, isoncut);
  }

  void ROSA_main::distance_query(DataWrapper& data, const vector<double>& Pp, const vector<double>& Np, double delta, vector<int>& isoncut)
  {
    vector<double> P(3);
	  for (int pIdx=0; pIdx < data.length(); pIdx++) 
    {
      // retrieve current point
      data( pIdx, P );
      // check distance
      if( fabs( Np[0]*(Pp[0]-P[0]) + Np[1]*(Pp[1]-P[1]) + Np[2]*(Pp[2]-P[2]) ) < delta )
      {
        isoncut[pIdx] = 1;
      }
	  }
  }
  
  void ROSA_main::normal_estimation()
  {
    ROS_ERROR("Normal Estimation...");
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(P.pts_);
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(ne_KNN);
    ne.compute(*cloud_normals);

    P.normals_.reset(new pcl::PointCloud<pcl::Normal>);
    P.normals_ = cloud_normals;
  }
  /* Only scaling up with norm_scale-fold */
  void ROSA_main::normalize()
  { 
    auto norm_t1 = std::chrono::high_resolution_clock::now();

    /* store original points */
    P.ori_pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*P.pts_, *P.ori_pts_);

    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D(*P.pts_,min,max);
    double x_scale, y_scale, z_scale, max_scale;
    x_scale = max.x-min.x; y_scale = max.y-min.y; z_scale = max.z-min.z; 
    if (x_scale >= y_scale)
      max_scale = x_scale;
    else 
      max_scale = y_scale;
    if (max_scale < z_scale)
      max_scale = z_scale;
    
    norm_scale = max_scale;
    pcl::compute3DCentroid(*P.pts_, centroid);

    for (int i=0; i<(int)P.pts_->points.size(); ++i)
    {
      P.pts_->points[i].x = (P.pts_->points[i].x-centroid(0))/max_scale;
      P.pts_->points[i].y = (P.pts_->points[i].y-centroid(1))/max_scale;
      P.pts_->points[i].z = (P.pts_->points[i].z-centroid(2))/max_scale;
    }

    /* normal estimation */
    normal_estimation();

    P.cloud_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr temp_all(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr temp_all_down(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*P.pts_, *P.normals_, *temp_all_down);
    pcl::concatenateFields (*P.pts_, *P.normals_, *P.cloud_with_normals);

    /* voxel grid downsample */
    int upper_size = estNum;
    int curPtSize = (int)temp_all_down->points.size();
    while (curPtSize > upper_size)
    {
      pcl::VoxelGrid<pcl::PointNormal> vg;
      vg.setInputCloud(temp_all_down);
      vg.setLeafSize(pt_downsample_voxel_size, pt_downsample_voxel_size, pt_downsample_voxel_size);
      vg.filter(*temp_all);
      
      curPtSize = (int)temp_all->points.size();
      if (curPtSize > upper_size)
      {
        pt_downsample_voxel_size += 0.002;
      }
    }

    pcl::VoxelGrid<pcl::PointNormal> vgf;
    vgf.setInputCloud(P.cloud_with_normals);
    vgf.setLeafSize(pt_downsample_voxel_size, pt_downsample_voxel_size, pt_downsample_voxel_size);
    vgf.filter(*P.cloud_with_normals);
    curPtSize = (int)P.cloud_with_normals->points.size();

    th_mah = 0.1*Radius; //* Determination relationship of constructing gragh 
    delta = pt_downsample_voxel_size; //* keep plane thickness and downsample size equal

    ROS_ERROR("Final downsample voxel size: %f, Cloud size: %d", pt_downsample_voxel_size, (int)P.cloud_with_normals->points.size());

    pcd_size_ = P.cloud_with_normals->points.size();
    P.pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    P.normals_.reset(new pcl::PointCloud<pcl::Normal>);
    P.pts_mat.resize(pcd_size_, 3);
    P.nrs_mat.resize(pcd_size_, 3);
    pcl::PointXYZ pt;
    pcl::Normal normal;

    for (int i=0; i<pcd_size_; ++i)
    {
      pt.x = P.cloud_with_normals->points[i].x; pt.y = P.cloud_with_normals->points[i].y; pt.z = P.cloud_with_normals->points[i].z; 
      normal.normal_x = -P.cloud_with_normals->points[i].normal_x; 
      normal.normal_y = -P.cloud_with_normals->points[i].normal_y; 
      normal.normal_z = -P.cloud_with_normals->points[i].normal_z;
      P.pts_->points.push_back(pt);
      P.normals_->points.push_back(normal);
      P.pts_mat(i,0) = P.pts_->points[i].x; P.pts_mat(i,1) = P.pts_->points[i].y; P.pts_mat(i,2) = P.pts_->points[i].z;
      P.nrs_mat(i,0) = P.normals_->points[i].normal_x; P.nrs_mat(i,1) = P.normals_->points[i].normal_y; P.nrs_mat(i,2) = P.normals_->points[i].normal_z;
    }

    auto norm_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> norm_ms = norm_t2 - norm_t1;
    double norm_time = (double)norm_ms.count();
    ROS_INFO("\033[32m[SSD] normalize time = %lf ms\033[32m", norm_time);
  } 
  /* orthonormal basis generation */
  Eigen::Matrix3d ROSA_main::create_orthonormal_frame(Eigen::Vector3d& v)
  {
    // ! /* random process for generating orthonormal basis */
    v = v/v.norm();
    double TH_ZERO = 1e-10;
    srand((unsigned)time(NULL));
    Eigen::Matrix3d M=Eigen::Matrix3d::Zero();
    M(0,0) = v(0); M(0,1) = v(1); M(0,2) = v(2); 
    Eigen::Vector3d new_vec, temp_vec;
    for (int i=1; i<3; ++i)
    {
      new_vec.setRandom();
      new_vec = new_vec/new_vec.norm();
      while (abs(1.0-v.dot(new_vec)) < TH_ZERO)
      {
        new_vec.setRandom();
        new_vec = new_vec/new_vec.norm();
      }
      for (int j=0; j<i; ++j)
      {
        temp_vec = (new_vec - new_vec.dot(M.row(j))*(M.row(j).transpose()));
        new_vec = temp_vec/temp_vec.norm();
      }
      M(i,0) = new_vec(0); M(i,1) = new_vec(1); M(i,2) = new_vec(2); 
    }

    return M;
  }

  Eigen::MatrixXd ROSA_main::rosa_compute_active_samples(int& idx, Eigen::Vector3d& p_cut, Eigen::Vector3d& v_cut)
  {
    Eigen::MatrixXd out_indxs(pcd_size_, 1);
    int out_size = 0;
    std::vector<int> isoncut(pcd_size_, 0);
    pcloud_isoncut(p_cut, v_cut, isoncut, P.datas, pcd_size_);
    
    std::vector<int> queue;
    queue.reserve(pcd_size_);
    queue.emplace_back(idx);

    int curr;
    while (!queue.empty())
    {
        curr = queue.back();
        queue.pop_back();
        isoncut[curr] = 2;
        out_indxs(out_size++, 0) = curr;

        for (size_t i = 0; i < P.neighs[curr].size(); ++i)
        {
            if (isoncut[P.neighs[curr][i]] == 1)
            {
                isoncut[P.neighs[curr][i]] = 3;
                queue.emplace_back(P.neighs[curr][i]);
            }
        }
    }

    out_indxs.conservativeResize(out_size, 1);

    return out_indxs;
  }

  Eigen::Vector3d ROSA_main::compute_symmetrynormal(Eigen::MatrixXd& local_normals)
  {
    Eigen::Matrix3d M; Eigen::Vector3d vec;
    double alpha = 0.0;
    int size = local_normals.rows();
    double Vxx, Vyy, Vzz, Vxy, Vyx, Vxz, Vzx, Vyz, Vzy;
    Vxx = (1.0+alpha)*local_normals.col(0).cwiseAbs2().sum()/size - pow(local_normals.col(0).sum(), 2)/pow(size, 2);
    Vyy = (1.0+alpha)*local_normals.col(1).cwiseAbs2().sum()/size - pow(local_normals.col(1).sum(), 2)/pow(size, 2);
    Vzz = (1.0+alpha)*local_normals.col(2).cwiseAbs2().sum()/size - pow(local_normals.col(2).sum(), 2)/pow(size, 2);
    Vxy = 2*(1.0+alpha)*(local_normals.col(0).cwiseProduct(local_normals.col(1))).sum()/size - 2*local_normals.col(0).sum()*local_normals.col(1).sum()/pow(size, 2);
    Vyx = Vxy;
    Vxz = 2*(1.0+alpha)*(local_normals.col(0).cwiseProduct(local_normals.col(2))).sum()/size - 2*local_normals.col(0).sum()*local_normals.col(2).sum()/pow(size, 2);
    Vzx = Vxz;
    Vyz = 2*(1.0+alpha)*(local_normals.col(1).cwiseProduct(local_normals.col(2))).sum()/size - 2*local_normals.col(1).sum()*local_normals.col(2).sum()/pow(size, 2);
    Vzy = Vyz;
    M << Vxx, Vxy, Vxz, Vyx, Vyy, Vyz, Vzx, Vzy, Vzz;

    BDCSVD<MatrixXd> svd(M, ComputeThinU | ComputeThinV);
    Eigen::Matrix3d U = svd.matrixU();
    vec = U.col(M.cols()-1);

    return vec;
  }

  double ROSA_main::symmnormal_variance(Eigen::Vector3d& symm_nor, Eigen::MatrixXd& local_normals)
  {
    Eigen::MatrixXd repmat; Eigen::VectorXd alpha;
    int num = local_normals.rows();
    repmat.resize(num,3);
    for (int i=0; i<num; ++i)
      repmat.row(i) = symm_nor;
    alpha = local_normals.cwiseProduct(repmat).rowwise().sum();
    int n = alpha.size(); double var;
    if (n>1)
      var = (n+1)*(alpha.squaredNorm()/(n+1) - alpha.mean()*alpha.mean())/n;
    else
    {
      var = alpha.squaredNorm()/(n+1) - alpha.mean()*alpha.mean();
    }

    return var;
  }

  Eigen::Vector3d ROSA_main::symmnormal_smooth(Eigen::MatrixXd& V, Eigen::MatrixXd& w)
  {
    Eigen::Matrix3d M; Eigen::Vector3d vec;
    double Vxx, Vyy, Vzz, Vxy, Vyx, Vxz, Vzx, Vyz, Vzy;
    Vxx = (w.cwiseProduct(V.col(0).cwiseAbs2())).sum();
    Vyy = (w.cwiseProduct(V.col(1).cwiseAbs2())).sum();
    Vzz = (w.cwiseProduct(V.col(2).cwiseAbs2())).sum();
    Vxy = (w.cwiseProduct(V.col(0)).cwiseProduct(V.col(1))).sum();
    Vyx = Vxy;
    Vxz = (w.cwiseProduct(V.col(0)).cwiseProduct(V.col(2))).sum();
    Vzx = Vxz;
    Vyz = (w.cwiseProduct(V.col(1)).cwiseProduct(V.col(2))).sum();
    Vzy = Vyz;
    M << Vxx, Vxy, Vxz, Vyx, Vyy, Vyz, Vzx, Vzy, Vzz;

    BDCSVD<MatrixXd> svd(M, ComputeThinU | ComputeThinV);
    Eigen::Matrix3d U = svd.matrixU();
    vec = U.col(0);

    return vec;
  }

  Eigen::Vector3d ROSA_main::closest_projection_point(Eigen::MatrixXd& P, Eigen::MatrixXd& V)
  {
    Eigen::Vector3d vec;
    Eigen::VectorXd Lix2, Liy2, Liz2;
    Lix2 = V.col(0).cwiseAbs2();
    Liy2 = V.col(1).cwiseAbs2();
    Liz2 = V.col(2).cwiseAbs2();

    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    Eigen::Vector3d B = Eigen::Vector3d::Zero();

    M(0,0) = (Liy2+Liz2).sum();
    M(0,1) = -(V.col(0).cwiseProduct(V.col(1))).sum();
    M(0,2) = -(V.col(0).cwiseProduct(V.col(2))).sum();
    B(0) = (P.col(0).cwiseProduct(Liy2 + Liz2)).sum() - (V.col(0).cwiseProduct(V.col(1)).cwiseProduct(P.col(1))).sum() - (V.col(0).cwiseProduct(V.col(2)).cwiseProduct(P.col(2))).sum();
    M(1,0) = -(V.col(1).cwiseProduct(V.col(0))).sum();
    M(1,1) = (Lix2 + Liz2).sum();
    M(1,2) = -(V.col(1).cwiseProduct(V.col(2))).sum();
    B(1) = (P.col(1).cwiseProduct(Lix2 + Liz2)).sum() - (V.col(1).cwiseProduct(V.col(0)).cwiseProduct(P.col(0))).sum() - (V.col(1).cwiseProduct(V.col(2)).cwiseProduct(P.col(2))).sum();
    M(2,0) = -(V.col(2).cwiseProduct(V.col(0))).sum();
    M(2,1) = -(V.col(2).cwiseProduct(V.col(1))).sum();
    M(2,2) = (Lix2 + Liy2).sum();
    B(2) = (P.col(2).cwiseProduct(Lix2 + Liy2)).sum() - (V.col(2).cwiseProduct(V.col(0)).cwiseProduct(P.col(0))).sum() - (V.col(2).cwiseProduct(V.col(1)).cwiseProduct(P.col(1))).sum();

    if (abs(M.determinant()) < 1e-3)
      vec << 1e8, 1e8, 1e8;
    else
      vec = M.inverse()*B;

    return vec;
  }

  int ROSA_main::argmax_eigen(Eigen::MatrixXd &x) 
  {
    Eigen::MatrixXd::Index maxRow, maxCol;
    x.maxCoeff(&maxRow,&maxCol);
    int idx = maxRow;
    return idx;
  }

  void ROSA_main::dfs(int& v)
  {
    list<int>::iterator it;
    P.visited[v] = true;
    
    int joint = v;
    bool exec_flag = false, push_flag = false;
    vector<int> branch; branch.push_back(joint);

    stack<int> tempStack;
    tempStack.push(v);
    while(!tempStack.empty())
    {
      v = tempStack.top();
      tempStack.pop();
      if (!P.visited[v])
      {
        exec_flag = false;
        if (branch.size() == 1)
        {
          if (v!=joint)
            exec_flag = ocr_node(v, P.graph[joint]);
          else
            exec_flag = true;
        }
        else
          exec_flag = true;

        if (exec_flag)
        {
          P.visited[v] = true;
          if (ocr_node(v, P.graph[branch.back()]))
          {
            branch.push_back(v);
            if (P.degrees(v,0) != 2)
            {
              P.branches.push_back(branch);
              vector<int>().swap(branch);
              branch.push_back(joint);
            }
            if (P.degrees(v,0) > 2)
              P.visited[v] = false;
          }
          else
          {
            branch.push_back(joint);
            P.branches.push_back(branch);
            vector<int>().swap(branch);
            branch.push_back(joint);
            branch.push_back(v);
          }
        }
      }
      
      push_flag = false;
      if (v==joint)
      {
        push_flag = true;
      }
      else
      {
        if (P.degrees(v,0)<=2)
          push_flag = true;
      }

      if (push_flag)
      {
        for (it = P.graph[v].begin(); it != P.graph[v].end(); it++)
        {
          if (!P.visited[*it])
          {
            tempStack.push(*it);
          }
        }
      }

      if (tempStack.empty() && (P.graph[v].front() == joint || P.graph[v].back() == joint) && (int)branch.size() > 2)
      {
        branch.push_back(v);
        branch.push_back(joint);
        P.branches.push_back(branch);
        vector<int>().swap(branch);
      }
    }
  }

  bool ROSA_main::ocr_node(int& n, list<int>& candidates)
  {
    bool flag = false;
    for (int& x:candidates)
    {
      if (n == x)
      {
        flag=true;
        break;
      }
    }

    return flag;
  }

  vector<vector<int>> ROSA_main::divide_branch(vector<int>& input_branch)
  {
    vector<vector<int>> divide_set;
    vector<int> temp_nodes;
    Eigen::Vector3d p1, p2, dir;
    Eigen::Vector3d p1_ori, p2_ori, dir_ori;
    Eigen::Vector3d p_end;
    double similarity, length;
    
    temp_nodes.push_back(input_branch[0]); temp_nodes.push_back(input_branch[1]);
    p1_ori = P.vertices.row(input_branch[0]); p2_ori = P.vertices.row(input_branch[1]);
    dir_ori = p2_ori-p1_ori;
    length = dir_ori.norm();

    for (int i=2; i<(int)input_branch.size(); ++i)
    {
      p1 = P.vertices.row(input_branch[i-1]);
      p2 = P.vertices.row(input_branch[i]);
      dir = p2-p1;
      length += dir.norm();
      similarity = abs(acos(dir_ori.dot(dir) /(dir_ori.norm()*dir.norm()+1e-4))*180 / M_PI);
      similarity = min(similarity, 180.0-similarity);
      if (i == (int)input_branch.size()-1)
      {
        if (similarity < angle_upper && length < length_upper)
        {
          temp_nodes.push_back(input_branch[i]);
          divide_set.push_back(temp_nodes);
        }
        else
        {
          divide_set.push_back(temp_nodes);
          vector<int>().swap(temp_nodes);
          temp_nodes.push_back(input_branch[i-1]); temp_nodes.push_back(input_branch[i]);
          divide_set.push_back(temp_nodes);
        }
      }
      else
      {
        if (similarity < angle_upper && length < length_upper)
        {
          temp_nodes.push_back(input_branch[i]);
          p_end = p2;
          dir_ori = p_end-p1_ori;
        }
        else
        {
          divide_set.push_back(temp_nodes);
          vector<int>().swap(temp_nodes);
          temp_nodes.push_back(input_branch[i-1]); temp_nodes.push_back(input_branch[i]);
          p1_ori = P.vertices.row(input_branch[i-1]); p2_ori = P.vertices.row(input_branch[i]);
          dir_ori = p2_ori-p1_ori;
          length = dir_ori.norm();
        }
      }
    }

    return divide_set;
  }

  vector<int> ROSA_main::merge_branch(vector<int>& input_branch)
  {
    vector<int> merge_set;
    int joint_id = -1, end_id = -1;
    bool inverse = false;

    if (P.degrees(input_branch.front()) > 1)
    {
      joint_id = input_branch.front(); end_id = input_branch.back(); inverse = false;
    }
    if (P.degrees(input_branch.back()) > 1)
    { 
      joint_id = input_branch.back(); end_id = input_branch.front(); inverse = true;
    }
    Eigen::Vector3d input_dir = P.vertices.row(end_id) - P.vertices.row(joint_id);
    
    /* find candidate branch set */
    map<int, vector<int>> candidates_set;
    for (int i=0; i<(int)P.branches.size(); ++i)
    {
      if (P.visited[i] == false)
      {if ((P.branches[i].front() == input_branch.front() && P.branches[i].back() == input_branch.back()) || P.branches[i].size() == 2)
        continue;
      else
      {
        if (P.branches[i].front() == joint_id || P.branches[i].back() == joint_id)
          candidates_set[i] = P.branches[i];
      }}
    }

    if (candidates_set.size() == 0)
      return input_branch;

    int near_merge_id = -1, set_id = -1;
    double sim_min = 10000.0, sim_cur;
    Eigen::Vector3d candi_dir;
    bool head = false;
    vector<int> final_candidate;
    for (const auto& pair : candidates_set)
    {
      if (pair.second.front() == joint_id)
      { near_merge_id = pair.second[1]; head = true; }
      if (pair.second.back() == joint_id)
      { near_merge_id = pair.second[pair.second.size()-2]; head = false; }
      
      candi_dir = P.vertices.row(joint_id) - P.vertices.row(near_merge_id);
      sim_cur = acos(input_dir.dot(candi_dir) /(input_dir.norm()*candi_dir.norm()+1e-4))*180 / M_PI;
      if (sim_cur > 90.0)
        sim_cur = 180.0-sim_cur;
      cout << "similarity:" << sim_cur << endl;

      if (sim_cur <= sim_min)
      {
        sim_min = sim_cur;
        set_id = pair.first;
        final_candidate = pair.second;
      }
    }

    /* merge input_branch and candidate_branch */
    P.visited[set_id] = true;
    merge_set = input_branch;
    if (inverse)
    {
      if (head)
        merge_set.insert(merge_set.end(), final_candidate.begin()+1, final_candidate.end());
      else
      {
        std::reverse(merge_set.begin(), merge_set.end());
        final_candidate.insert(final_candidate.end(), merge_set.begin()+1, merge_set.end());
        merge_set = final_candidate;
      }
    }
    else
    {
      if (head)
      {
        std::reverse(merge_set.begin(), merge_set.end());
        merge_set.insert(merge_set.end(), final_candidate.begin()+1, final_candidate.end());
      }
      else
      {
        final_candidate.insert(final_candidate.end(), merge_set.begin()+1, merge_set.end());
        merge_set = final_candidate;
      }
    }

    return merge_set;
  }

  bool ROSA_main::prune(vector<int>& input_branch)
  {
    bool prune_flag = false;
    int joint_id = -1, end_id = -1;
    if (P.degrees(input_branch.front()) > 1)
    {
      joint_id = input_branch.front(); end_id = input_branch.back();
    }
    if (P.degrees(input_branch.back()) > 1)
    { 
      joint_id = input_branch.back(); end_id = input_branch.front();
    }
    Eigen::Vector3d input_dir = P.vertices.row(end_id) - P.vertices.row(joint_id);
    double input_norm = input_dir.norm();

    list<int>::iterator it; 
    vector<int> neighbors;
    for (it = P.graph[joint_id].begin(); it != P.graph[joint_id].end(); ++it)
    {
      if (*it != end_id)
      {
        neighbors.push_back(*it);
      }
    }
    
    Eigen::Vector3d neighbor_dir;
    for (auto x:neighbors)
    {
      neighbor_dir = P.vertices.row(x) - P.vertices.row(joint_id);
      double neigh_norm = neighbor_dir.norm();
      if (neigh_norm > 2*input_norm)
      {
        prune_flag = true;
        break;
      }
    }

    return prune_flag;
  }

  vector<int> ROSA_main::query_cut_plane_pts(Eigen::Vector3d& p_plane, Eigen::Vector3d& v_plane)
  {
    vector<int> o_indxs, indxs;
    vector<int> isoncut(pcd_size_,0);
    pcloud_isoncut(p_plane, v_plane, isoncut, P.datas, pcd_size_);
    /* find nearest point */
    Eigen::Vector3d in_plane_pt;
    double distance=1001.0, distance_min=1000.0; int nearest_idx=-1;
    for (int i=0; i<(int)isoncut.size(); ++i)
    {
      if (isoncut[i]>0)
      {
        /* search */
        in_plane_pt = P.pts_mat.row(i);
        distance = (p_plane-in_plane_pt).norm();
        if (distance <= distance_min)
        {
          nearest_idx = i;
          distance_min = distance; 
        }
      }
    }
    /* find neighborhood */
    deque<int> queue; queue.push_back(nearest_idx);
    int curr;
    while (!((bool)queue.empty()))
    {
      curr = queue[0];
      queue.pop_front();
      isoncut[curr] = 2;
      if (P.pts_distributed[curr] == false)
        o_indxs.push_back(curr);
      vector<int>().swap(indxs);
      for (int i=0; i<(int)P.neighs[curr].size(); ++i)
      {
        if (isoncut[P.neighs[curr][i]] == 1)
          indxs.push_back(P.neighs[curr][i]);
      }
      for (int j=0; j<(int)indxs.size(); ++j)
      {
        isoncut[indxs[j]] = 3;
        queue.push_back(indxs[j]);
      }
    }
    
    return o_indxs;
  }

  double ROSA_main::distance_point_line(Eigen::Vector3d& point, Eigen::Vector3d& line_pt, Eigen::Vector3d& line_dir)
  {
    // Find a point on the line closest to the given point
    Eigen::Vector3d d = point - line_pt;
    double t = d.dot(line_dir);
    Eigen::Vector3d closest_pt_on_line = line_pt + t * line_dir;
    
    // Calculate the distance between the given point and the closest point on the line
    return (point - closest_pt_on_line).norm();
  }

  Eigen::Vector3d ROSA_main::PCA(Eigen::MatrixXd& A)
  {
    Eigen::Vector3d vec;
    Eigen::Vector3d centroid = A.colwise().mean();
    Eigen::Matrix3d cov = (A.rowwise() - centroid.transpose()).transpose() * (A.rowwise() - centroid.transpose()) / double(A.rows() - 1);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    vec = eig.eigenvectors().col(2);

    return vec;
  }
}
