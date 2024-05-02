/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main algorithm of basic solver for planning
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

#include <hierarchical_coverage_planner/hcsolver.h>

mutex mtx;
condition_variable con_var;
const double MAX = 1e6;

namespace predrecon
{
  HCSolver::HCSolver(){
  }

  HCSolver::~HCSolver(){
  }

  void HCSolver::init(ros::NodeHandle& nh, Eigen::Vector3d& path_start_, SDFMap::Ptr& hcmap, double& res_, Eigen::Vector3d& origin_)
  {
    // * Module Initialization
    mt_nh_ = nh;

    astar_.reset(new Astar);
    astar_->init_hc(nh, hcmap);

    solve_raycaster_.reset(new RayCaster);
    solve_raycaster_->setParams(res_, origin_);

    this->mt_map_ = hcmap;
    this->solve_map_ = hcmap;

    // * Params Initialization
    solver_start_ = path_start_;
    nh.param("hcplanner/precision_", precision_, -1);
    nh.param("hcplanner/global_solver", GlobalSolver_, string("null"));
    nh.param("hcplanner/global_par_file", GlobalPar_, string("null"));
    nh.param("hcplanner/global_problem_file", GlobalProF_, string("null"));
    nh.param("hcplanner/global_result_file", GlobalResult_, string("null"));
    nh.param("hcplanner/global_runs", GlobalRuns_, -1);
    nh.param("hcplanner/local_folder", LocalFolder_, string("null"));
    nh.param("hcplanner/local_runs", LocalRuns_, -1);
    nh.param("hcplanner/tripod_head", tripod_head_trigger_, false);
    nh.param("hcplanner/vmax_", vm_, -1.0);
    nh.param("hcplanner/amax_", am_, -1.0);
    nh.param("hcplanner/jmax_", jm_, -1.0);
    nh.param("hcplanner/ydmax_", yd_, -1.0);
    nh.param("hcplanner/yddmax_", ydd_, -1.0);
    nh.param("hcplanner/local2opt_trial", local2optNum, -1);
    nh.param("astar/searching_res", astarSearchingRes, -1.0);
  }

  vector<int> HCSolver::GlobalSubspaceSequence(map<int, vector<Eigen::VectorXd>>& sub_vps)
  {
    vector<int> GlobalSeq;
    
    Eigen::Vector3d sub_cen = Eigen::Vector3d::Zero();
    for (const auto& pair:sub_vps)
    {
      sub_cen(0) = 0.0; sub_cen(1) = 0.0; sub_cen(2) = 0.0;
      for (auto pose:pair.second)
      {
        sub_cen(0) += pose(0);
        sub_cen(1) += pose(1);
        sub_cen(2) += pose(2);
      }
      sub_cen = sub_cen/(double)pair.second.size();
      centroids.push_back(sub_cen);
    }

    if ((int)sub_vps.size() > 1)
    {
      /* write par file */
      GlobalParWrite();
      /* construct ATSP cost matrix */
      Eigen::MatrixXd GloablCostMat;
      GloablCostMat = GlobalCostMat(solver_start_, centroids);
      /* write problem file */
      GlobalProblemWrite(GloablCostMat);
      /* ATSP solving */
      string command_ = "cd " + GlobalSolver_ + " && ./LKH " + GlobalPar_;
      const char* charPtr = command_.c_str();
      system_back_=system(charPtr);
      /* read solution results */
      GlobalSeq = GlobalResultsRead();
    }
    else
    {
      int id_ = -1;
      for (const auto& pair:sub_vps)
        id_ = pair.first;
      GlobalSeq = {id_};
    }

    return GlobalSeq;
  }

  map<int, vector<int>> HCSolver::GlobalBoundaryPoints(map<int, vector<Eigen::VectorXd>>& sub_vps, vector<int>& globalseq)
  { 
    map<int, vector<int>> boundary_ids_;

    if ((int)sub_vps.size() == 1)
    {
      int sub_id = -1;
      pcl::PointCloud<pcl::PointXYZ>::Ptr oneCloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointXYZ onept, startpt;
      for (const auto& pair:sub_vps)
      {
        sub_id = pair.first;
        for (auto vp:pair.second)
        {
          onept.x = vp(0); onept.y = vp(1); onept.z = vp(2);
          oneCloud->points.push_back(onept);
        }
      }
      pcl::KdTreeFLANN<pcl::PointXYZ> oneTree;
      oneTree.setInputCloud(oneCloud);
      vector<int> nearest(1);
      vector<float> nn_squared_distance(1);
      startpt.x = solver_start_(0); startpt.y = solver_start_(1); startpt.z = solver_start_(2);  
      oneTree.nearestKSearch(startpt, 1, nearest, nn_squared_distance);

      vector<int> boundary; boundary.push_back(nearest[0]);
      boundary_ids_[sub_id] = boundary;
      
      return boundary_ids_;
    }
    
    vector<Eigen::Vector3d> global_site_; global_site_.push_back(solver_start_);
    vector<int> site_seq_; site_seq_.push_back(0);
    for (auto x:globalseq)
    {
      global_site_.push_back(centroids[x]);
      site_seq_.push_back(x);
    }

    /* find start and end of each sub-space */
    Eigen::Vector3d start_s_site_, start_e_site_, end_s_site_, end_e_site_;
    int sub_space_id_;
    int start_id, end_id;
    vector<int> temp_sub_ids_;
    vector<Eigen::VectorXd> sub_space_vps_;
    for (int i=0; i<(int)global_site_.size()-1; ++i)
    {
      vector<int>().swap(temp_sub_ids_);
      sub_space_id_ = site_seq_[i+1];
      sub_space_vps_ = sub_vps.find(sub_space_id_)->second;
      // for start_id
      start_s_site_ = global_site_[i];
      start_e_site_ = global_site_[i+1];
      start_id = FindSphereNearestPoint(start_s_site_, start_e_site_, sub_space_vps_);
      temp_sub_ids_.push_back(start_id);
      // for end_id, notably, final sub-space is without end constraint.
      end_s_site_ = global_site_[i+1];
      if (i+2 < (int)global_site_.size())
      {
        end_e_site_ = global_site_[i+2];
        end_id = FindSphereNearestPoint(end_s_site_, end_e_site_, sub_space_vps_);
        temp_sub_ids_.push_back(end_id);
      }
      boundary_ids_[sub_space_id_] = temp_sub_ids_;
    }

    return boundary_ids_;
  }

  tuple<map<int, vector<Eigen::VectorXd>>, map<int, vector<vector<Eigen::VectorXd>>>> HCSolver::LocalConditionalPath(map<int, vector<Eigen::VectorXd>>& sub_vps, map<int, vector<int>>& global_boundary, bool turn)
  {
    auto hcopp_astar_t1 = std::chrono::high_resolution_clock::now();
    for (const auto& pair:sub_vps)
    {
      LocalFindPath(pair.second, global_boundary.find(pair.first)->second, pair.first, turn);
    }
    auto hcopp_astar_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> hcopp_astar_ms = hcopp_astar_t2 - hcopp_astar_t1;
    double hcopp_astar_time = (double)hcopp_astar_ms.count();
    ROS_INFO("\033[33m[Planner] path searching time = %lf ms.\033[32m", hcopp_astar_time);

    int sub_space_id_, numThreads=(int)sub_vps.size(), num=0;
    thread threads[numThreads];

    for (const auto& pair:sub_vps)
    {
      sub_space_id_ = pair.first;
      threads[num] = thread(&HCSolver::LocalPathFinder, this, pair.second, global_boundary.find(sub_space_id_)->second, sub_space_id_);
      num++;
    }

    /* Join all threads if they haven't already been joined */
    for (int i = 0; i < numThreads; i++)
    {
      threads[i].join();
    }
    
    return make_tuple(local_sub_path_viewpts_, local_sub_path_waypts_);
  }

  tuple<vector<Eigen::VectorXd>, vector<vector<Eigen::VectorXd>>> HCSolver::CoverageFullPath(
    Eigen::Vector3d& cur_pos, vector<int>& globalseq, map<int, vector<Eigen::VectorXd>>& localviewpts, map<int, vector<vector<Eigen::VectorXd>>>& localwaypts)
  {
    vector<Eigen::VectorXd> full_viewpts_;
    vector<vector<Eigen::VectorXd>> full_waypts_;
    Eigen::Vector3d p1, p2;
    vector<Eigen::Vector3d> waypts_;
    vector<Eigen::VectorXd> UPwaypts_;

    Eigen::VectorXd start_pose; start_pose.resize(5);
    start_pose << cur_pos(0), cur_pos(1), cur_pos(2), 0.0, 0.0;
    full_viewpts_.push_back(start_pose);
    /* start path */
    vector<Eigen::Vector3d>().swap(waypts_);
    vector<Eigen::VectorXd>().swap(UPwaypts_);
    waypts_ = {};
    UPwaypts_ = {};
    full_waypts_.push_back(UPwaypts_);
    Eigen::Vector3d lastPos = full_viewpts_.back().head(3);
    for (auto pose:localviewpts.find(globalseq[0])->second)
    {
      Eigen::Vector3d curPos = pose.head(3);
      if ((lastPos-curPos).norm() < 1e-3)
        continue;
      full_viewpts_.push_back(pose);
    }
    full_waypts_.insert(full_waypts_.end(), localwaypts.find(globalseq[0])->second.begin(), localwaypts.find(globalseq[0])->second.end());

    /* following paths */
    for (int i=0; i<(int)globalseq.size()-1; ++i)
    {
      vector<Eigen::Vector3d>().swap(waypts_);
      vector<Eigen::VectorXd>().swap(UPwaypts_);
      p1 = localviewpts.find(globalseq[i])->second.back().head(3);
      p2 = localviewpts.find(globalseq[i+1])->second.front().head(3);
      bridge_length_ = search_Path(p1, p2, waypts_);
      if ((int)waypts_.size() > 2)
      {
        AngleInterpolation(localviewpts.find(globalseq[i])->second.back(), localviewpts.find(globalseq[i+1])->second.front(), waypts_, UPwaypts_);
        full_waypts_.push_back(UPwaypts_);
      }
      else
        full_waypts_.push_back({});

      Eigen::Vector3d lastPos = full_viewpts_.back().head(3);
      for (auto pose:localviewpts.find(globalseq[i+1])->second)
      {
        Eigen::Vector3d curPos = pose.head(3);
        if ((lastPos-curPos).norm() < 1e-3)
          continue;
        full_viewpts_.push_back(pose);
      }
      full_waypts_.insert(full_waypts_.end(), localwaypts.find(globalseq[i+1])->second.begin(), localwaypts.find(globalseq[i+1])->second.end());
    }

    return make_tuple(full_viewpts_, full_waypts_);
  }

  vector<Eigen::VectorXd> HCSolver::LocalRefine(vector<Eigen::Vector3d>& Joints, double& Range, vector<Eigen::VectorXd>& Path, bool turn)
  {
    vector<Eigen::VectorXd> RefinedPath, FinalPath;
    
    // ! Random Local 2-opt/3-opt
    RefinedPath = Path;
    pcl::KdTreeFLANN<pcl::PointXYZ> PathTree;
    vector<int> indxs;
	  vector<float> radius_squared_distance;
    pcl::PointXYZ serachPt;

    vector<int> innervpsID, finalID;
    ConstructKDTree(RefinedPath, PathTree);
    for (auto jt:Joints)
    {
      serachPt.x = jt(0); serachPt.y = jt(1); serachPt.z = jt(2);
      PathTree.radiusSearch(serachPt, Range, indxs, radius_squared_distance);
      innervpsID.insert(innervpsID.end(), indxs.begin(), indxs.end());
      vector<Eigen::Vector3d> innervps;
      for (int k=0; k<(int)indxs.size(); ++k)
        innervps.push_back(RefinedPath[indxs[k]].head(3));
      JointVps.push_back(innervps);
    }
    set<int> uniqueSet(innervpsID.begin(), innervpsID.end());
    for (const auto& num : uniqueSet)
      finalID.push_back(num);
    for (auto id:finalID)
      LocalVps.push_back(RefinedPath[id].head(3));

    for (int i=0; i<(int)RefinedPath.size(); ++i)
      RefineID[RefinedPath[i].head(3)] = i;
    
    double refinelength = 0.0;
    for (int i=1; i<(int)RefinedPath.size()-1; ++i)
      refinelength += (RefinedPath[i+1].head(3)-RefinedPath[i].head(3)).norm();

    /* Local random 2 Opt */
    auto twoopt_t1 = std::chrono::high_resolution_clock::now();
    // * data prepare START
    int gID = 0;
    Site* StartSite = new Site;
    StartSite->X = RefinedPath.front()(0); StartSite->Y = RefinedPath.front()(1); StartSite->Z = RefinedPath.front()(2);
    StartSite->Pitch = RefinedPath.front().size() == 5? RefinedPath.front()(3):0.0;
    StartSite->Yaw = RefinedPath.front().tail(1)(0); 
    StartSite->Start = true; StartSite->End = false;
    StartSite->GlobalID = gID;
    gID++;
    AllPathSite.push_back(StartSite);
    for (int i=1; i<(int)RefinedPath.size()-1; ++i)
    {
      Site* tempSite = new Site;
      tempSite->X = RefinedPath[i](0); tempSite->Y = RefinedPath[i](1); tempSite->Z = RefinedPath[i](2);
      tempSite->Pitch = RefinedPath[i].size() == 5? RefinedPath[i](3):0.0;
      tempSite->Yaw = RefinedPath[i].tail(1)(0);
      tempSite->Start = false; tempSite->End = false;
      tempSite->GlobalID = gID;
      gID++;
      AllPathSite.push_back(tempSite);
    }
    Site* EndSite = new Site;
    EndSite->X = RefinedPath.back()(0); EndSite->Y = RefinedPath.back()(1); EndSite->Z = RefinedPath.back()(2);
    EndSite->Pitch = RefinedPath.back().size() == 5? RefinedPath.back()(3):0.0;
    EndSite->Yaw = RefinedPath.back().tail(1)(0); 
    EndSite->Start = false; EndSite->End = true;
    EndSite->GlobalID = gID;
    AllPathSite.push_back(EndSite);

    AllPathSite.front()->Suc = AllPathSite[1];
    for (int i=1; i<(int)AllPathSite.size()-1; ++i)
    {
      AllPathSite[i]->Pred = AllPathSite[i-1];
      AllPathSite[i]->Suc = AllPathSite[i+1];
    }
    AllPathSite.back()->Pred = AllPathSite[(int)AllPathSite.size()-2];
    
    int count = 0;
    for (auto vp:LocalVps)
    {
      int id = RefineID.find(vp)->second;
      if (AllPathSite[id]->Start == true || AllPathSite[id]->End == true || AllPathSite[id]->Pred->Start == true || AllPathSite[id]->Suc->End == true || AllPathSite[id]->Pred->Pred->Start == true || AllPathSite[id]->Suc->Suc->End == true)
      {
        continue;
      }
      AllPathSite[id]->LocalID = count;
      LocalSite.push_back(AllPathSite[id]);
      count++;
    }
    // * data prepare END

    // * Local 2-opt/3-opt START
    swapTimes = 0;
    for (int i=0; i<local2optNum; ++i)
    {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> dis(0.0, 1.0);
      double prob_local = dis(gen);
      if (prob_local > 0.2)
      {
        RandomLocal2Opt(turn);
      }
      else
      {
        RandomLocal3Opt(turn);
      }
    }
    ROS_INFO("\033[37m[LocalRefine] total local search attempts: %d, valid local search attempts: %d. \033[32m", local2optNum, swapTimes);
    // * Local 2-opt/3-opt END
    
    // * obtain final path START
    Site *StartNode = new Site;
    for (auto n:AllPathSite)
    {
      if (n->Start == true)
        StartNode = n;
    }
    Eigen::VectorXd pose; pose.resize(5);
    pose << StartNode->X, StartNode->Y, StartNode->Z, StartNode->Pitch, StartNode->Yaw;
    FinalPath.push_back(pose);
    Eigen::VectorXd last_pose;
    
    Site *LastNode = new Site; LastNode = StartNode;
    Site *NextNode = new Site;
    NextNode->End = false;
    while (NextNode->End != true)
    {
      NextNode = LastNode->Suc;
      pose(0) = NextNode->X; pose(1) = NextNode->Y; pose(2) = NextNode->Z; pose(3) = NextNode->Pitch; pose(4) = NextNode->Yaw;
      last_pose = FinalPath.back();
      // --- START insert collision-free interpath ---
      vector<Eigen::Vector3d> ori_path;
      vector<Eigen::VectorXd> updated_path;
      Eigen::Vector3d last_pos = last_pose.head(3);
      Eigen::Vector3d now_pos = pose.head(3);
      inter_cost = search_Path(last_pos, now_pos, ori_path);
      if ((int)ori_path.size() > 2)
        AngleInterpolation(last_pose, pose, ori_path, updated_path);
      else
        updated_path = {last_pose, pose};
      
      if ((int)updated_path.size() > 2)
      {
        for (int k=1; k<(int)updated_path.size()-1; ++k)
          FinalPath.push_back(updated_path[k]);
      }
      // --- END insert collision-free interpath ---

      FinalPath.push_back(pose);
      LastNode = NextNode;
    }
    auto twoopt_t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> twoopt_ms = twoopt_t2 - twoopt_t1;
    // * obtain final path END

    double local2optlength = 0.0;
    for (int i=1; i<(int)FinalPath.size()-1; ++i)
      local2optlength += (FinalPath[i+1].head(3)-FinalPath[i].head(3)).norm();
    
    ROS_INFO("\033[37m[LocalRefine] local path refinement improvement = %lf m.\033[32m", refinelength-local2optlength);

    return FinalPath;
  }

  // ! ------------------------------------- Utils -------------------------------------
  Eigen::MatrixXd HCSolver::GlobalCostMat(Eigen::Vector3d& start_, vector<Eigen::Vector3d>& targets)
  {
    vector<Eigen::Vector3d> total_;
    total_.push_back(start_);
    total_.insert(total_.begin()+1, targets.begin(), targets.end());
    Eigen::Vector3d vec_i, vec_j;
    double cost_;

    int dim = (int)total_.size();
    Eigen::MatrixXd costmat_;
    costmat_.resize(dim, dim);
    costmat_.setZero();

    for (int i=1; i<dim; ++i)
      for (int j=1; j<dim; ++j)
      {
        vec_i = total_[i];
        vec_j = total_[j];
        cost_ = (vec_i-vec_j).norm();
        costmat_(i,j) = cost_;
      }
    
    for (int k=0; k<dim; ++k)
    {
      vec_i = total_[0];
      vec_j = total_[k];
      cost_ = (vec_i-vec_j).norm();
      costmat_(0,k) = cost_;
    }

    return costmat_;
  }

  void HCSolver::GlobalParWrite()
  {
    ofstream par_file(GlobalPar_);
    par_file << "PROBLEM_FILE = " << GlobalProF_ << "\n";
    par_file << "GAIN23 = NO\n";
    par_file << "OUTPUT_TOUR_FILE =" << GlobalResult_ << "\n";
    par_file << "RUNS = " << to_string(GlobalRuns_) << "\n";
    par_file.close();
  }

  void HCSolver::GlobalProblemWrite(Eigen::MatrixXd& costMat)
  {
    const int dimension = costMat.rows();
    ofstream prob_file(GlobalProF_);
    string prob_spec = "NAME : global\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
    prob_file << prob_spec;

    for (int i=0; i<dimension; ++i)
    {
      for (int j=0; j<dimension; ++j)
      {
        int int_cost = costMat(i,j)*precision_;
        prob_file << to_string(int_cost) << " ";
      }
      prob_file << "\n";
    }

    prob_file << "EOF";
    prob_file.close();
  }

  vector<int> HCSolver::GlobalResultsRead()
  {
    vector<int> results;

    ifstream res_file(GlobalResult_);
    string res;
    while (getline(res_file, res)) 
      if (res.compare("TOUR_SECTION") == 0) break;
    
    while (getline(res_file, res)) 
    {
      int id = stoi(res);
      if (id == 1)
        continue;
      if (id == -1) break;
      results.push_back(id - 2);
    }
    res_file.close();

    return results;
  }

  int HCSolver::FindSphereNearestPoint(Eigen::Vector3d& spherePtA, Eigen::Vector3d& spherePtB, vector<Eigen::VectorXd>& vps)
  {
    int nearestID = -1;
    Eigen::Vector3d vp_pos_;
    double dist_min_ = MAX;

    for (int i=0; i<(int)vps.size(); ++i) 
    {
      vp_pos_(0) = vps[i](0); vp_pos_(1) = vps[i](1); vp_pos_(2) = vps[i](2); 
      // Compute the distance between the point and the line segment AB
      Eigen::Vector3d AP = vp_pos_ - spherePtA;
      Eigen::Vector3d BP = vp_pos_ - spherePtB;

      double within_dist_ = AP.norm() + BP.norm();
      if (within_dist_ < dist_min_)
      {
        dist_min_ = within_dist_;
        nearestID = i;
      }
    }

    return nearestID;
  }
  /* the path from p1 to p2 */
  double HCSolver::search_Path(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, vector<Eigen::Vector3d>& path)
  {
    /* try to connect two points with straight line */
    Eigen::MatrixXd ori_path;
    bool safe = true;

    Eigen::Vector3i idx;
    solve_raycaster_->input(p1, p2);
    while (solve_raycaster_->nextId(idx)) 
    {
      if (solve_map_->hcmd_->occupancy_inflate_buffer_hc_[solve_map_->toAddress_hc(idx)] == 1 || solve_map_->hcmd_->occupancy_buffer_internal_[solve_map_->toAddress_hc(idx)] == 1 || !solve_map_->isInMap_hc(idx)) 
      {
        safe = false;
        break;
      }
      
      if (safe == false)
        break;
    }

    if (safe) 
    {
      path = { p1, p2 };

      phy_dist = (p1-p2).norm();

      return (p1-p2).norm();
    }
    /* Search a path using decreasing resolution */
    vector<double> res = { astarSearchingRes };
    for (int k = 0; k < (int)res.size(); ++k) 
    {
      astar_->reset();
      astar_->setResolution(res[k]);
      if (astar_->hc_search(p1, p2) == Astar::REACH_END) 
      {
        path = astar_->getPath();
        shortenPath(path);
        
        phy_dist = astar_->pathLength(path);

        return astar_->pathLength(path);
      }
    }
    /* Use Astar early termination cost as an estimate */
    path = { p1, p2 };
    phy_dist = 100000.0;
    return 100000.0;
  }

  void HCSolver::shortenPath(vector<Eigen::Vector3d>& path)
  {
    if (path.empty()) 
    {
      ROS_ERROR("Empty path to shorten");
      return;
    }
    // Shorten the tour, only critical intermediate points are reserved.
    const double dist_thresh = 5.0;
    vector<Eigen::Vector3d> short_tour = { path.front() };
    for (int i = 1; i < (int)path.size() - 1; ++i) 
    {
      if ((path[i] - short_tour.back()).norm() > dist_thresh)
        short_tour.push_back(path[i]);
      else 
      {
        // Add waypoints to shorten path only to avoid collision
        solve_raycaster_->input(short_tour.back(), path[i + 1]);
        Eigen::Vector3i idx;
        while (solve_raycaster_->nextId(idx)) 
        {
          if (solve_map_->hcmd_->occupancy_buffer_hc_[solve_map_->toAddress_hc(idx)] == 1 || solve_map_->hcmd_->occupancy_buffer_internal_[solve_map_->toAddress_hc(idx)] == 1) 
          {
            short_tour.push_back(path[i]);
            break;
          }
        }
      }
    }
    if ((path.back() - short_tour.back()).norm() > 1e-3) 
      short_tour.push_back(path.back());

    path = short_tour;
  }
  /* for the platform without tripod head */
  double HCSolver::compute_Cost(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const double& y1, const double& y2, vector<Eigen::Vector3d>& path)
  {
    /* cost of position change */
    double pos_dist = search_Path(p1, p2, path);
    double pos_time = -1.0;

    pos_time = pos_dist/vm_;
    
    /* cost of yaw change */
    double yaw_diff = fabs(y2 - y1);
    yaw_diff = min(yaw_diff, 2 * M_PI - yaw_diff);
    double yaw_time = yaw_diff/yd_;

    return max(pos_time, yaw_time);
  }
  /* uniform acceleration time cost model */
  double HCSolver::computeTimeCost(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const double& y1, const double& y2, vector<Eigen::Vector3d>& path)
  {
    double pos_dist = search_Path(p1, p2, path);
    double pos_cost = 0.0;
    if ((int)path.size() > 2)
    {
      pos_cost += (path[1]-path[0]).norm()/vm_;
      for (int i=0; i<(int)path.size()-1; ++i)
        pos_cost += pathTimeUniAcc(path[i-1], path[i], path[i+1]);
    }
    else
    {
      pos_cost = pos_dist/vm_;
    }

    double angle_diff = fabs(y2 - y1);
    angle_diff = min(angle_diff, 2 * M_PI - angle_diff);
    double angle_cost = angle_diff/yd_;

     return max(pos_cost, angle_cost);
  }
  /* given A, B, C three waypoints, uniform acceleration motion model to calculate time cost of BC */
  double HCSolver::pathTimeUniAcc(Eigen::Vector3d& pred, Eigen::Vector3d& cur, Eigen::Vector3d& suc)
  {
    // 1. calculate the angle between two segments
    double theta = acos((pred-cur).normalized().dot((suc-cur).normalized()));
    // 2. calculate the upper accelerating distance
    double s = pow(vm_,2)*sin(theta)*cos(theta/2)/am_;
    double l = (cur-suc).norm();
    // 3. calculate time cost
    double time = 0.0;
    if (s < l)
    {
      time = l/vm_ + 2*vm_*pow(sin(theta/2),3)/(am_);
    }
    else
    {
      double temp = pow(vm_,2)*pow(cos(theta),2) + 2*am_*sin(theta/2)*l;
      time = (pow(temp, 0.5)-vm_*cos(theta))/(am_*sin(theta/2));
    }

    double flightCost = time;

    return flightCost;
  }

  /* for the platform with tripod head */
  double HCSolver::compute_Cost_tripod_head(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const double& y1, const double& y2, vector<Eigen::Vector3d>& path)
  {
    /* cost of position change */
    double pos_dist = search_Path(p1, p2, path);
    double pos_time = -1.0;

    pos_time = pos_dist/vm_;
    
    return pos_time;
  }

  double HCSolver::computeTimeCost_tripod_head(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const double& y1, const double& y2, vector<Eigen::Vector3d>& path)
  {
    double pos_dist = search_Path(p1, p2, path);
    double pos_cost = 0.0;
    if ((int)path.size() > 2)
    {
      pos_cost += (path[1]-path[0]).norm()/vm_;
      for (int i=0; i<(int)path.size()-1; ++i)
        pos_cost += pathTimeUniAcc(path[i-1], path[i], path[i+1]);
    }
    else
    {
      pos_cost = pos_dist/vm_;
    }

    return pos_cost;
  }

  void HCSolver::LocalFindPath(vector<Eigen::VectorXd> vps, vector<int> boundary_, int sub_id_, bool turn)
  {
    int start_id = -1, end_id = -1;
    vector<int> local_id_init_;
    bool bilateral = false;

    if (boundary_.size() == 2)
    {
      start_id = boundary_[0];
      end_id = boundary_[1];
      local_id_init_.push_back(start_id);
      for (int id=0; id<(int)vps.size(); ++id)
      {
        if (id != start_id && id != end_id)
          local_id_init_.push_back(id);
      }
      local_id_init_.push_back(end_id);
      bilateral = true;
    }
    else if (boundary_.size() == 1)
    {
      start_id = boundary_[0];
      local_id_init_.push_back(start_id);
      for (int id=0; id<(int)vps.size(); ++id)
      {
        if (id != start_id)
          local_id_init_.push_back(id);
      }
      bilateral = false;
    }
    /* local initial idxs conditioned on boundary */
    local_sub_init_idx_[sub_id_] = local_id_init_;
    
    int dim = (int)local_id_init_.size();
    Eigen::MatrixXd costmat_, phymat_;
    costmat_ = Eigen::MatrixXd::Zero(dim, dim);
    phymat_ = Eigen::MatrixXd::Zero(dim, dim);
    
    Eigen::Vector3d pos_i, pos_j;
    double yaw_i, yaw_j;
    double pitch_i, pitch_j;
    double cost_ij_;
    vector<Eigen::Vector3d> ij_path_;
    vector<Eigen::VectorXd> updated_ij_path_;
    vector<double> ij_pos_;

    for (int i=1; i<(int)local_id_init_.size(); ++i)
      for (int j=1; j<(int)local_id_init_.size(); ++j)
      {
        vector<Eigen::Vector3d>().swap(ij_path_);
        vector<Eigen::VectorXd>().swap(updated_ij_path_);
        pos_i = vps[local_id_init_[i]].head(3);
        pos_j = vps[local_id_init_[j]].head(3);
        pitch_i = vps[local_id_init_[i]](3);
        pitch_j = vps[local_id_init_[j]](3);
        yaw_i = vps[local_id_init_[i]].tail(1)(0);
        yaw_j = vps[local_id_init_[j]].tail(1)(0);

        double p_diff = min(abs(pitch_i-pitch_j), 2*M_PI-abs(pitch_i-pitch_j));
        double y_diff = min(abs(yaw_i-yaw_j), 2*M_PI-abs(yaw_i-yaw_j));
        double a1, a2;
        if (p_diff > y_diff)
        {
          a1 = pitch_i;
          a2 = pitch_j;
        }
        else
        {
          a1 = yaw_i;
          a2 = yaw_j;
        }
        
        if ((pos_i - pos_j).norm() < 20.0)
        {
        if (tripod_head_trigger_ == true)
        {
          if (turn == true)
            cost_ij_ = computeTimeCost_tripod_head(pos_i, pos_j, a1, a2, ij_path_);
          else
            cost_ij_ = compute_Cost_tripod_head(pos_i, pos_j, a1, a2, ij_path_);
        }
        else
        {
          if (turn == true)
            cost_ij_ = computeTimeCost(pos_i, pos_j, a1, a2, ij_path_);
          else
            cost_ij_ = compute_Cost(pos_i, pos_j, a1, a2, ij_path_);
        }
        }
        else
        {
          cost_ij_ = 100000.0;
        }
        vector<int> path_index_ = {sub_id_, local_id_init_[i], local_id_init_[j]};
        costmat_(i,j) = cost_ij_;
        phymat_(i,j) = phy_dist;
        ij_pos_ = {pos_i(0), pos_i(1), pos_i(2), pos_j(0), pos_j(1), pos_j(2)};
        allAstarCost_[ij_pos_] = cost_ij_;
        if ((int)ij_path_.size() > 2)
        {
          AngleInterpolation(vps[local_id_init_[i]], vps[local_id_init_[j]], ij_path_, updated_ij_path_);
        }
        else
          updated_ij_path_ = {vps[local_id_init_[i]], vps[local_id_init_[j]]};
        path_waypts_[path_index_] = updated_ij_path_;
        allAstarPath_[ij_pos_] = updated_ij_path_;
      }
    /* start constraint */
    for (int k=0; k<dim; ++k)
    {
      vector<Eigen::Vector3d>().swap(ij_path_);
      vector<Eigen::VectorXd>().swap(updated_ij_path_);

      pos_i = vps[local_id_init_[0]].head(3);
      pos_j = vps[local_id_init_[k]].head(3);
      pitch_i = vps[local_id_init_[0]](3);
      pitch_j = vps[local_id_init_[k]](3);
      yaw_i = vps[local_id_init_[0]].tail(1)(0);
      yaw_j = vps[local_id_init_[k]].tail(1)(0);

      double p_diff = min(abs(pitch_i-pitch_j), 2*M_PI-abs(pitch_i-pitch_j));
      double y_diff = min(abs(yaw_i-yaw_j), 2*M_PI-abs(yaw_i-yaw_j));
      double a1, a2;
      if (p_diff > y_diff)
      {
        a1 = pitch_i;
        a2 = pitch_j;
      }
      else
      {
        a1 = yaw_i;
        a2 = yaw_j;
      }

      if ((pos_i - pos_j).norm() < 20.0)
      {
      if (tripod_head_trigger_ == true)
      {
        if (turn == true)
          cost_ij_ = computeTimeCost_tripod_head(pos_i, pos_j, a1, a2, ij_path_);
        else
          cost_ij_ = compute_Cost_tripod_head(pos_i, pos_j, a1, a2, ij_path_);
      }
      else
      {
        if (turn == true)
          cost_ij_ = computeTimeCost(pos_i, pos_j, a1, a2, ij_path_);
        else
          cost_ij_ = compute_Cost(pos_i, pos_j, a1, a2, ij_path_);
      }
      }
      else
      {
        cost_ij_ = 100000.0;
      }

      vector<int> path_index_ = {sub_id_, local_id_init_[0], local_id_init_[k]};
      costmat_(0,k) = cost_ij_;
      phymat_(0,k) = phy_dist;
      ij_pos_ = {pos_i(0), pos_i(1), pos_i(2), pos_j(0), pos_j(1), pos_j(2)};
      allAstarCost_[ij_pos_] = cost_ij_;
      if ((int)ij_path_.size() > 2)
        AngleInterpolation(vps[local_id_init_[0]], vps[local_id_init_[k]], ij_path_, updated_ij_path_);
      else
        updated_ij_path_ = {vps[local_id_init_[0]], vps[local_id_init_[k]]};
      path_waypts_[path_index_] = updated_ij_path_;
      allAstarPath_[ij_pos_] = updated_ij_path_;
    }
    /* end constraint */
    if (bilateral == true)
    {
      for (int m=1; m<dim-1; ++m)
      {
        costmat_(dim-1, m) = 100000.0;
        phymat_(dim-1, m) = 100000.0;
      }
    }

    local_sub_costmat_[sub_id_] = costmat_;
    local_sub_phymat_[sub_id_] = phymat_;
  }

  void HCSolver::LocalPathFinder(vector<Eigen::VectorXd> vps, vector<int> boundary_, int sub_id_)
  {
    Eigen::MatrixXd phyMat = local_sub_phymat_.find(sub_id_)->second;

    if ((int)vps.size() < 3 || (int)phyMat.rows() < 3)
    {
      vector<Eigen::VectorXd> viewpointsSpec;
      vector<vector<Eigen::VectorXd>> waypointsSpec;

      viewpointsSpec.push_back(vps[boundary_[0]]);
      if ((int)vps.size() == 2)
      {
        waypointsSpec.push_back({});
        if (vps[0](0) == vps[boundary_[0]](0) && vps[0](1) == vps[boundary_[0]](1) && vps[0](2) == vps[boundary_[0]](2))
          viewpointsSpec.push_back(vps[1]);
        else
          viewpointsSpec.push_back(vps[0]);
      }
      else
      {
        waypointsSpec.push_back({});
        viewpointsSpec.push_back(vps[0]);
      }
      local_sub_path_viewpts_[sub_id_] = viewpointsSpec;
      local_sub_path_waypts_[sub_id_] = waypointsSpec;
    }
    else
    {
    vector<int> init_id_ = local_sub_init_idx_.find(sub_id_)->second;
    /* prepare related files */
    string LocalParF_ = LocalFolder_+"/sub_"+to_string(sub_id_)+".par";
    string LocalProbF_ = LocalFolder_+"/sub_"+to_string(sub_id_)+".tsp";
    string LocalResultF_ = LocalFolder_+"/sub_"+to_string(sub_id_)+"_solution.txt";
    
    vector<Eigen::VectorXd> path_viewpointsPrior;
    vector<vector<Eigen::VectorXd>> path_waypointsPrior;
    /* preparation end */

    /* write parameter file */
    ofstream par_file(LocalParF_);
    par_file << "PROBLEM_FILE = " << LocalProbF_ << "\n";
    par_file << "GAIN23 = NO\n";
    par_file << "OUTPUT_TOUR_FILE =" << LocalResultF_ << "\n";
    par_file << "RUNS = " << to_string(LocalRuns_) << "\n";
    par_file.close();
    /* write parameter end */

    /* write problem file */
    Eigen::MatrixXd costMat = local_sub_costmat_.find(sub_id_)->second;
    ostringstream v_s, a_s;
    v_s << fixed << setprecision(1) << vm_;
    a_s << fixed << setprecision(1) << am_;
    const int dimension = costMat.rows();
    ofstream prob_file(LocalProbF_);
    string prob_spec;

    prob_spec = "NAME : local_" + to_string(sub_id_) +
    "\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
    "\nEDGE_WEIGHT_TYPE : "
    "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX" + 
    "\nEDGE_WEIGHT_SECTION\n";

    prob_file << prob_spec;
    
    for (int i=0; i<dimension; ++i)
    {
      for (int j=0; j<dimension; ++j)
      {
        int int_cost = costMat(i,j)*precision_;
        prob_file << to_string(int_cost) << " ";
      }
      prob_file << "\n";
    }

    prob_file << "EOF";
    prob_file.close();
    /* write problem end */

    /* TSP solving... based on the initial idxs */
    string local_command_;
    local_command_ = "cd " + GlobalSolver_ + " && ./LKH " + LocalParF_;
    const char* local_charPtr = local_command_.c_str();
    local_system_back_ = system(local_charPtr);
    /* TSP solved */

    /* read results */
    vector<int> results;
    ifstream res_file(LocalResultF_);
    string res;
    while (getline(res_file, res)) 
      if (res.compare("TOUR_SECTION") == 0) break;
    while (getline(res_file, res)) 
    {
      int id = stoi(res);
      if (id == -1) break;
      results.push_back(id - 1);
    }
    res_file.close();
    // viewpt_0, [waypts_0], viewpt_1, [waypts_1], viewpt_2, ...
    vector<Eigen::VectorXd> path_viewpoints_;
    vector<vector<Eigen::VectorXd>> path_waypoints_;
    for (int i=0; i<(int)results.size(); ++i)
    {
      path_viewpoints_.push_back(vps[init_id_[results[i]]]);
      if (i < (int)results.size()-1)
      {
        vector<int> retrieval_ = {sub_id_, init_id_[results[i]], init_id_[results[i+1]]};
        vector<Eigen::VectorXd> inter_pts_;
        if ((int)path_waypts_.find(retrieval_)->second.size()>2)
        {
          for (int m=1; m<(int)path_waypts_.find(retrieval_)->second.size()-1; ++m)
            inter_pts_.push_back(path_waypts_.find(retrieval_)->second[m]);
        }
        else
          inter_pts_ = {};
        path_waypoints_.push_back(inter_pts_);
      }
    }
    /* read results end */

    /* record results --> Prior as LowerBound*/
    vector<Eigen::VectorXd> viewpointsFinal;
    vector<vector<Eigen::VectorXd>> waypointsFinal;

    viewpointsFinal = path_viewpoints_;
    waypointsFinal = path_waypoints_;

    local_sub_path_viewpts_[sub_id_] = viewpointsFinal;
    local_sub_path_waypts_[sub_id_] = waypointsFinal;
    /* record results end */
    }
    
    std::unique_lock<std::mutex> lock(mtx);
    con_var.notify_all();
  }  
  /* 
  INPUT : [START, WAYPOINTS, END], size_t : N or [WAYPOINTS], size_t : N-2
  OUTPUT : [UPDATED WAYPOINTS], size_t : N-2 
  */
  void HCSolver::AngleInterpolation(Eigen::VectorXd& start, Eigen::VectorXd& end, vector<Eigen::Vector3d>& waypts_, vector<Eigen::VectorXd>& updates_waypts_)
  {
    int dof = start.size(), angle = dof-3;
    double dist_gap = 0.5;
    double whole_dist = 0.0;
    vector<Eigen::VectorXd> effect_vps_;
    vector<Eigen::VectorXd> seg_posi_;
    double yaw_start = 0.0, yaw_end = 0.0, pitch_start = 0.0, pitch_end = 0.0;
    int cal_flag_yaw = 0, cal_flag_pitch = 0;
    int cal_dir_yaw = 0, cal_dir_pitch = 0;

    if (angle == 1)
    {
      yaw_start = start(3)*180.0/M_PI;
      yaw_end = end(3)*180.0/M_PI;
      cal_flag_yaw = yaw_end - yaw_start > 0? 1:-1;
      cal_dir_yaw = abs(yaw_end - yaw_start)>180.0? -1:1;
    }
    else
    {
      yaw_start = start(4)*180.0/M_PI;
      yaw_end = end(4)*180.0/M_PI;
      cal_flag_yaw = yaw_end - yaw_start > 0? 1:-1;
      cal_dir_yaw = abs(yaw_end - yaw_start)>180.0? -1:1;
      pitch_start = start(3)*180.0/M_PI;
      pitch_end = end(3)*180.0/M_PI;
      cal_flag_pitch = ((pitch_end - pitch_start) > 0)? 1:-1;
      cal_dir_pitch = abs(pitch_end - pitch_start)>180.0? -1:1;
    }
    
    effect_vps_.push_back(start);
    for (auto x:waypts_)
    {
      if ((x-start.head(3)).norm() > dist_gap && (x-end.head(3)).norm() > dist_gap)
      {
        Eigen::VectorXd aug_x; aug_x.resize(dof);
        aug_x(0) = x(0); aug_x(1) = x(1); aug_x(2) = x(2);
        effect_vps_.push_back(aug_x);
      }
    }
    effect_vps_.push_back(end);

    seg_posi_.push_back(start);
    seg_posi_.insert(seg_posi_.end(), effect_vps_.begin(), effect_vps_.end());
    seg_posi_.push_back(end);
    for (int i=0; i<(int)seg_posi_.size()-1; ++i)
      whole_dist += (seg_posi_[i+1].head(3)-seg_posi_[i].head(3)).norm();
    
    if ((int)effect_vps_.size() > 0)
    {
      double yaw_gap = abs(yaw_end - yaw_start)>180.0? (360.0-abs(yaw_end - yaw_start)):abs(yaw_end - yaw_start);
      if (angle == 1)
      {
        for (int i=1; i<(int)effect_vps_.size()-1; ++i)
        {
          double e_dist = (effect_vps_[i].head(3)-start.head(3)).norm();
          effect_vps_[i](3) = (yaw_start + cal_dir_yaw*cal_flag_yaw*yaw_gap*e_dist/whole_dist)*M_PI/180.0;
          while (effect_vps_[i](3) < -M_PI)
            effect_vps_[i](3) += 2 * M_PI;
          while (effect_vps_[i](3) > M_PI)
            effect_vps_[i](3) -= 2 * M_PI;

          updates_waypts_.push_back(effect_vps_[i]);
        }
      }
      else
      {
        double pitch_gap = abs(pitch_end - pitch_start)>180.0? (360.0-abs(pitch_end - pitch_start)):abs(pitch_end - pitch_start);
        for (int i=1; i<(int)effect_vps_.size()-1; ++i)
        {
          double e_dist = (effect_vps_[i].head(3)-start.head(3)).norm();
          effect_vps_[i](3) = (pitch_start + cal_dir_pitch*cal_flag_pitch*pitch_gap*e_dist/whole_dist)*M_PI/180.0;
          effect_vps_[i](4) = (yaw_start + cal_dir_yaw*cal_flag_yaw*yaw_gap*e_dist/whole_dist)*M_PI/180.0;
          while (effect_vps_[i](3) < -M_PI)
            effect_vps_[i](3) += 2 * M_PI;
          while (effect_vps_[i](3) > M_PI)
            effect_vps_[i](3) -= 2 * M_PI;
          while (effect_vps_[i](4) < -M_PI)
            effect_vps_[i](4) += 2 * M_PI;
          while (effect_vps_[i](4) > M_PI)
            effect_vps_[i](4) -= 2 * M_PI;
          
          updates_waypts_.push_back(effect_vps_[i]);
        }
      }
    }
  }
  
  void HCSolver::ConstructKDTree(vector<Eigen::VectorXd>& pathlist, pcl::KdTreeFLANN<pcl::PointXYZ>& tree)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr PathCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ p;
    for (auto node:pathlist)
    {
      p.x = node(0); p.y = node(1); p.z = node(2);
      PathCloud->points.push_back(p);
    }
    tree.setInputCloud(PathCloud);
  }
  /* time cost for k-opt */
  double HCSolver::kOptTimeCost(vector<Eigen::Vector3d>& path)
  {
    double cost = 0.0;
    if ((int)path.size() > 2)
    {
      cost += (path[1]-path[0]).norm()/vm_;
      for (int i=0; i<(int)path.size()-1; ++i)
        cost += pathTimeUniAcc(path[i-1], path[i], path[i+1]);
    }
    else
    {
      cost = (path[1]-path[0]).norm()/vm_;
    }

    return cost;
  }

  /* [s1, s2] [s3, s4] --> [s1, s4] [s2, s3] */
  /* Reverse neighbor: s1, s3, NoReverse neighbor: s2, s4 */
  void HCSolver::Make2Opt(Site *s1, Site *s2, Site *s3, Site *s4, bool turn)
  {
    // determine s4
    // Site *s4 = new Site;
    Site *Last = new Site;
    Site *Next = new Site;
    Site *ReverseStart = new Site;
    Site *ReverseEnd = new Site;

    Eigen::Vector3d A, B, C, D, E, F, G, H;
    bool seqF = true, endF = true;
    if (s2 == s1->Suc)
    {
      /* A->B->s1->s2->C->D, E->F->s4->s3->G->H */
      s4 = s3->Pred;
      seqF = true;
      A << s1->Pred->Pred->X, s1->Pred->Y, s1->Pred->Z;
      B << s1->Pred->X, s1->Pred->Y, s1->Pred->Z;
      C << s2->Suc->X, s2->Suc->Y, s2->Suc->Z;
      D << s2->Suc->Suc->X, s2->Suc->Suc->Y, s2->Suc->Suc->Z;
      E << s4->Pred->Pred->X, s4->Pred->Pred->Y, s4->Pred->Pred->Z;
      F << s4->Pred->X, s4->Pred->Y, s4->Pred->Z;
      G << s3->Suc->X, s3->Suc->Y, s3->Suc->Z;
      H << s3->Suc->Suc->X, s3->Suc->Suc->Y, s3->Suc->Suc->Z;
    }
    if (s2 == s1->Pred)
    {
      /* D->C->s2->s1->B->A, H->G->s3->s4->F->E */
      s4 = s3->Suc;
      seqF = false;
      A << s1->Suc->Suc->X, s1->Suc->Suc->Y, s1->Suc->Suc->Z;
      B << s1->Suc->X, s1->Suc->Y, s1->Suc->Z;
      C << s2->Pred->X, s2->Pred->Y, s2->Pred->Z;
      D << s2->Pred->Pred->X, s2->Pred->Pred->Y, s2->Pred->Pred->Z;
      E << s4->Suc->Suc->X, s4->Suc->Suc->Y, s4->Suc->Suc->Z;
      F << s4->Suc->X, s4->Suc->Y, s4->Suc->Z;
      G << s3->Pred->X, s3->Pred->Y, s3->Pred->Z;
      H << s3->Pred->Pred->X, s3->Pred->Pred->Y, s3->Pred->Pred->Z;
    }
    // determine edge endpoints
    vector<Site*> swapSet = {s1, s2, s3, s4};
    Site *start = new Site; Site *end = new Site;
    start = AllPathSite.front();
    end = AllPathSite.back();
    int sflag = 0;
    Last = start;
    while (sflag != 2)
    {
      Next = Last->Suc;
      int curid = Next->GlobalID;
      for (auto s:swapSet)
      {
        if (curid == s->GlobalID)
          sflag++;
      }
      Last = Next;
    }
    ReverseStart = Last;

    sflag = 0;
    Last = end;
    while (sflag != 2)
    {
      Next = Last->Pred;
      int curid = Next->GlobalID;
      for (auto s:swapSet)
      {
        if (curid == s->GlobalID)
          sflag++;
      }
      Last = Next;
    }
    ReverseEnd = Last;
    if (s2 == ReverseStart || s2 == ReverseEnd)
      endF = true;
    if (s1 == ReverseStart || s1 == ReverseEnd)
      endF = false;
    // calculate gain
    double beforeCost, afterCost, Gain;
    Eigen::Vector3d x1, x2, x3, x4;
    x1 << s1->X, s1->Y, s1->Z;
    x2 << s2->X, s2->Y, s2->Z;
    x3 << s3->X, s3->Y, s3->Z;
    x4 << s4->X, s4->Y, s4->Z;

    double yaw_time_b1, yaw_time_b2;
    double pitch_time_b1, pitch_time_b2;
    yaw_time_b1 = min(abs(s1->Yaw-s2->Yaw), 2 * M_PI - abs(s1->Yaw-s2->Yaw))/yd_;
    yaw_time_b2 = min(abs(s3->Yaw-s4->Yaw), 2 * M_PI - abs(s3->Yaw-s4->Yaw))/yd_;
    pitch_time_b1 = min(abs(s1->Pitch-s2->Pitch), 2 * M_PI - abs(s1->Pitch-s2->Pitch))/yd_;
    pitch_time_b2 = min(abs(s3->Pitch-s4->Pitch), 2 * M_PI - abs(s3->Pitch-s4->Pitch))/yd_;
    // beforeCost = max((x1-x2).norm()/vm_, yaw_time_b1) + max((x3-x4).norm()/vm_, yaw_time_b2);
    beforeCost = max(max((x1-x2).norm()/vm_, yaw_time_b1), pitch_time_b1) + max(max((x3-x4).norm()/vm_, yaw_time_b2), pitch_time_b2);

    double yaw_time_a1, yaw_time_a2;
    double pitch_time_a1, pitch_time_a2;
    yaw_time_a1 = min(abs(s1->Yaw-s4->Yaw), 2 * M_PI - abs(s1->Yaw-s4->Yaw))/yd_;
    yaw_time_a2 = min(abs(s2->Yaw-s3->Yaw), 2 * M_PI - abs(s2->Yaw-s3->Yaw))/yd_;
    pitch_time_a1 = min(abs(s1->Pitch-s4->Pitch), 2 * M_PI - abs(s1->Pitch-s4->Pitch))/yd_;
    pitch_time_a2 = min(abs(s2->Pitch-s3->Pitch), 2 * M_PI - abs(s2->Pitch-s3->Pitch))/yd_;
    // afterCost = max((x1-x4).norm()/vm_, yaw_time_a1)+ max((x2-x3).norm()/vm_, yaw_time_a2);
    afterCost = max(max((x1-x4).norm()/vm_, yaw_time_a1), pitch_time_a1) + max(max((x2-x3).norm()/vm_, yaw_time_a2), pitch_time_a2);

    Gain = beforeCost - afterCost;
    // swap
    if (Gain > 0)
    {
      bool swap = false;
      if (turn == true)
      {
        double dist_gain = 0.0;
        vector<Eigen::Vector3d> before_path;
        vector<Eigen::Vector3d> after_path;
        vector<Eigen::Vector3d> turn_path_1, turn_path_2, turn_path_3, turn_path_4;
        double cost_1 = 0.0, cost_2 = 0.0, cost_3 = 0.0, cost_4 = 0.0;
        double angle1, angle2, angle3, angle4;
        if (seqF == true)
        {
          if (endF == true)
          {
            /* Before: B->s1->s2->C->D + F->s4->s3->G->H */
            /* After:  B->s1->s4->F->E + C->s2->s3->G->H */
            before_path.clear();
            if (yaw_time_b1 > pitch_time_b1)
            {
              angle1 = s1->Yaw;
              angle2 = s2->Yaw;
            }
            else
            {
              angle1 = s1->Pitch;
              angle2 = s2->Pitch;
            }
            opt_dist = computeTimeCost(x1, x2, angle1, angle2, before_path);
            turn_path_1.push_back(B);
            turn_path_1.insert(turn_path_1.end(), before_path.begin(), before_path.end());
            turn_path_1.push_back(C);
            turn_path_1.push_back(D);
            cost_1 = kOptTimeCost(turn_path_1);
            dist_gain += opt_dist;

            before_path.clear();
            if (yaw_time_b2 > pitch_time_b2)
            {
              angle3 = s3->Yaw;
              angle4 = s4->Yaw;
            }
            else
            {
              angle3 = s3->Pitch;
              angle4 = s4->Pitch;
            }
            opt_dist = computeTimeCost(x4, x3, angle4, angle3, before_path);
            turn_path_2.push_back(F);
            turn_path_2.insert(turn_path_2.end(), before_path.begin(), before_path.end());
            turn_path_2.push_back(G);
            turn_path_2.push_back(H);
            cost_2 = kOptTimeCost(turn_path_2);
            dist_gain += opt_dist;

            after_path.clear();
            if (yaw_time_a1 > pitch_time_a1)
            {
              angle1 = s1->Yaw;
              angle4 = s4->Yaw;
            }
            else
            {
              angle1 = s1->Pitch;
              angle4 = s4->Pitch;
            }
            opt_dist = computeTimeCost(x1, x4, angle1, angle4, after_path);
            turn_path_3.push_back(B);
            turn_path_3.insert(turn_path_3.end(), after_path.begin(), after_path.end());
            turn_path_3.push_back(F);
            turn_path_3.push_back(E);
            cost_3 = kOptTimeCost(turn_path_3);
            dist_gain -= opt_dist;

            after_path.clear();
            if (yaw_time_a2 > pitch_time_a2)
            {
              angle2 = s2->Yaw;
              angle3 = s3->Yaw;
            }
            else
            {
              angle2 = s2->Pitch;
              angle3 = s3->Pitch;
            }
            opt_dist = computeTimeCost(x2, x3, angle2, angle3, after_path);
            turn_path_4.push_back(C);
            turn_path_4.insert(turn_path_4.end(), after_path.begin(), after_path.end());
            turn_path_4.push_back(G);
            turn_path_4.push_back(H);
          }
          else
          {
            /* Before: B->s1->s2->C->D + F->s4->s3->G->H */
            /* After:  F->s4->s1->B->A + G->s3->s2->C->D */
            before_path.clear();
            if (yaw_time_b1 > pitch_time_b1)
            {
              angle1 = s1->Yaw;
              angle2 = s2->Yaw;
            }
            else
            {
              angle1 = s1->Pitch;
              angle2 = s2->Pitch;
            }
            opt_dist = computeTimeCost(x1, x2, angle1, angle2, before_path);
            turn_path_1.push_back(B);
            turn_path_1.insert(turn_path_1.end(), before_path.begin(), before_path.end());
            turn_path_1.push_back(C);
            turn_path_1.push_back(D);
            cost_1 = kOptTimeCost(turn_path_1);
            dist_gain += opt_dist;

            before_path.clear();
            if (yaw_time_b2 > pitch_time_b2)
            {
              angle3 = s3->Yaw;
              angle4 = s4->Yaw;
            }
            else
            {
              angle3 = s3->Pitch;
              angle4 = s4->Pitch;
            }
            opt_dist = computeTimeCost(x4, x3, angle4, angle3, before_path);
            turn_path_2.push_back(F);
            turn_path_2.insert(turn_path_2.end(), before_path.begin(), before_path.end());
            turn_path_2.push_back(G);
            turn_path_2.push_back(H);
            cost_2 = kOptTimeCost(turn_path_2);
            dist_gain += opt_dist;

            after_path.clear();
            if (yaw_time_a1 > pitch_time_a1)
            {
              angle1 = s1->Yaw;
              angle4 = s4->Yaw;
            }
            else
            {
              angle1 = s1->Pitch;
              angle4 = s4->Pitch;
            }
            opt_dist = computeTimeCost(x4, x1, angle4, angle1, after_path);
            turn_path_3.push_back(F);
            turn_path_3.insert(turn_path_3.end(), after_path.begin(), after_path.end());
            turn_path_3.push_back(B);
            turn_path_3.push_back(A);
            cost_3 = kOptTimeCost(turn_path_3);
            dist_gain -= opt_dist;

            after_path.clear();
            if (yaw_time_a2 > pitch_time_a2)
            {
              angle2 = s2->Yaw;
              angle3 = s3->Yaw;
            }
            else
            {
              angle2 = s2->Pitch;
              angle3 = s3->Pitch;
            }
            opt_dist = computeTimeCost(x3, x2, angle3, angle2, after_path);
            turn_path_4.push_back(G);
            turn_path_4.insert(turn_path_4.end(), after_path.begin(), after_path.end());
            turn_path_4.push_back(C);
            turn_path_4.push_back(D);
            cost_4 = kOptTimeCost(turn_path_4);
            dist_gain -= opt_dist;
          }
        }
        else
        {
          if (endF == true)
          {
            /* Before: C->s2->s1->B->A + G->s3->s4->F->E */
            /* After:  F->s4->s1->B->A + G->s3->s2->C->D */
            before_path.clear();
            if (yaw_time_b1 > pitch_time_b1)
            {
              angle1 = s1->Yaw;
              angle2 = s2->Yaw;
            }
            else
            {
              angle1 = s1->Pitch;
              angle2 = s2->Pitch;
            }
            opt_dist = computeTimeCost(x2, x1, angle2, angle1, before_path);
            turn_path_1.push_back(C);
            turn_path_1.insert(turn_path_1.end(), before_path.begin(), before_path.end());
            turn_path_1.push_back(B);
            turn_path_1.push_back(A);
            cost_1 = kOptTimeCost(turn_path_1);
            dist_gain += opt_dist;

            before_path.clear();
            if (yaw_time_b2 > pitch_time_b2)
            {
              angle3 = s3->Yaw;
              angle4 = s4->Yaw;
            }
            else
            {
              angle3 = s3->Pitch;
              angle4 = s4->Pitch;
            }
            opt_dist = computeTimeCost(x3, x4, angle3, angle4, before_path);
            turn_path_2.push_back(G);
            turn_path_2.insert(turn_path_2.end(), before_path.begin(), before_path.end());
            turn_path_2.push_back(F);
            turn_path_2.push_back(E);
            cost_2 = kOptTimeCost(turn_path_2);
            dist_gain += opt_dist;

            after_path.clear();
            if (yaw_time_a1 > pitch_time_a1)
            {
              angle1 = s1->Yaw;
              angle4 = s4->Yaw;
            }
            else
            {
              angle1 = s1->Pitch;
              angle4 = s4->Pitch;
            }
            opt_dist = computeTimeCost(x4, x1, angle4, angle1, after_path);
            turn_path_3.push_back(F);
            turn_path_3.insert(turn_path_3.end(), after_path.begin(), after_path.end());
            turn_path_3.push_back(B);
            turn_path_3.push_back(A);
            cost_3 = kOptTimeCost(turn_path_3);
            dist_gain -= opt_dist;

            after_path.clear();
            if (yaw_time_a2 > pitch_time_a2)
            {
              angle2 = s2->Yaw;
              angle3 = s3->Yaw;
            }
            else
            {
              angle2 = s2->Pitch;
              angle3 = s3->Pitch;
            }
            opt_dist = computeTimeCost(x3, x2, angle3, angle2, after_path);
            turn_path_4.push_back(G);
            turn_path_4.insert(turn_path_4.end(), after_path.begin(), after_path.end());
            turn_path_4.push_back(C);
            turn_path_4.push_back(D);
            cost_4 = kOptTimeCost(turn_path_4);
            dist_gain -= opt_dist;
          }
          else
          {
            /* Before: C->s2->s1->B->A + G->s3->s4->F->E */
            /* After:  B->s1->s4->F->E + C->s2->s3->G->H */
            before_path.clear();
            if (yaw_time_b1 > pitch_time_b1)
            {
              angle1 = s1->Yaw;
              angle2 = s2->Yaw;
            }
            else
            {
              angle1 = s1->Pitch;
              angle2 = s2->Pitch;
            }
            opt_dist = computeTimeCost(x2, x1, angle2, angle1, before_path);
            turn_path_1.push_back(C);
            turn_path_1.insert(turn_path_1.end(), before_path.begin(), before_path.end());
            turn_path_1.push_back(B);
            turn_path_1.push_back(A);
            cost_1 = kOptTimeCost(turn_path_1);
            dist_gain += opt_dist;
            
            before_path.clear();
            if (yaw_time_b2 > pitch_time_b2)
            {
              angle3 = s3->Yaw;
              angle4 = s4->Yaw;
            }
            else
            {
              angle3 = s3->Pitch;
              angle4 = s4->Pitch;
            }
            opt_dist = computeTimeCost(x3, x4, angle3, angle4, before_path);
            turn_path_2.push_back(G);
            turn_path_2.insert(turn_path_2.end(), before_path.begin(), before_path.end());
            turn_path_2.push_back(F);
            turn_path_2.push_back(E);
            cost_2 = kOptTimeCost(turn_path_2);
            dist_gain += opt_dist;

            after_path.clear();
            if (yaw_time_a1 > pitch_time_a1)
            {
              angle1 = s1->Yaw;
              angle4 = s4->Yaw;
            }
            else
            {
              angle1 = s1->Pitch;
              angle4 = s4->Pitch;
            }
            opt_dist = computeTimeCost(x1, x4, angle1, angle4, after_path);
            turn_path_3.push_back(B);
            turn_path_3.insert(turn_path_3.end(), after_path.begin(), after_path.end());
            turn_path_3.push_back(F);
            turn_path_3.push_back(E);
            cost_3 = kOptTimeCost(turn_path_3);
            dist_gain -= opt_dist;

            after_path.clear();
            if (yaw_time_a2 > pitch_time_a2)
            {
              angle2 = s2->Yaw;
              angle3 = s3->Yaw;
            }
            else
            {
              angle2 = s2->Pitch;
              angle3 = s3->Pitch;
            }
            opt_dist = computeTimeCost(x2, x3, angle2, angle3, after_path);
            turn_path_4.push_back(C);
            turn_path_4.insert(turn_path_4.end(), after_path.begin(), after_path.end());
            turn_path_4.push_back(G);
            turn_path_4.push_back(H);
            cost_4 = kOptTimeCost(turn_path_4);
            dist_gain -= opt_dist;
          }
        }
        double phy_gain = cost_1+cost_2-cost_3-cost_4;
        if (phy_gain > 0 || dist_gain > 0)
        // if (dist_gain > 0)
        {
          swap = true;
        }
      }
      else
      {
        double angle1, angle2, angle3, angle4;
        if (yaw_time_b1 > pitch_time_b1)
        {
          angle1 = s1->Yaw;
          angle2 = s2->Yaw;
        }
        else
        {
          angle1 = s1->Pitch;
          angle2 = s2->Pitch;
        }
        vector<Eigen::Vector3d> before_path;
        double dist_1 = compute_Cost(x1, x2, angle1, angle2, before_path);
        before_path.clear();
        if (yaw_time_b2 > pitch_time_b2)
        {
          angle3 = s3->Yaw;
          angle4 = s4->Yaw;
        }
        else
        {
          angle3 = s3->Pitch;
          angle4 = s4->Pitch;
        }
        double dist_2 = compute_Cost(x3, x4, angle3, angle4, before_path);

        vector<Eigen::Vector3d> after_path;
        if (yaw_time_a1 > pitch_time_a1)
        {
          angle1 = s1->Yaw;
          angle4 = s4->Yaw;
        }
        else
        {
          angle1 = s1->Pitch;
          angle4 = s4->Pitch;
        }
        double dist_3 = compute_Cost(x1, x4, angle1, angle4, after_path);
        after_path.clear();
        if (yaw_time_a2 > pitch_time_a2)
        {
          angle2 = s2->Yaw;
          angle3 = s3->Yaw;
        }
        else
        {
          angle2 = s2->Pitch;
          angle3 = s3->Pitch;
        }
        double dist_4 = compute_Cost(x2, x3, angle2, angle3, after_path);
        double phy_gain = dist_1+dist_2-dist_3-dist_4;
        if (phy_gain > 0)
          swap = true;
      }

      if (swap == true)
      {
      swapTimes++;
      // apply the 2-opt move by reversing the order of the edges between the endpoints
      //* e.g. start -> ... -> s1 -> s2 -> ... -> s4 -> s3 -> ... -> end 
      //* Reverse tour between s2 and s4
      if (s2 == s1->Suc)
      {
        s1->Suc = s4;
        s2->Pred = s3;
        s3->Pred = s2;
        s4->Suc = s1;
      }
      if (s2 == s1->Pred)
      {
        s1->Pred = s4;
        s2->Suc = s3;
        s3->Suc = s2;
        s4->Pred = s1;
      }

      Site *TempPred = new Site;
      Site *TempSuc = new Site;
      Site *Terminal = new Site;

      if (ReverseStart == ReverseEnd->Pred)
      {
        TempPred = ReverseStart->Pred;
        TempSuc = ReverseStart->Suc;
        ReverseStart->Pred = TempSuc;
        ReverseStart->Suc = TempPred;

        TempPred = ReverseEnd->Pred;
        TempSuc = ReverseEnd->Suc;
        ReverseEnd->Pred = TempSuc;
        ReverseEnd->Suc = TempPred;
      }
      else
      {
        Last = ReverseStart;
        Terminal = ReverseEnd->Suc;
        while (Last != Terminal)
        {
          Next = Last->Suc;
          TempPred = Last->Pred;
          TempSuc = Last->Suc;
          Last->Pred = TempSuc;
          Last->Suc = TempPred;
          Last = Next;
        }
      }
      } 
    }
  }

  /* [s1, s2] [s3, s4] [s5, s6] --> (1/8)*transformation */
  void HCSolver::Make3Opt(Site *s1, Site *s2, Site *s3, Site *s4, Site *s5, Site *s6, bool turn)
  {
    std::random_device rdprob2;
    std::mt19937 genprob2(rdprob2());
    std::uniform_real_distribution<> disprob2(0.0, 1.0);
    double prob2 = disprob2(genprob2);
    if (prob2 > 0.5)
    {
      Make2Opt(s1, s2, s3, s4, turn);

      std::random_device rdprob3;
      std::mt19937 genprob3(rdprob3());
      std::uniform_real_distribution<> disprob3(0.0, 1.0);
      double prob3 = disprob3(genprob3);
      if (prob3 > 0.5)
      {
        std::random_device rdprob4;
        std::mt19937 genprob4(rdprob4());
        std::uniform_real_distribution<> disprob4(0.0, 1.0);
        double prob4 = disprob4(genprob4);
        if (prob4 > 0.5)
          s2 = s1->Pred;
        else
          s2 = s1->Suc;
        if (s5 != s2->Pred && s5 != s2->Suc && s5 != s1->Pred && s5 != s1->Suc)
          Make2Opt(s1, s2, s5, s6, turn);
      }
      else
      {
        std::random_device rdprob5;
        std::mt19937 genprob5(rdprob5());
        std::uniform_real_distribution<> disprob5(0.0, 1.0);
        double prob5 = disprob5(genprob5);
        if (prob5 > 0.5)
          s4 = s3->Pred;
        else
          s4 = s3->Suc;
        if (s5 != s4->Pred && s5 != s4->Suc && s5 != s3->Pred && s5 != s3->Suc)
          Make2Opt(s3, s4, s5, s6, turn);
      }
    }
    else
    {
      Make2Opt(s1, s2, s5, s6, turn);

      std::random_device rdprob6;
      std::mt19937 genprob6(rdprob6());
      std::uniform_real_distribution<> disprob6(0.0, 1.0);
      double prob6 = disprob6(genprob6);
      if (prob6 > 0.5)
      {
        std::random_device rdprob7;
        std::mt19937 genprob7(rdprob7());
        std::uniform_real_distribution<> disprob7(0.0, 1.0);
        double prob7 = disprob7(genprob7);
        if (prob7 > 0.5)
          s2 = s1->Pred;
        else
          s2 = s1->Suc;
        if (s3 != s2->Pred && s3 != s2->Suc && s3 != s1->Pred && s3 != s1->Suc)
          Make2Opt(s1, s2, s3, s4, turn);
      }
      else
      {
        std::random_device rdprob8;
        std::mt19937 genprob8(rdprob8());
        std::uniform_real_distribution<> disprob8(0.0, 1.0);
        double prob8 = disprob8(genprob8);
        if (prob8 > 0.5)
          s6 = s5->Pred;
        else
          s6 = s5->Suc;
        if (s3 != s6->Pred && s3 != s6->Suc && s3 != s5->Pred && s3 != s5->Suc)
          Make2Opt(s5, s6, s3, s4, turn);
      }
    }
  }

  void HCSolver::RandomLocal2Opt(bool turn)
  {
    Site *t1 = new Site;
    Site *t2 = new Site;
    Site *t3 = new Site;
    Site *t4 = new Site;
    vector<int> excludeID;
    vector<Site*> remainSite;

    int localNum = LocalSite.size();
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, localNum - 1);
    int randomInt = dis(gen);
    t1 = LocalSite[randomInt];
    excludeID.push_back(t1->LocalID);
    
    std::random_device rdprob;
    std::mt19937 genprob(rdprob());
    std::uniform_real_distribution<> disprob(0.0, 1.0);
    double prob = disprob(genprob);
    if (prob > 0.5)
    {
      /* avoid t3->t2->t1 since t4 will be t2*/
      t2 = t1->Pred;
      excludeID.push_back(t2->LocalID); 
      if (t2->Pred->Start != true && t2->Pred->End != true)
        excludeID.push_back(t2->Pred->LocalID);
    }
    else
    {
      /* avoid t1->t2->t3 since t4 will be t2*/
      t2 = t1->Suc;
      excludeID.push_back(t2->LocalID); 
      if (t2->Suc->Start != true && t2->Suc->End != true)
        excludeID.push_back(t2->Suc->LocalID);
    }
    Eigen::Vector3d posT1; posT1 << t1->X, t1->Y, t1->Z;
    Eigen::Vector3d posT2; posT2 << t2->X, t2->Y, t2->Z;
    Eigen::Vector3d vecT1T2 = (posT2 - posT1).normalized();

    for (auto s:LocalSite)
    {
      bool in = true;
      for (auto ex:excludeID)
      {
        if (s->LocalID == ex)
        {
          in = false;
          break;
        }
      }
      Eigen::Vector3d posCandidate; posCandidate << s->X, s->Y, s->Z;
      if (in == true && (posCandidate-posT2).norm() < 20.0)
      {
        Eigen::Vector3d posCandidatePred; posCandidatePred << s->Pred->X, s->Pred->Y, s->Pred->Z;
        Eigen::Vector3d posCandidateSuc; posCandidateSuc << s->Suc->X, s->Suc->Y, s->Suc->Z;
        Eigen::Vector3d vecCandidatePred = (posCandidate - posCandidatePred).normalized();
        Eigen::Vector3d vecCandidateSuc = (posCandidateSuc - posCandidate).normalized();

        double diffAnglePred = acos(vecT1T2.dot(vecCandidatePred));
        diffAnglePred = min(diffAnglePred, 2 * M_PI - diffAnglePred);
        double diffAngleSuc = acos(vecT1T2.dot(vecCandidateSuc));
        diffAngleSuc = min(diffAngleSuc, 2 * M_PI - diffAngleSuc);
        double diffSel = min(diffAnglePred, diffAngleSuc);

        if (diffSel > M_PI/36.0)
          remainSite.push_back(s);
      }
    }

    if ((int)remainSite.size() > 0)
    {
      int remainNum = remainSite.size();
      random_device rdR;
      mt19937 genR(rdR());
      uniform_int_distribution<> disR(0, remainNum - 1);
      int randomIntRemain = disR(genR);
      t3 = remainSite[randomIntRemain];

      Make2Opt(t1, t2, t3, t4, turn);
    }
  }

  void HCSolver::RandomLocal3Opt(bool turn)
  {
    Site *t1 = new Site;
    Site *t2 = new Site;
    Site *t3 = new Site;
    Site *t4 = new Site;
    Site *t5 = new Site;
    Site *t6 = new Site;
    vector<int> excludeID;
    vector<Site*> remainSite;

    int localNum = LocalSite.size();
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, localNum - 1);
    int randomInt = dis(gen);
    t1 = LocalSite[randomInt];
    excludeID.push_back(t1->LocalID);
    
    std::random_device rdprob;
    std::mt19937 genprob(rdprob());
    std::uniform_real_distribution<> disprob(0.0, 1.0);
    double prob = disprob(genprob);
    
    if (prob > 0.5)
    {
      /* avoid t3->t2->t1 since t4 will be t2*/
      t2 = t1->Pred;
      excludeID.push_back(t2->LocalID); 
      if (t2->Pred->Start != true && t2->Pred->End != true)
        excludeID.push_back(t2->Pred->LocalID);
    }
    else
    {
      /* avoid t1->t2->t3 since t4 will be t2*/
      t2 = t1->Suc;
      excludeID.push_back(t2->LocalID); 
      if (t2->Suc->Start != true && t2->Suc->End != true)
        excludeID.push_back(t2->Suc->LocalID);
    }
    Eigen::Vector3d posT1; posT1 << t1->X, t1->Y, t1->Z;
    Eigen::Vector3d posT2; posT2 << t2->X, t2->Y, t2->Z;
    Eigen::Vector3d vecT1T2 = (posT2 - posT1).normalized();

    for (auto s:LocalSite)
    {
      bool in = true;
      for (auto ex:excludeID)
      {
        if (s->LocalID == ex)
        {
          in = false;
          break;
        }
      }
      Eigen::Vector3d posCandidate; posCandidate << s->X, s->Y, s->Z;
      if (in == true && (posCandidate-posT2).norm() < 20.0)
      {
        Eigen::Vector3d posCandidatePred; posCandidatePred << s->Pred->X, s->Pred->Y, s->Pred->Z;
        Eigen::Vector3d posCandidateSuc; posCandidateSuc << s->Suc->X, s->Suc->Y, s->Suc->Z;
        Eigen::Vector3d vecCandidatePred = (posCandidate - posCandidatePred).normalized();
        Eigen::Vector3d vecCandidateSuc = (posCandidateSuc - posCandidate).normalized();

        double diffAnglePred = acos(vecT1T2.dot(vecCandidatePred));
        diffAnglePred = min(diffAnglePred, 2 * M_PI - diffAnglePred);
        double diffAngleSuc = acos(vecT1T2.dot(vecCandidateSuc));
        diffAngleSuc = min(diffAngleSuc, 2 * M_PI - diffAngleSuc);
        double diffSel = min(diffAnglePred, diffAngleSuc);

        if (diffSel > M_PI/36.0)
          remainSite.push_back(s);
      }
    }

    if ((int)remainSite.size() > 0)
    {
      int remainNum = remainSite.size();
      random_device rdR;
      mt19937 genR(rdR());
      uniform_int_distribution<> disR(0, remainNum - 1);
      int randomIntRemain = disR(genR);
      t3 = remainSite[randomIntRemain];

      /* choose t5 & t6 */
      vector<int> excludeID2;
      vector<Site*> remainSite2;
      excludeID2.push_back(t1->LocalID); 
      if (t1->Pred->Start != true && t1->Pred->End != true)
        excludeID2.push_back(t1->Pred->LocalID);
      if (t1->Suc->Start != true && t1->Suc->End != true)
        excludeID2.push_back(t1->Suc->LocalID);
      excludeID2.push_back(t2->LocalID); 
      if (t2->Pred->Start != true && t2->Pred->End != true)
        excludeID2.push_back(t2->Pred->LocalID);
      if (t2->Suc->Start != true && t2->Suc->End != true)
        excludeID2.push_back(t2->Suc->LocalID);
      excludeID2.push_back(t3->LocalID);
      if (t3->Pred->Start != true && t3->Pred->End != true)
        excludeID2.push_back(t3->Pred->LocalID);
      if (t3->Suc->Start != true && t3->Suc->End != true)
        excludeID2.push_back(t3->Suc->LocalID);

      Eigen::Vector3d posT3; posT3 << t3->X, t3->Y, t3->Z;
      Eigen::Vector3d posT4; posT4 << t3->Pred->X, t3->Pred->Y, t3->Pred->Z;
      Eigen::Vector3d vecT3T4 = (posT4 - posT3).normalized();

      for (auto s:LocalSite)
      {
        bool in = true;
        for (auto ex:excludeID2)
        {
          if (s->LocalID == ex)
          {
            in = false;
            break;
          }
        }
        Eigen::Vector3d posCandidate; posCandidate << s->X, s->Y, s->Z;
        if (in == true && (posCandidate-posT3).norm() < 20.0)
        {
          Eigen::Vector3d posCandidatePred; posCandidatePred << s->Pred->X, s->Pred->Y, s->Pred->Z;
          Eigen::Vector3d posCandidateSuc; posCandidateSuc << s->Suc->X, s->Suc->Y, s->Suc->Z;
          Eigen::Vector3d vecCandidatePred = (posCandidate - posCandidatePred).normalized();
          Eigen::Vector3d vecCandidateSuc = (posCandidateSuc - posCandidate).normalized();

          double diffAnglePred = acos(vecT3T4.dot(vecCandidatePred));
          diffAnglePred = min(diffAnglePred, 2 * M_PI - diffAnglePred);
          double diffAngleSuc = acos(vecT3T4.dot(vecCandidateSuc));
          diffAngleSuc = min(diffAngleSuc, 2 * M_PI - diffAngleSuc);
          double diffSel = min(diffAnglePred, diffAngleSuc);
          
          if (diffSel > M_PI/36.0)
            remainSite2.push_back(s);
        }
      }

      if ((int)remainSite2.size() > 0)
      {
        int remainNum2 = remainSite2.size();
        random_device rdR2;
        mt19937 genR2(rdR2());
        uniform_int_distribution<> disR2(0, remainNum2 - 1);
        int randomIntRemain2 = disR2(genR2);
        t5 = remainSite2[randomIntRemain2];

        Make3Opt(t1, t2, t3, t4, t5, t6, turn);
      }
    }
  }

} // namespace predrecon