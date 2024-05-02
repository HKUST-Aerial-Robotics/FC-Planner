/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main file of executing FC-Planner.
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

#include <hierarchical_coverage_planner/hcopp.h>

namespace predrecon
{
  HCOPP::HCOPP(){
  }

  HCOPP::~HCOPP(){
  }

  void HCOPP::init(ros::NodeHandle& nh)
  {
    // * Module Initialization
    PathPlanner_.reset(new hierarchical_coverage_planner);
    TrajGen_.reset(new TrajGenerator);
    percep_utils_.reset(new PerceptionUtils);
    VisUtils_.reset(new PlanningVisualization(nh));

    PathPlanner_->init(nh);
    TrajGen_->init(nh);
    percep_utils_->init(nh);

    // * Params Initialization
    HCOPPnh = nh;
    nh.param("hcopp/TimeStamp", compT, -1.0);
    nh.param("hcopp/TrajSampleFrequency", freq, -1.0);
    
    // * Trigger Initialization
    trigger_sub = nh.subscribe("/move_base_simple/goal", 1, &HCOPP::triggerCallback, this);
    flight_sub = nh.subscribe("/initialpose", 1, &HCOPP::flightCallback, this);
    
    ROS_INFO("\033[31m[FC-Planner] Initialized! \033[32m");
  }

  void HCOPP::ExecPlan()
  {
    // * Path Planning
    PathPlanner_->HCPlan(HCOPPnh);
    
    int path_size = PathPlanner_->PR.FullPath_.size();
    Eigen::Vector3d now_posi = PathPlanner_->PR.FullPath_[1].head(3);
    vector<Eigen::Vector3d> wps;
    vector<int> wps_waypt; 
    Eigen::Vector3d inter_wp;
    double hcoppL = (PathPlanner_->PR.FullPath_[2].head(3)-now_posi).norm();
    int wpID = 0;
    for (int i=2; i<(int)path_size; ++i)
    {
      inter_wp = PathPlanner_->PR.FullPath_[i].head(3);
      if (i<path_size-1)
        hcoppL += (PathPlanner_->PR.FullPath_[i+1].head(3)-PathPlanner_->PR.FullPath_[i].head(3)).norm();
      wps.push_back(inter_wp);
      if (PathPlanner_->PR.waypoints_indicators_[i] == true)
        wps_waypt.push_back(wpID);
      wpID++;
    }
    vector<bool> wps_waypt_indicators_(wps.size(), false);
    for (int w=0; w<(int)wps_waypt.size(); ++w)
      wps_waypt_indicators_[wps_waypt[w]] = true;

    vector<double> pitchs;
    vector<double> yaws;
    for (int i=1; i<path_size; ++i)
    {
      yaws.push_back(PathPlanner_->PR.FullPath_[i].tail(1)(0));
    }
    if ((int)PathPlanner_->PR.FullPath_[0].size() == 5)
    {
      for (int i=1; i<path_size; ++i)
      {
        pitchs.push_back(PathPlanner_->PR.FullPath_[i](3));
      }
    }
    
    ROS_INFO("\033[31m[PathAnalyzer] full path size = %d. \033[32m", (int)PathPlanner_->PR.FullPath_.size());
    ROS_INFO("\033[31m[PathAnalyzer] all viewpoints quantity = %d. \033[32m", PathPlanner_->viewpointNum);
    ROS_INFO("\033[31m[PathAnalyzer] all waypoints quantity = %d. \033[32m", (int)wps_waypt.size());
    ROS_INFO("\033[31m[PathAnalyzer] path coverage rate = %lf %%. \033[32m", PathPlanner_->coverage*100.0);
    ROS_INFO("\033[31m[PathAnalyzer] path length = %lf m.\033[32m", hcoppL);
    ROS_INFO("\033[31m[PathAnalyzer] system computation latency = %lf ms.\033[32m", PathPlanner_->hcoppCT);
    
    // * Motion Planning 
    TrajGen_->HCTraj(now_posi, wps, wps_waypt_indicators_, pitchs, yaws, PathPlanner_->voxelMap, PathPlanner_->corridorProgress, 1.3);
    ROS_INFO("\033[35m[Traj] --- <Trajectory Generation finished> --- \033[35m");
    
    // * FC-Planner Trajectory Coverage Evaluation
    TrajGen_->outputTraj(freq);
    double coverageRate = PathPlanner_->CoverageEvaluation(TrajGen_->TrajPose);
    TrajGen_->outputCloud(PathPlanner_->visible_group, freq);
    ROS_INFO("\033[36m[CoverageAnalyzer] FC-Planner trajectory coverage rate = %lf %%. \033[32m", coverageRate*100.0);
  }

  void HCOPP::triggerCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
  {
    PathPlanner_->skeleton_operator->visFlag = true;
    PathPlanner_->visFlag = true;
    TrajGen_->visFlag = true;
    ExecPlan();
  }

  void HCOPP::flightCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    ROS_INFO("\033[33m[Flight] START================================= \033[32m");
    double totalTime = TrajGen_->minco_traj.getTotalDuration();
    int timeID = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr incrementalPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr CurrentPoints (new pcl::PointCloud<pcl::PointXYZ>);
    double speed = 0.1*freq;
    for (double t = 0.0; t < totalTime; t += freq)
    {
      Eigen::Vector3d X = TrajGen_->minco_traj.getPos(t);
      double pitch = 0.0, yaw = 0.0;
      if ((int)PathPlanner_->PR.FullPath_[0].size() == 5)
        TrajGen_->getPitch(t, pitch);
      TrajGen_->getYaw(t, yaw);
      vector<Eigen::Vector3d> travelPos;
      travelPos.push_back(X);
      percep_utils_->setPose_PY(X, pitch, yaw);
      vector<Eigen::Vector3d> l1, l2;
      percep_utils_->getFOV_PY(l1, l2);
      incrementalPoints = PathPlanner_->visible_group[timeID];
      *CurrentPoints += *incrementalPoints;
       
      VisUtils_->publishCurrentFoV(l1, l2, yaw);
      VisUtils_->publishTravelTraj(travelPos, 0.4, Eigen::Vector4d(0, 0, 1, 1), timeID);
      VisUtils_->publishVisiblePoints(CurrentPoints, timeID);
      ros::Duration(speed).sleep();
      timeID++;
    }
    ROS_INFO("\033[33m[Flight] END================================= \033[32m");
  }
}