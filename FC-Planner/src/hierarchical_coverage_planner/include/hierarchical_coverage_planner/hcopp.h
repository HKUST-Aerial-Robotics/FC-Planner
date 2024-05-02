/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of HCOPP class, which implements
 *                   the whole process of FC-Planner.
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

#ifndef _HCOPP_H_
#define _HCOPP_H_

#include <active_perception/perception_utils.h>
#include <hierarchical_coverage_planner/hcplanner.h>
#include <hierarchical_coverage_planner/hctraj.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;
using std::unique_ptr;

namespace predrecon
{

struct PointCluster 
{
  double x, y, z;
  int cluster;
};

class PerceptionUtils;

class HCOPP
{
public:
  HCOPP();
  ~HCOPP();
  /* Func */
  void init(ros::NodeHandle& nh);
  void ExecPlan();
  void triggerCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg);
  void flightCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  /* Utils */
  unique_ptr<hierarchical_coverage_planner> PathPlanner_;
  unique_ptr<TrajGenerator> TrajGen_;
  unique_ptr<PerceptionUtils> percep_utils_;
  unique_ptr<PlanningVisualization> VisUtils_;
  ros::Subscriber trigger_sub;
  ros::Subscriber flight_sub;
  /* Param */
  ros::NodeHandle HCOPPnh;
  double compT, freq;
};

}

#endif