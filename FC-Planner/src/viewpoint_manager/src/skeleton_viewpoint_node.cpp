/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the execution file of skeleton-guided viewpoint generation.
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

#include <viewpoint_manager/skeleton_viewpoint.h>
#include <ros/ros.h>

#include <viewpoint_manager/backward.hpp>
namespace backward
{
    backward::SignalHandling sh;
}

using namespace predrecon;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "skeleton_viewpoint_node");
    ros::NodeHandle nh("~");

    SkeletonViewpoint skeleton_vp;
    skeleton_vp.init(nh);
    skeleton_vp.execution(nh);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}