/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main algorithm of safe path searching 
 *                   using A* algorithm.
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

#include <path_searching/astar2.h>

using namespace std;
using namespace Eigen;

namespace predrecon {
Astar::Astar() {
}

Astar::~Astar() {
  for (int i = 0; i < allocate_num_; i++)
    delete path_node_pool_[i];
}

void Astar::init_hc(ros::NodeHandle& nh, const SDFMap::Ptr& hc_map_)
{
  nh.param("astar/resolution_astar", resolution_, -1.0);
  nh.param("astar/lambda_heu_hc", lambda_heu_hc_, -1.0);
  nh.param("astar/max_search_time", max_search_time_, -1.0);
  nh.param("astar/allocate_num", allocate_num_, -1);
  nh.param("viewpoint_manager/zGround", zFlag, false);
  nh.param("viewpoint_manager/GroundPos", groundz, -1.0);
  nh.param("viewpoint_manager/safeHeight", safeheight, -1.0);

  tie_breaker_ = 1.0 + 1.0 / 1000;
  this->map_hc_ = hc_map_;
  /* --- params --- */
  this->inv_resolution_ = 1.0 / resolution_;
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new Node;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
  early_terminate_cost_ = 0.0;

}

void Astar::setResolution(const double& res) {
  resolution_ = res;
  this->inv_resolution_ = 1.0 / resolution_;
}

int Astar::hc_search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt)
{ 
  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  map_hc_->posToIndex_hc(start_pt, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = lambda_heu_hc_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  map_hc_->posToIndex_hc(end_pt, end_index);

  open_set_.push(cur_node);
  open_set_map_.insert(make_pair(cur_node->index, cur_node));
  use_node_num_ += 1;

  const auto t1 = ros::Time::now();
  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;
    if (reach_end) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    // Early termination if time up
    if ((ros::Time::now() - t1).toSec() > max_search_time_) {
      // std::cout << "early" << endl;
      early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
      return NO_PATH;
    }

    open_set_.pop();
    open_set_map_.erase(cur_node->index);
    close_set_map_.insert(make_pair(cur_node->index, 1));
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;
    Eigen::Vector3d stepIdx;

    for (int dx = -1; dx <= 1; dx += 1)
      for (int dy = -1; dy <= 1; dy += 1)
        for (int dz = -1; dz <= 1; dz += 1) 
        {
          stepIdx << dx, dy, dz;
          if (stepIdx.norm() < 1e-3) continue;
          nbr_pos = cur_pos;
          Vector3i nbr_safe;
          map_hc_->posToIndex_hc(nbr_pos, nbr_safe);
          nbr_safe(0) = nbr_safe(0) + stepIdx(0);
          nbr_safe(1) = nbr_safe(1) + stepIdx(1);
          nbr_safe(2) = nbr_safe(2) + stepIdx(2);
          map_hc_->indexToPos_hc(nbr_safe, nbr_pos);
          // Check safety
          if (zFlag == true)
          {
            if (nbr_pos(2) < groundz + safeheight)
              continue;
          }
          if (!map_hc_->isInMap_hc(nbr_safe)) continue;
          if (map_hc_->get_Internal(nbr_safe) == SDFMap::OCCUPANCY::HC_INTERNAL || (int)map_hc_->hcmd_->occupancy_buffer_hc_[map_hc_->toAddress_hc(nbr_safe)] == 1 || (int) map_hc_->hcmd_->occupancy_inflate_buffer_hc_[map_hc_->toAddress_hc(nbr_safe)] == 1)
          {
            continue;
          }

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1*len; l < len; l += 0.1*len) {
            Vector3d ckpt = cur_pos + l * dir;
            Vector3i ckpt_idx_;
            map_hc_->posToIndex_hc(ckpt, ckpt_idx_);
            if (zFlag == true)
            {
              if (ckpt(2) < groundz + safeheight)
              {
                safe = false;
                break;
              }
            }
            if (map_hc_->get_Internal(ckpt) == SDFMap::OCCUPANCY::HC_INTERNAL || (int)map_hc_->hcmd_->occupancy_buffer_hc_[map_hc_->toAddress_hc(ckpt_idx_)] == 1 || (int) map_hc_->hcmd_->occupancy_inflate_buffer_hc_[map_hc_->toAddress_hc(ckpt_idx_)] == 1) 
            {
              safe = false;
              break;
            }
          }
          if (!safe) continue;

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          map_hc_->posToIndex_hc(nbr_pos, nbr_idx);
          if (close_set_map_.find(nbr_idx) != close_set_map_.end()) continue;

          NodePtr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          auto node_iter = open_set_map_.find(nbr_idx);
          if (node_iter == open_set_map_.end()) {
            neighbor = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              // cout << "run out of node pool." << endl;
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < node_iter->second->g_score) {
            neighbor = node_iter->second;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + lambda_heu_hc_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          open_set_map_[nbr_idx] = neighbor;
        }
  }
  // cout << "open set empty, no path!" << endl;
  // cout << "use node num: " << use_node_num_ << endl;
  // cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

double Astar::getEarlyTerminateCost() {
  return early_terminate_cost_;
}

void Astar::reset() {
  open_set_map_.clear();
  close_set_map_.clear();
  path_nodes_.clear();

  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
  open_set_.swap(empty_queue);
  for (int i = 0; i < use_node_num_; i++) {
    path_node_pool_[i]->parent = NULL;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
}

double Astar::pathLength(const vector<Eigen::Vector3d>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;
  for (int i = 0; i < (int)path.size() - 1; ++i)
    length += (path[i + 1] - path[i]).norm();
  return length;
}

void Astar::backtrack(const NodePtr& end_node, const Eigen::Vector3d& end) {
  path_nodes_.push_back(end);
  path_nodes_.push_back(end_node->position);
  NodePtr cur_node = end_node;
  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node->position);
  }
  reverse(path_nodes_.begin(), path_nodes_.end());
}

std::vector<Eigen::Vector3d> Astar::getPath() {
  return path_nodes_;
}

double Astar::getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));
  double h = 0.0;
  double diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return tie_breaker_ * h;
}

double Astar::getManhHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));
  return tie_breaker_ * (dx + dy + dz);
}

double Astar::getEuclHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
  return tie_breaker_ * (x2 - x1).norm();
}

std::vector<Eigen::Vector3d> Astar::getVisited() {
  vector<Eigen::Vector3d> visited;
  for (int i = 0; i < use_node_num_; ++i)
    visited.push_back(path_node_pool_[i]->position);
  return visited;
}

void Astar::posToIndex(const Eigen::Vector3d& pt, Eigen::Vector3i& idx) {
  idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
}

} // namespace predrecon
