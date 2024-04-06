/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the header file of Extra_Del class, which implements the 
 *                   advanced index retrieval of Eigen::Matrix and Eigen::Vector.
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

#ifndef _EXTRA_DEL_H_
#define _EXTRA_DEL_H_

#include<iostream>
#include<algorithm>
#include<Eigen/Eigen>
using namespace std;
using namespace Eigen;

namespace predrecon
{

class Extra_Del
{
public:
	MatrixXd rows_ext_V(VectorXi ind, MatrixXd matrix);
	MatrixXd rows_ext_M(MatrixXd ind, MatrixXd matrix);
	MatrixXd cols_ext_V(VectorXi ind, MatrixXd matrix);
	MatrixXd cols_ext_M(MatrixXd ind, MatrixXd matrix);
	MatrixXd rows_del_M(MatrixXd ind, MatrixXd matrix);
	MatrixXd cols_del_M(MatrixXd ind, MatrixXd matrix);
};

}

#endif