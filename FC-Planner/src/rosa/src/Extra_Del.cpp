/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the main algorithm of advanced index retrieval for
 *                   Eigen::Matrix and Eigen::Vector.
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

#include <rosa/Extra_Del.h>

using namespace std;
using namespace Eigen;

namespace predrecon
{

MatrixXd Extra_Del::rows_ext_V(VectorXi ind, MatrixXd matrix){
	MatrixXd zs1(ind.size(), 1);
	zs1 << (ind.head(ind.size())).cast<double>();
	MatrixXd final_matrix(zs1.size(), matrix.cols());
	int num = zs1.size();
	for (int k = 0; k < num; k++)
	{
		final_matrix.row(k) = matrix.row(zs1(k, 0));
	}
	return final_matrix;
}

MatrixXd Extra_Del::rows_ext_M(MatrixXd ind, MatrixXd matrix){
	MatrixXd final_matrix(ind.size(), matrix.cols());
	int num = ind.size();
	for (int k = 0; k < num; k++)
	{
		final_matrix.row(k) = matrix.row(ind(k,0));
	}
	return final_matrix;
}

MatrixXd Extra_Del::cols_ext_V(VectorXi ind, MatrixXd matrix){
	MatrixXd zs1(ind.size(), 1);
	zs1 << (ind.head(ind.size())).cast<double>();
	MatrixXd final_matrix(matrix.rows(), zs1.size());
	int num = zs1.size();
	for (int k = 0; k < num; k++)
	{
		final_matrix.col(k) = matrix.col(zs1(k, 0));
	}
	return final_matrix;
}

MatrixXd Extra_Del::cols_ext_M(MatrixXd ind, MatrixXd matrix){
	MatrixXd final_matrix(matrix.rows(), ind.size());
	int num = ind.size();
	for (int k = 0; k < num; k++)
	{
		final_matrix.col(k) = matrix.col(ind(k, 0));
	}
	return final_matrix;
}

MatrixXd Extra_Del::rows_del_M(MatrixXd ind, MatrixXd matrix){
	int num = matrix.rows();
	VectorXd xl(num);
	for (int i = 0; i < num; i++)
	{
		xl(i) = i;
	}
	for (int i = 0; i < ind.size(); i++)
	{
		xl.coeffRef(ind(i)) = std::numeric_limits<double>::quiet_NaN();
	}
	VectorXd out_index(num - ind.size());
	int index(0);
	for (int i = 0; i < num; i++){
		if (isnan(xl(i)))
		{
			continue;
		}
		else
		{
			out_index(index) = i;
		}
		index++;
	}
	MatrixXd zs1(out_index.size(), 1);
	zs1 << (out_index.head(out_index.size())).cast<double>();
	MatrixXd final_matrix(zs1.size(), matrix.cols());
	int num1 = zs1.size();
	for (int k = 0; k < num1; k++)
	{
		final_matrix.row(k) = matrix.row(zs1(k, 0));
	}
	return final_matrix;
}

MatrixXd Extra_Del::cols_del_M(MatrixXd ind, MatrixXd matrix){
	int num = matrix.rows();
	VectorXd xl(num);
	for (int i = 0; i < num; i++)
	{
		xl(i) = i;
	}
	for (int i = 0; i < ind.size(); i++)
	{
		xl.coeffRef(ind(i)) = std::numeric_limits<double>::quiet_NaN();
	}
	VectorXd out_index(num - ind.size());
	int index(0);
	for (int i = 0; i < num; i++){
		if (isnan(xl(i)))
		{
			continue;
		}
		else
		{
			out_index(index) = i;
		}
		index++;
	}
	MatrixXd zs1(out_index.size(), 1);
	zs1 << (out_index.head(out_index.size())).cast<double>();
	MatrixXd final_matrix(matrix.rows(), zs1.size());
	int num1 = zs1.size();
	for (int k = 0; k < num1; k++)
	{
		final_matrix.col(k) = matrix.col(zs1(k, 0));
	}
	return final_matrix;
}

}