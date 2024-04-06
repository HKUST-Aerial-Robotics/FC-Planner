/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the structure of datawrapper for plane query.
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


#ifndef _DATAWRAPPER_H_
#define _DATAWRAPPER_H_

#include <assert.h>
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

namespace predrecon
{

class DataWrapper{
    private: 
    double*   	     data;
    int       		 npoints;
    const static int ndim = 3; 
        
    public:
        
    void factory( double* data, int npoints ){
        this->data 	  = data;
        this->npoints = npoints;
    }
    /** 
     *  Data retrieval function
     *  @param a address over npoints
     *  @param b address over the dimensions
     */
    inline double operator()(int a, int b){
        assert( a<npoints );
        assert( b<ndim );
        return data[ a + npoints*b ];
    }
    // retrieve a single point at offset a, in a vector (preallocated structure)
    inline void operator()(int a, vector<double>& p){
        assert( a<npoints );
        assert( (int)p.size() == ndim );
        p[0] = data[ a + 0*npoints ];
        p[1] = data[ a + 1*npoints ];
        p[2] = data[ a + 2*npoints ];
    }
    int length(){
        return this->npoints;
    }
};

}

#endif