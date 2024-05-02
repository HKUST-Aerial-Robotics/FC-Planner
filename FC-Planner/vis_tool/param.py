# ⭐⭐⭐**********************************************⭐⭐⭐ #
# * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST
# * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
# * Date         :    Mar. 2024
# * E-mail       :    cfengag at connect dot ust dot hk.
# * Description  :    This file is parameter generation script.
# * License      :    GNU General Public License <http://www.gnu.org/licenses/>.
# * Project      :    FC-Planner is free software: you can redistribute it and/or 
# *                   modify it under the terms of the GNU Lesser General Public 
# *                   License as published by the Free Software Foundation, 
# *                   either version 3 of the License, or (at your option) any 
# *                   later version.
# *                   FC-Planner is distributed in the hope that it will be useful,
# *                   but WITHOUT ANY WARRANTY; without even the implied warranty 
# *                   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
# *                   See the GNU General Public License for more details.
# * Website      :    https://hkust-aerial-robotics.github.io/FC-Planner/
# ⭐⭐⭐**********************************************⭐⭐⭐ #

import math
import argparse

def read_txt(file):
    f = open(file, "r")
    data = []
    for line in f:
        x = line.split(", ")
        d = []
        for i in range(1, len(x)):
            y = x[i].split(" ")[1]
            if i == len(x)-1:
                y = y[:-2]
            d.append(y)
        data.append(d)
    f.close()
    
    xyzypr = []
    for k in data:
        yaw = float(k[4])*180.0/math.pi
        while (yaw > 360.0):
            yaw -= 360.0
        while (yaw < 0.0):
            yaw += 360.0
        pitch = float(k[3])*180.0/math.pi
        temp = [k[0], k[1], k[2], str(yaw), str(pitch), str(0.0)]
        xyzypr.append(temp)
    
    return xyzypr

def write_pose(pos_file, traj):
    g = open(pos_file, "w")
    for i in range(len(traj)):
        pos_str = str(i).rjust(4,'0')+".png "+traj[i][0]+" "+traj[i][1]+" "+traj[i][2]+"\n"
        g.write(pos_str)
    g.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--traj_path", type=str,default='./assets/TrajInfo.txt')
    parser.add_argument("--log_path", type=str,default='./assets/FlightLog.txt')
    args = parser.parse_args()

    file = args.traj_path
    pos_txt = args.log_path
    traj = read_txt(file)
    write_pose(pos_txt, traj)