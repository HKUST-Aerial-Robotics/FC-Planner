<div align="center">
    <h1>⚽ FC-Planner</h1>
    <h2>A Skeleton-guided Planning Framework for Fast Aerial Coverage of Complex 3D Scenes</h2>
    <strong>ICRA 2024</strong>
    <br>
        <a href="https://chen-albert-feng.github.io/AlbertFeng.github.io/" target="_blank">Chen Feng</a><sup>2</sup>,
        <a href="https://uav.hkust.edu.hk/current-members/" target="_blank">Haojia Li</a><sup>2</sup>,
        <a href="http://sysu-star.com/people/" target="_blank">Mingjie Zhang</a><sup>1</sup>,
        <a href="https://uav.hkust.edu.hk/current-members/" target="_blank">Xinyi Chen</a><sup>2</sup>,
        <a href="http://sysu-star.com/people/" target="_blank">Boyu Zhou</a><sup>1,†</sup>, and
        <a href="https://uav.hkust.edu.hk/group/" target="_blank">Shaojie Shen</a><sup>2</sup>
    <p>
        <h45>
            <sup>1</sup>SYSU STAR Group &nbsp;&nbsp;
            <sup>2</sup>HKUST Aerial Robotics Group &nbsp;&nbsp;
            <br>
        </h5>
        <sup>†</sup>Corresponding Author
    </p>
    <a href='https://arxiv.org/pdf/2309.13882.pdf'><img src='https://img.shields.io/badge/arXiv-2309.13882-red' alt='arxiv'></a>
    <a href='https://hkust-aerial-robotics.github.io/FC-Planner/'><img src='https://img.shields.io/badge/Project_Page-FC_Planner-green' alt='Project Page'></a>
    <a href="https://www.youtube.com/watch?v=U-X4OddXI88"><img alt="YouTube" src="https://img.shields.io/badge/YouTube-Video-blue"/></a>
</div>

## 📢 News

* **[02/05/2024]**: All remaining code is released (Viewpoint generation & Planning).
* **[06/04/2024]**: Code of Skeleton-based Space Decomposition (SSD) is released.
* **[15/03/2024]**: FC-Planner is extended to multi-UAV autonomous reconstruction as FC-Hetero, check this [repo](https://github.com/SYSU-STAR/FC-Hetero) for details.
* **[29/01/2024]**: FC-Planner is accepted to ICRA 2024.

## 📜 Introduction

**[ICRA'24]** This repository maintains the implementation of "FC-Planner: A Skeleton-guided Planning Framework for Fast Aerial Coverage of Complex 3D Scenes".

<p align="center">
  <img src="misc/top_2024.png" width = 60% height = 60%/>
  <img src="misc/fc-title.gif" width = 60% height = 60%/>
</p>

**FC-Planner** is a highly efficient planning framework tailored for fast coverage of large and complex 3D scenes. Both the simulation and real-world experiments demonstrate the superior system simplicity and performance of our method compared to state-of-the-art ones.

Please cite our paper if you use this project in your research:
* [FC-Planner: A Skeleton-guided Planning Framework for Fast Aerial Coverage of Complex 3D Scenes](https://arxiv.org/pdf/2309.13882.pdf), Chen Feng, Haojia Li, Mingjie Zhang, Xinyi Chen, Boyu Zhou, and Shaojie Shen, 2024 IEEE International Conference on Robotics and Automation (ICRA).

```
@article{feng2023fc,
  title={FC-Planner: A Skeleton-guided Planning Framework for Fast Aerial Coverage of Complex 3D Scenes},
  author={Feng, Chen and Li, Haojia and Jiang, Jinqi and Chen, Xinyi and Shen, Shaojie and Zhou, Boyu},
  journal={arXiv preprint arXiv:2309.13882},
  year={2023}
}
```

Please kindly star ⭐️ this project if it helps you. We take great efforts to develop and maintain it 😁.

## 🛠️ Installation

* ROS Noetic (Ubuntu 20.04) or ROS Melodic (Ubuntu 18.04)
* PCL 1.7
* Eigen3

The project has been tested on Ubuntu 20.04 LTS (ROS Noetic) and Ubuntu 18.04 LTS (ROS Melodic). Directly clone our package (using ssh here):

```shell
  sudo apt update
  sudo apt install cpufrequtils
  sudo apt install libompl-dev
  git clone git@github.com:HKUST-Aerial-Robotics/FC-Planner.git
  cd FC-Planner
  catkin_make
```

## 🚀 Quick Start

Run ```Rviz``` for coverage flight visualization and open another terminal for running the simulation:
```shell
sudo cpufreq-set -g performance
source devel/setup.zsh && roslaunch hierarchical_coverage_planner rviz.launch
source devel/setup.zsh && roslaunch hierarchical_coverage_planner mbs.launch
```
Trigger the quadrotor to start planning by the ``2D Nav Goal``, then ``2D Pose Estimate`` to start coverage flight as shown in the left gif. More scenes are provided in ```src/hierarchical_coverage_planner/launch```
<p align="center">
  <img src="misc/mbs.gif" width = 400 height = 240/>
  <img src="misc/mbs_recon.gif" width = 400 height = 240/>
</p>

Afterwards, you will obtain the discrete trajectory in ```src/hierarchical_coverage_planner/solution/Traj/TrajInfoMBS.txt```. You can use this coverage trajectory to capture the images of the target, and then reconstruct it. The detailed step can be found in [vis tool doc](./FC-Planner/vis_tool/README.md). The reconstruction result is depicted in the right gif.

#### • Use in Your Application

If you have successfully run the simulation and want to use FC-Planner in your project, please explore the launch files in ```src/hierarchical_coverage_planner/launch```. Important parameters that may be changed in your usage. Notably, FC-Planner accepts ***surface point cloud*** as input, so you may be careful about the type of your data.

## 🔌 Plug-in Packages

For the benefit of the community, we modularize the whole project and extract two important functions (space decomposition and viewpoint generation) as plug-in packages for your use.

#### • Skeleton-based Space Decomposition (SSD)

The individual package of SSD is given for your purposeful usage, where some example scenes are provided in ```src/rosa/launch```.

Run ```Rviz``` for SSD visualization and open another terminal for SSD execution:
```shell
sudo cpufreq-set -g performance
source devel/setup.zsh && roslaunch rosa rviz.launch
source devel/setup.zsh && roslaunch rosa ${SCENE}.launch (e.g., redbird.launch)
```

Afterwards, you will see the SSD results of HKUST RedBird in your ```Rviz``` as follows:
<p align="center">
  <img src="misc/redbird.png" width = 60% height = 60%/>
</p>

As for your ***customerized scene*** and ***more details***, you can find in [rosa package doc](FC-Planner/src/rosa/README.md). 

#### • Iterative Updates of Viewpoint Pose

Here we provide an independent package of finding the minimal viewpoint set for a given area needed to cover, *i.e.*, our proposed ***Iterative Updates of Viewpoint Pose***.

Specifically, we give a 3D case guided by skeleton. Run ```Rviz``` for visualization and open another terminal for viewpoint generation execution:
```shell
sudo cpufreq-set -g performance
source devel/setup.zsh && roslaunch viewpoint_manager rviz.launch
source devel/setup.zsh && roslaunch viewpoint_manager mbs.launch
```

Afterwards, you will see the viewpoint generation results of Marina Bay Sands in your ```Rviz``` as follows:
<p align="center">
  <img src="misc/iterative_updates.png" width = 80% height = 60%/>
</p>

You can also use other guidance to generate viewpoints, *e.g.*, normals. For example, we give a 2.5D case (indoor office) using normal guidance to generate viewpoints.
```shell
sudo cpufreq-set -g performance
source devel/setup.zsh && roslaunch viewpoint_manager rviz_normal.launch
source devel/setup.zsh && roslaunch viewpoint_manager normal.launch
```

Afterwards, you will see the viewpoint generation results of this 2.5D case in your ```Rviz``` as follows:
<p align="center">
  <img src="misc/2.5D_case.png" width = 80% height = 60%/>
</p>

For more usage details, you can find in [viewpoint manager doc](./FC-Planner/src/viewpoint_manager/README.md). 

## 🎮 Demo

<p align="center">
  <img src="misc/pipe.gif" width = 45% height = 45%/>
  <img src="misc/mbs.gif" width = 45% height = 45%/>
  <img src="misc/pacifico.gif" width = 45% height = 45%/>
  <img src="misc/christ.gif" width = 45% height = 45%/>
</p>
<p align="center">
  <img src="misc/realworld.gif"/>
</p>

## 🤗 FC-family Works

* [PredRecon](https://github.com/HKUST-Aerial-Robotics/PredRecon) (ICRA2023): Prediction-boosted Planner for Aerial Reconstruction.
* [FC-Planner](https://github.com/HKUST-Aerial-Robotics/FC-Planner) (ICRA2024): Highly Efficient Global Planner for Aerial Coverage.
* [FC-Hetero](https://github.com/SYSU-STAR/FC-Hetero) (Submitted to IROS2024): Heterogenous Multi-UAV Planner for Aerial Reconstruction.
