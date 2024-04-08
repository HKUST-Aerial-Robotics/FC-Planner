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

* **[06/04/2024]**: Code of Skeleton-based Space Decomposition (SSD) is released.
* **[29/01/2024]**: FC-Planner is accepted to ICRA 2024.

## 📜 Introduction

**[ICRA'24]** This repository maintains the implementation of "FC-Planner: A Skeleton-guided Planning Framework for Fast Aerial Coverage of Complex 3D Scenes".

<p align="center">
  <img src="misc/top_2024.png" width = 60% height = 60%/>
  <img src="misc/fc-title.gif" width = 60% height = 60%/>
</p>

**FC-Planner** is a skeleton-guided planning framework tailored for fast coverage of large and complex 3D scenes. Both the simulation and real-world experiments demonstrate the superior system simplicity and performance of our method compared to state-of-the-art ones.

Please cite our paper if you use this project in your research:
* [FC-Planner: A Skeleton-guided Planning Framework for Fast Aerial Coverage of Complex 3D Scenes](https://arxiv.org/pdf/2309.13882.pdf), Chen Feng, Haojia Li, Mingjie Zhang, Xinyi Chen, Boyu Zhou, and Shaojie Shen, 2024 IEEE International Conference on Robotics and Automation (ICRA).

```shell
@article{feng2023fc,
  title={FC-Planner: A Skeleton-guided Planning Framework for Fast Aerial Coverage of Complex 3D Scenes},
  author={Feng, Chen and Li, Haojia and Jiang, Jinqi and Chen, Xinyi and Shen, Shaojie and Zhou, Boyu},
  journal={arXiv preprint arXiv:2309.13882},
  year={2023}
}
```

Please kindly star ⭐️ this project if it helps you. We take great efforts to develop and maintain it 😁.

## 🛠️ Installation

* ROS Noetic (Ubuntu 20.04)
* PCL 1.7
* Eigen3

The project has been tested on Ubuntu 20.04 LTS (ROS Noetic). Directly clone our package (using ssh here):

```shell
  sudo apt update
  sudo apt install cpufrequtils
  sudo apt install libompl-dev
  git clone git@github.com:HKUST-Aerial-Robotics/FC-Planner.git
  cd FC-Planner
  catkin_make
```

## 🚀  Quick Start

#### Skeleton-based Space Decomposition (SSD)

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

As for your customerized scene, you can create the corresponding ```.launch``` file using the template in the given example scenes in ```src/rosa/launch```. For your convenience, we offer the meaning of each hyperparameter for your adjustment.

```shell
rosa_main/estimation_num                  : [int] -->  the number of points for skeleton extraction, using uniform downsampling.
rosa_main/pcd                             : [string] -->  the path of input scene, using ".pcd" form.
rosa_main/estimation_number               : [int] --> the neighboring number in normal estimation.
rosa_main/radius                          : [double] --> the radius of relative neighborhood for each point.
rosa_main/num_drosa                       : [int] --> the iteration number of calculating position and orientation of ROSA points.
rosa_main/num_dcrosa                      : [int] --> the iteration number of smoothening and shrinking ROSA points.
rosa_main/sample_r                        : [double] --> the sampling radius for pivotal ROSA points selection.
rosa_main/alpha                           : [double] --> the scale factor of Euclidean manner and projection manner in recentering.
rosa_main/upper_bound_angle_inner_decomp  : [double] --> upper direction consistency of each branch.
rosa_main/upper_bound_length_inner_decomp : [double] --> upper length of each branch.
rosa_main/Prune                           : [bool] --> identifier for branch pruning.
rosa_main/lower_bound_length              : [double] --> lower length of each branch.
```

#### ⏳ Remaining code will come soon...