# ⚽ FC-Planner

## *ROSA package*

✍️ Author: [Chen Feng](https://chen-albert-feng.github.io/AlbertFeng.github.io/)

### 📖 Usage

The individual package of Skeleton-based Space Decomposition (SSD) is given for your purposeful usage, where some example scenes are provided in ```src/rosa/launch```.

Run ```Rviz``` for SSD visualization and open another terminal for SSD execution:
```shell
sudo cpufreq-set -g performance
source devel/setup.zsh && roslaunch rosa rviz.launch
source devel/setup.zsh && roslaunch rosa ${SCENE}.launch (e.g., redbird.launch)
```

Afterwards, you will see the SSD results of HKUST RedBird in your ```Rviz``` as follows:
<p align="center">
  <img src="../../../misc/redbird.png" width = 60% height = 60%/>
</p>

### ⚙️ Parameter

As for your customerized scene, you can create the corresponding ```.launch``` file using the template in the given example scenes in ```src/rosa/launch```. For your convenience, we offer the meaning of each hyperparameter for your adjustment.

```
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
