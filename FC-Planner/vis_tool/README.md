# ⚽ FC-Planner

## *Visualization tool*

✍️ Author: [Chen Feng](https://chen-albert-feng.github.io/AlbertFeng.github.io/)

### 🛠️ Installation

We write this script to render the result in a professional 3D software [blender](https://www.blender.org/) 😀.

* Install blender 3.6.x in your computer by running the following command

```
sudo snap install blender --channel=3.6lts/stable --classic
```

### 📖 Usage

To validate the planning results of FC-Planner, we use a script in [MASSTAR](https://github.com/SYSU-STAR/MASSTAR) to simulate the images captured by the quadrotors during flight. Then we could use [Reality Capture](https://www.capturingreality.com/) to reconstruct the target scene.

Firstly, you should discrete the executed trajectory into a set of waypoints in a txt file, as shown in [example traj](./assets/TrajInfo.txt). If you use FC-Planner, this txt will be stored in ```src/hierarchical_coverage_planner/solution/Traj```.

* Make sure you have already install blender 3.6.x and python 3.10
* Directly run the following command in your terminal

You can modify the following parameters to your own data:

```
PIXEL_SIZE_CCD   : [double] --> physical pixel size in CCD
PIXEL_W          : [int] --> the number of horizontal pixels
PIXEL_H          : [int] --> the number of vertical pixels
FOV_ANGLE        : [double] --> horizontal FOV angle
STEP             : [int] --> downsampling frames
--model_path     : [string] --> your model path (absolute path)
--traj_path      : [string] --> your discreted waypoints txt file (absolute path)
--renderout_path : [string] --> your rendered outputs path (absolute path)
```

- Turn on blender software, then click ```Scripting``` and import ```render_in_traj.py```.
- Modify the file path in your workspace
- Click the triangle button to run the script.
- Obtain the rendered images like [example images](./assets/examples/).

Afterwards, you can generate the flight log file for assisting 3D reconstruction in [Reality Capture](https://www.capturingreality.com/).

```shell
python param.py --traj_path ${YOUR_TRAJ_TXT} --log_path ${YOUR_LOG_TXT}
```

You should utilize the flight log file ```${YOUR_LOG_TXT}``` in [Reality Capture](https://www.capturingreality.com/) by clicking button ```Import Flight Log``` and choosing ```local:1 Euclidean``` coordinate system. Then, you can follow the reconstruction procedure in [Reality Capture](https://www.capturingreality.com/) to obtain you 3D model and enjoy it!