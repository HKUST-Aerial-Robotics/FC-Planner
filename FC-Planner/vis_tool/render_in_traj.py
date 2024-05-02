# ⭐⭐⭐**********************************************⭐⭐⭐ #
# * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST
# * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
# * Date         :    Mar. 2024
# * E-mail       :    cfengag at connect dot ust dot hk.
# * Description  :    This file is the main function of rendering using blender.
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

import argparse
import math
import os
import sys
import numpy as np

import bpy
from mathutils import Vector

FORMAT_VERSION = 6
UNIFORM_LIGHT_DIRECTION = [0.09387503, -0.63953443, -0.7630093]
PIXEL_SIZE_CCD = 0.05
PIXEL_W = 640
PIXEL_H = 480
FOV_ANGLE = 75
STEP = 5

def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def clear_lights():
    bpy.ops.object.select_all(action="DESELECT")
    for obj in bpy.context.scene.objects.values():
        if isinstance(obj.data, bpy.types.Light):
            obj.select_set(True)
    bpy.ops.object.delete()

# clear the scene and import model with different format
def import_model(path):
    clear_scene()
    _, ext = os.path.splitext(path)
    ext = ext.lower()
    if ext == ".obj":
        bpy.ops.import_scene.obj(filepath=path)
    elif ext in [".glb", ".gltf"]:
        bpy.ops.import_scene.gltf(filepath=path)
    elif ext == ".stl":
        bpy.ops.import_mesh.stl(filepath=path)
    elif ext == ".fbx":
        bpy.ops.import_scene.fbx(filepath=path)
    elif ext == ".dae":
        bpy.ops.wm.collada_import(filepath=path)
    elif ext == ".ply":
        bpy.ops.import_mesh.ply(filepath=path)
    else:
        raise RuntimeError(f"unexpected extension: {ext}")


def rotate_model():
    # Select all objects in the scene
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active = None
    bpy.ops.object.select_all(action='SELECT')

    # Rotate selected objects 90 degrees around the X axis
    for obj in bpy.context.selected_objects:
        obj.rotation_euler[0] += math.radians(-90)

def scene_root_objects():
    for obj in bpy.context.scene.objects.values():
        if not obj.parent:
            yield obj

# check the object in ths scene if it is 3D Mesh and yield it
def scene_meshes():
    for obj in bpy.context.scene.objects.values():
        if isinstance(obj.data, (bpy.types.Mesh)):
            yield obj

# add the camera in the blender
def create_camera():
    # https://b3d.interplanety.org/en/how-to-create-camera-through-the-blender-python-api/
    camera_data = bpy.data.cameras.new(name="Camera")
    camera_object = bpy.data.objects.new("Camera", camera_data)
    bpy.context.scene.collection.objects.link(camera_object)
    bpy.context.scene.camera = camera_object

# set the location and direction of the camera
def set_camera(camera_pos, camera_dir):
    bpy.context.scene.camera.location = camera_pos
    #print("camera=",bpy.context.scene.camera.location)
    # print('camera dir==============',camera_dir)
    bpy.context.scene.camera.rotation_euler = camera_dir
    bpy.context.view_layer.update()

def scene_bbox(single_obj=None, ignore_matrix=False):
    bbox_min = (math.inf,) * 3
    bbox_max = (-math.inf,) * 3
    found = False
    for obj in scene_meshes() if single_obj is None else [single_obj]:
        found = True
        for coord in obj.bound_box:
            coord = Vector(coord)
            if not ignore_matrix:
                coord = obj.matrix_world @ coord
            bbox_min = tuple(min(x, y) for x, y in zip(bbox_min, coord))
            bbox_max = tuple(max(x, y) for x, y in zip(bbox_max, coord))
    if not found:
        raise RuntimeError("no objects in scene to compute bounding box for")
    return Vector(bbox_min), Vector(bbox_max)

def set_camera_fov_horizontal(camera_name, fov): 
    frame_width = PIXEL_W*PIXEL_SIZE_CCD 
    camera = bpy.data.objects[camera_name].data 
    camera.sensor_fit = 'HORIZONTAL' 
    camera.lens = 0.5 * frame_width / math.tan(0.5 * math.radians(fov))

def process_traj_info(traj, sample_stride):
    
    point_num = len(traj)
    print(point_num)
    traj_dim = np.array(['TIMESTAMP:', 'X:', 'Y:', 'Z:', 'PITCH:', 'YAW:'])

    camera_traj = [0] * point_num
    camera_waypoint_5D = [0] * point_num

    i = 0
    for traj_point in traj[::sample_stride]:
        # print("traj_point",traj_point)
        dimension = int(len(traj_point) / 2)
        camera_traj_point = [0] * 6
        for d in range(dimension):

            if traj_point[d * 2] == traj_dim[d]:
                # print("index correct")
                separated = traj_point[2 * d + 1].split(',', 1)
                # print("separated",separated[0])
                camera_traj_point[d] = float(separated[0])
            elif traj_dim[d] != traj_dim[d]:
                print("index incorrect!!!")
        # print("camera_traj_point",camera_traj_point)

        camera_traj[i] = camera_traj_point
        i = i + 1

    point_num_sampled = i
    print(len(camera_traj[:point_num_sampled]))

    camera_waypoint_5D = np.array(camera_traj[:point_num_sampled])[:, [1, 2, 3, 4, 5]]
    
    return camera_waypoint_5D

def render_in_traj(traj_path:str,renderout_path: str,light_mode: str,fast_mode:bool):
    
    input_traj = np.loadtxt(traj_path, dtype=str)
    # sample image timestamp = step * t_discrete
    step = STEP
    camera_traj = process_traj_info(input_traj, step)
    (frame_num,traj_dim)=np.shape(camera_traj)
    # print('frame_array',camera_traj[0])
    if traj_dim != 5:
        print("incorrect dimension of camera's trajectory ")
    for i in range(frame_num): 
        x = camera_traj[i][0]
        y = camera_traj[i][1]
        z = camera_traj[i][2]
        pitch = camera_traj[i][3]
        yaw = camera_traj[i][4]
        # print("euler============",pitch,'0',yaw)
        camera_pos = (x,y,z)
        camera_dir = Vector([math.cos(pitch)*math.cos(yaw), math.cos(pitch)*math.sin(yaw),math.sin(pitch)])
        # camera_dir = - Vector(camera_pos)
        rot_quat = camera_dir.to_track_quat("-Z", "Y")
        camera_dir = rot_quat.to_euler()
        # print('camera pos==============',camera_pos)
        # print('camera dir==============',camera_dir)
        set_camera(camera_pos=camera_pos, camera_dir=camera_dir)


        # create_uniform_light()
        render_scene(
            os.path.join(renderout_path, f"{i:04}.png"),
            fast_mode=fast_mode,
        )

def create_light(location, energy=1.0, angle=0.5 * math.pi / 180):
    light_data = bpy.data.lights.new(name="Light", type="SUN")
    light_data.energy = energy
    light_data.angle = angle
    light_object = bpy.data.objects.new(name="Light", object_data=light_data)

    direction = -location
    rot_quat = direction.to_track_quat("-Z", "Y")
    light_object.rotation_euler = rot_quat.to_euler()
    bpy.context.view_layer.update()

    bpy.context.collection.objects.link(light_object)
    light_object.location = location


def create_uniform_light(backend):
    clear_lights()
    # Random direction to decorrelate axis-aligned sides.
    pos = 100*Vector(UNIFORM_LIGHT_DIRECTION)
    angle = 0.0092 if backend == "CYCLES" else math.pi
    create_light(pos, energy=10.0, angle=angle)
    create_light(-pos, energy=10.0, angle=angle)


def setup_nodes(renderout_path, capturing_material_alpha: bool = False):
    tree = bpy.context.scene.node_tree
    links = tree.links

    for node in tree.nodes:
        tree.nodes.remove(node)

    # Helpers to perform math on links and constants.
    def node_op(op: str, *args, clamp=False):
        node = tree.nodes.new(type="CompositorNodeMath")
        node.operation = op
        if clamp:
            node.use_clamp = True
        for i, arg in enumerate(args):
            if isinstance(arg, (int, float)):
                node.inputs[i].default_value = arg
            else:
                links.new(arg, node.inputs[i])
        return node.outputs[0]

    def node_clamp(x, maximum=1.0):
        return node_op("MINIMUM", x, maximum)

    def node_mul(x, y, **kwargs):
        return node_op("MULTIPLY", x, y, **kwargs)

    input_node = tree.nodes.new(type="CompositorNodeRLayers")
    input_node.scene = bpy.context.scene

    input_sockets = {}
    for output in input_node.outputs:
        input_sockets[output.name] = output

    if capturing_material_alpha:
        color_socket = input_sockets["Image"]
    else:
        raw_color_socket = input_sockets["Image"]
        color_node = tree.nodes.new(type="CompositorNodeConvertColorSpace")
        color_node.from_color_space = "Linear"
        color_node.to_color_space = "sRGB"
        tree.links.new(raw_color_socket, color_node.inputs[0])
        color_socket = color_node.outputs[0]
    split_node = tree.nodes.new(type="CompositorNodeSepRGBA")
    tree.links.new(color_socket, split_node.inputs[0])

    if capturing_material_alpha:
        return


# Key function
def render_scene(renderout_path, fast_mode: bool):
    use_workbench = bpy.context.scene.render.engine == "BLENDER_WORKBENCH"
    if use_workbench:
        # We must use a different engine to compute depth maps.
        bpy.context.scene.render.engine = "BLENDER_EEVEE"
        bpy.context.scene.eevee.taa_render_samples = 5  # faster, since we discard image.
    if fast_mode:
        if bpy.context.scene.render.engine == "BLENDER_EEVEE":
            bpy.context.scene.eevee.taa_render_samples = 5
        elif bpy.context.scene.render.engine == "CYCLES":
            bpy.context.scene.cycles.samples = 256
    else:
        if bpy.context.scene.render.engine == "CYCLES":
            bpy.context.scene.cycles.time_limit = 5
    bpy.context.view_layer.update()
    bpy.context.scene.use_nodes = True
    bpy.context.scene.view_layers["ViewLayer"].use_pass_z = True
    bpy.context.scene.view_settings.view_transform = "Raw"  # sRGB done in graph nodes
    bpy.context.scene.render.film_transparent = True
    bpy.context.scene.render.resolution_x = PIXEL_W
    bpy.context.scene.render.resolution_y = PIXEL_H
    bpy.context.scene.render.image_settings.file_format = "PNG"
    bpy.context.scene.render.image_settings.color_mode = "RGBA"
    bpy.context.scene.render.image_settings.color_depth = "16"
    bpy.context.scene.render.filepath = renderout_path
    setup_nodes(renderout_path)
    bpy.ops.render.render(write_still=True)


# Key function
def save_rendering_dataset(
    model_path: str,
    traj_path: str,
    renderout_path: str,
    backend: str,
    light_mode: str,
    fast_mode: bool,
):
    assert light_mode in ["random", "uniform", "camera"]

    import_model(model_path) 
    rotate_model()
    # print('scene bounding box======', scene_bbox())
    bpy.context.scene.render.engine = backend
    # normalize_scene()
    create_uniform_light(backend)

    create_camera()

    set_camera_fov_horizontal("Camera", FOV_ANGLE)

    #render the sampled points in the trajectory
    render_in_traj(traj_path,renderout_path,fast_mode,light_mode)

def main():
    raw_args = sys.argv[1 :]
    parser = argparse.ArgumentParser()
    # ! modify the path of the model and trajectory
    parser.add_argument("--model_path", type=str,default='assets/model/MBS.obj')
    parser.add_argument("--traj_path", type=str,default='assets/TrajInfo.txt')
    parser.add_argument("--renderout_path", type=str,default='assets/test/')
    parser.add_argument("--backend", type=str, default="BLENDER_EEVEE")
    parser.add_argument("--light_mode", type=str, default="uniform")
    parser.add_argument("--fast_mode", action="store_true")
    # unused
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1280)
    args = parser.parse_args(raw_args)

    save_rendering_dataset(
        model_path=args.model_path,
        traj_path=args.traj_path,
        renderout_path=args.renderout_path,
        backend=args.backend,
        light_mode=args.light_mode,
        fast_mode=args.fast_mode,
    )

if __name__ == "__main__":
    main()
