import numpy as np
import trimesh
import open3d as o3d

scale = 40

def transform_model(input_file, output_file, rotation_matrix):
    scene = trimesh.load(input_file)

    center = np.mean([geometry.centroid for geometry in scene.geometry.values()], axis=0)

    for geometry_name, geometry in scene.geometry.items():
        geometry.vertices -= center

        rotated_vertices = np.dot(geometry.vertices, rotation_matrix.T)
        geometry.vertices = rotated_vertices
        
        geometry.vertices *= scale

        geometry.vertices += center

    scene.export(output_file)

    print(f"Transformation completed! {input_file} has been transformed and saved as {output_file}")

    return center

def transform_cloud(input_file, output_file, transformation_matrix, center):
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(input_file)

    # Move the points of the point cloud to the origin
    pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) - center)

    # Apply the transformation matrix
    transformed_points = np.dot(np.asarray(pcd.points), transformation_matrix.T)
    pcd.points = o3d.utility.Vector3dVector(transformed_points)

    pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) * scale)

    # Move the points of the point cloud back to the original center
    pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) + center)

    # Write the output file
    o3d.io.write_point_cloud(output_file, pcd)

    print(f"Transformation completed! {input_file} has been transformed and saved as {output_file}")

if __name__ == "__main__":
    input_glb_file = "/home/eason/data/japan/merge.glb" 
    output_glb_file = "/home/eason/data/japan/merge_t.glb" 
    input_obj_file = "/home/eason/data/japan/merge.obj" 
    output_obj_file = "/home/eason/data/japan/merge_t.obj" 
    input_pcd_file = "/home/eason/data/japan/merge.pcd" 
    output_pcd_file = "/home/eason/data/japan/merge_t.pcd" 

    # glb: y x z
    transformation_matrix = np.array([
        [0, 0, -1],
        [0, 1, 0],
        [1, 0, 0]
    ])
    center_glb = transform_model(input_glb_file, output_glb_file, transformation_matrix)

    # obj: x y z
    transformation_matrix = np.array([
        [1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]
    ])
    center_obj = transform_model(input_obj_file, output_obj_file, transformation_matrix)
    
    # pcd: x y z
    transformation_matrix = np.array([
        [1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]
    ])
    transform_cloud(input_pcd_file, output_pcd_file, transformation_matrix, center_obj)