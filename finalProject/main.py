import open3d as o3d
import numpy as np

import make_wall
import remove_wall
import plane_extractor
import alpha_concave

import matplotlib.pyplot as plt

def concat_pc(pc1, pc2):
    result = o3d.geometry.PointCloud()
    points = np.concatenate((np.asarray(pc1.points), np.asarray(pc2.points)), axis=0)
    colors = np.concatenate((np.asarray(pc1.colors), np.asarray(pc2.colors)), axis=0)
    normals = np.concatenate((np.asarray(pc1.normals), np.asarray(pc2.normals)), axis=0)

    result.points = o3d.utility.Vector3dVector(points)
    result.colors = o3d.utility.Vector3dVector(colors)
    result.normals = o3d.utility.Vector3dVector(normals)

    return result

#for cropped_1

ALPHA = 10.0
MAKE_WALL_SAMPLE_RATE = 0.1
MIN_POINTS = 20
REMOVE_WALL_RADIUS = 0.053


#for cropped_2
# ALPHA = 10.0
# MAKE_WALL_SAMPLE_RATE = 0.1
# MIN_POINTS = 10
# REMOVE_WALL_RADIUS = 0.053

pc = o3d.io.read_point_cloud("cropped_1.ply")

#extract ceil, floor
_, high_z, min_z, floor = plane_extractor.extract_floor(pc, show_progress=False)    # 면 뜯어내기(다운샘플 + z축 편집)
plane, high_z, min_z, ceil = plane_extractor.extract_ceil(pc, show_progress=False)    # 면 뜯어내기(다운샘플 + z축 편집)

#extract boundary
boundary = alpha_concave.alpha_concave(plane, alpha=ALPHA)
o3d.visualization.draw_geometries([boundary])

#remove ceil, floor, wall
furniture = o3d.geometry.PointCloud()
removed = remove_wall.remove_wall(pc, np.asarray(boundary.points)[:, :2], method="cylinder", radius=REMOVE_WALL_RADIUS, min_z=min_z , high_z=high_z)
removed = remove_wall.remove_ceil_and_floor(removed, high_z - 0.07, min_z + 0.07)

with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(
    removed.cluster_dbscan(eps=0.05, min_points=30, print_progress=True))
max_label = labels.max()

for i in range(max_label) :
    temp = o3d.geometry.PointCloud()
    index = np.where(labels, )
print(labels)

print(f"point cloud has {max_label + 1} clusters")

o3d.visualization.draw_geometries([removed])
o3d.io.write_point_cloud("furniture.ply", removed)

#extend wall
extend_p, extend_n = make_wall.extend_wall(boundary, min_z, high_z, min_points=MIN_POINTS)

extend_pc = o3d.geometry.PointCloud()
extend_pc.points = o3d.utility.Vector3dVector(extend_p)
extend_pc.normals = o3d.utility.Vector3dVector(extend_n * -1)
o3d.visualization.draw_geometries([extend_pc])

floor.normals = o3d.utility.Vector3dVector(np.tile([0,0,1], reps=[np.asarray(floor.points).shape[0], 1]))
ceil.normals = o3d.utility.Vector3dVector(np.tile([0,0,-1], reps=[np.asarray(ceil.points).shape[0], 1]))

extend_pc = concat_pc(extend_pc, ceil)
extend_pc = concat_pc(extend_pc, floor)

#reconstruction
o3d.visualization.draw_geometries([extend_pc])

with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        extend_pc, depth=8)

o3d.visualization.draw_geometries([mesh])

o3d.io.write_triangle_mesh("mesh.obj",mesh, print_progress=True)

f = o3d.io.read_point_cloud("furniture.ply")
m = o3d.io.read_triangle_mesh("mesh.ply")

