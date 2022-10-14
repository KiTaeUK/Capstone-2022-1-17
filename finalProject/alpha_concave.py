import numpy as np
import alphashape
import open3d as o3d

def alpha_concave(points, alpha=10.0):
    boundary = alphashape.alphashape(points, alpha)  # 3.44

    x = []
    y = []

    some_poly = boundary

    # Extract the point values that define the perimeter of the polygon
    x, y = some_poly.exterior.coords.xy
    out = [None] * (len(x) - 1)
    for i in range(0, len(x) - 1):
        out[i] = [x[i], y[i]]

    result = np.array(out)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.pad(np.array(result), (0, 1), 'constant', constant_values=0)[:-1, :])

    return pcd