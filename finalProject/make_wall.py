import numpy as np
import open3d as o3d

# Input data type ----------------------------------------------
# points : <class 'numpy.ndarray'>
def make_wall(boundary_points, normal, min_z, high_z, sample_rate=0.075) :
    big = max(min_z, high_z)
    small = min(min_z, high_z)

    result = np.empty((1,3))
    result_normal = np.empty((1,3))

    for i in np.arange(small, big, sample_rate) :
        temp = np.pad(np.array(boundary_points), [(0,0),(0, 1)], 'constant', constant_values=i)
        result = np.concatenate((result, temp), axis=0)

        if normal != [] :
            result_normal = np.concatenate((result_normal, normal), axis=0)

    # return type is <class 'numpy.ndarray'>

    return result[1:], result_normal[1:]

def extend_wall(boundary_pc, min_z, high_z, sample_rate=0.001, min_points=10) :
    result = []
    save = []
    wall_points, _ = make_wall(np.asarray(boundary_pc.points)[:,:2], [], 0, 1, sample_rate=0.1)
    wall_pc = o3d.geometry.PointCloud()
    wall_pc.points = o3d.utility.Vector3dVector(wall_points)
    plane_models = []

    while( (np.asarray(wall_pc.points).shape[0]/10) > min_points ):
        print(np.asarray(wall_pc.points).shape[0]/10)
        # 1. wall_pc에서 segment를 구한다.
        [a, b, c, d], inliers = wall_pc.segment_plane(distance_threshold=0.01,
                                                     ransac_n=3,
                                                     num_iterations=4000)
        plane_models.append([a, b, c, d])
        inlier_cloud = wall_pc.select_by_index(inliers)

        if (-1.03 <= a <= -0.97 or 0.97 <= a <= 1.03) and -0.08 < b < 0.08 :
            a = 1.0
            b = 0.0

        print(f"asdasqPlane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        #plane_model을 가공한다.
        # (ax + by + cz + d = 0) 꼴에서, c=0으로 생각하고, y = -(a/b)x - d/b로 구한다.
        a = -(a/b)
        d = -(d/b)

        #2. 구한 segment를 wall_pc의 점을 제거하며 확장한다.
        inlier_points = np.asarray(inlier_cloud.points)
        min_x, min_y = np.min(inlier_points[:, 0]), np.min(inlier_points[:, 1])
        max_x, max_y = np.max(inlier_points[:, 0]), np.max(inlier_points[:, 1])

        # 2-1. segment의 plane_model과 , 최소 좌표 point 부터 최대 좌표 point까지 서치하며 해당 점 주변에 점이 있는지 확인한다.
        # 있는 경우, 주변 점을 제거하고, 평면을 확장한다.
        # 없는 경우, 설정된 오차 허용 횟수에 따라 더 2-1를 수행한다.
        #A. min_x ~ max_x 사이의 값은 모두 포함한다.
        for x in np.arange(min_x-1, max_x+1, sample_rate) :
            y = a*x + d
            save.append([x,y,0])
            for i in np.arange(0, 1, 0.1) :
                result.append([x,y,i])

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(np.asarray(result))
        #o3d.visualization.draw_geometries([pc])
        #o3d.visualization.draw_geometries([pc, wall_pc])

        dists = np.asarray(wall_pc.compute_point_cloud_distance(pc))
        ind = np.where(dists > 0.10)[0]
        wall_pc = wall_pc.select_by_index(ind)
        """
        error = 0
        #B. min_x 보다 작은 값에서 조사한다.
        while(error < error_tolerance) :
            for i in boundary_pc :
                #
                if () :
                    error = error + 1

        error = 0
        #C. max_x 보다 큰 값에서 조사한다.
        while (error < error_tolerance):
            if ():
                error = error + 1

        z = np.arange(min_z, high_z, radius * 0.7)
        pillar_points = np.tile(boundary[i], (z.shape[0], 1))

        pillar_points = np.hstack((pillar_points, z.reshape(z.shape[0], 1)))
        pillar_pc = o3d.geometry.PointCloud()
        pillar_pc.points = o3d.utility.Vector3dVector(pillar_points)

        dists = np.asarray(pc.compute_point_cloud_distance(pillar_pc))
        ind = np.where(dists > radius)[0]
        pc = pc.select_by_index(ind)
        print(i)
        """
    cross = get_cross(plane_models)
    cpc = o3d.geometry.PointCloud()
    cpc.points = o3d.utility.Vector3dVector(np.asarray(cross))

    dists = np.asarray(cpc.compute_point_cloud_distance(boundary_pc))
    ind = np.where(dists > 0.4)[0]
    cpc = cpc.select_by_index(ind, invert=True)
    cpc.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([cpc])
    cross = np.unique(np.round(np.asarray(cpc.points),8), axis=0)

    print(cross)

    sort_cross_index = sort_ccw(cross)
    print("sort_cross")
    print(sort_cross_index)

    normal = cal_normal(cross, sort_cross_index)
    result, normal = interpolate_wall(cross, normal, sort_cross_index)
    result, normal = make_wall(result, normal, min_z, high_z, sample_rate=0.04)


    return result, normal

#여러 평면 모델 (ax+by+cz+d=0)을 넣어 평면의 교점들을 찾아 리턴
def get_cross(plane_models) :
    result = []
    for i in range(len(plane_models)) :
        a1, b1, c1, d1 = plane_models[i]

        for j in range(len(plane_models)) :
            if i != j :
                a2, b2, c2, d2 = plane_models[j]

                if a1*b2 - a2*b1 == 0 :
                    pass

                x = (b1*d2 - b2*d1)/(a1*b2 - a2*b1)
                y = -(a1/b1 * x) - d1/b1
                print([x,y])
                result.append([x,y,0])

    return np.asarray(result)

#주어진 점을 반시계방향으로 정렬하여 오름차순으로 해당 점을 가리키는 index 리스트 리턴
def sort_ccw(points) :
    center = [np.mean(points[:, 0]), np.mean(points[:, 1]), 0]

    angle = []
    for i in range(points.shape[0]):
        delta = points[i] - center
        if delta[1] < 0:
            deg = 360 - np.degrees(np.arccos(delta[0] / np.linalg.norm(delta)))
        else:
            deg = np.degrees(np.arccos(delta[0] / np.linalg.norm(delta)))

        dist = np.linalg.norm(delta)
        angle.append([i, deg, dist])

    angle.sort(key=lambda v: (v[1], v[2]))

    angle = np.int64(np.asarray(angle))

    for i in range(len(points)) :
        _pc = o3d.geometry.PointCloud()
        _pc.points = o3d.utility.Vector3dVector(points[angle[:i, 0]])

    return angle[:, 0]

def interpolate_wall(points, normals, sorted_index, split_count=60) :
    result = []
    result_normal = []

    points_length = len(points)
    for i in range(points_length) :
        temp = []

        curr_point = points[sorted_index[i % points_length]]
        next_point = points[sorted_index[(i+1) % points_length]]
        d = (next_point - curr_point) / split_count

        for j in range(split_count+1) :
            temp.append(curr_point + d * j)
            result_normal.append(normals[i])

        result += temp

    result = np.concatenate((result, points), axis=0)
    result_normal += normals

    return np.asarray(result)[:, :2], np.asarray(result_normal)

def cal_normal(points, sorted_index) :
    result = []
    plane = [] #두 교점이 이루는 평면의 법선벡터
    delta = [] #두 교점 사이의 벡터
    center = [] #두 교점의 중심점

    points_length = len(points)
    for i in range(points_length): #get point delta
        curr_point = points[sorted_index[i % points_length]]
        next_point = points[sorted_index[(i + 1) % points_length]]
        print(f'***************************************{curr_point}')
        print(f'***************************************{next_point}')

        delta.append((next_point-curr_point))
        plane.append([delta[i][1], -delta[i][0], 1])
        center.append((next_point+curr_point) / 2)

    delta = np.asarray(delta)
    plane = np.asarray(plane)
    center = np.asarray(center)

    delta[:, 2] = 1
    center[:, 2] = 1

    # 1. 평면의 법선 벡터(plane)가 수직선인지 확인한다.
    # 2. 평면의 법선 벡터가 다른 delta 벡터와 만나는지 확인한다.(교점을 구한다)
    # 3. 수직이 아닌 경우, 교점의 x 좌표가 center의 x좌표보다 큰지 확인한다.
    # 4. 수직이 아닌 경우, 교점이 해당 delta가 가지는 두 points 사이에 존재하는지 확인한다.
    # 5. 3,4 를 만족하는 경우, count를 1 증가시킨다.
    # 6. count가 홀수이면, 해당 벡터를 normal로 설정하고 다음 평면 법선 벡터로 넘어간다.
    # 7. count가 짝수이면, 해당 평면 법선 벡터를 뒤집은 벡터를 사용하여 2~6을 반복한다.
    # 8. 둘다 홀수가 아닌 경우 오류를 출력한다.
    for i in range(points_length) :
        temp = np.cross(center[i], center[i]+plane[i])
        count = 0
        reverse_count = 0
        print(f'***************************************{points[i]}')

        for j in range(points_length):
            if i != j:
                curr_point = points[sorted_index[j % points_length]]
                curr_point[2] = 1
                next_point = points[sorted_index[(j + 1) % points_length]]
                next_point[2] = 1

                v = np.cross(temp , np.cross(curr_point, next_point))
                cross = v / v[2]

                is_cross_on_right_center = cross[0] >= center[i][0]
                is_cross_on_left_center = cross[0] < center[i][0]
                is_on_delta_vector = check_point_bewteen_points(curr_point, next_point, cross)

                if is_on_delta_vector :
                    if is_cross_on_right_center :
                        count += 1
                    elif is_cross_on_left_center :
                        reverse_count += 1

        result.append(plane[i] / np.linalg.norm(plane[i]))
        result[i][2] = 0

    print(result)

    return result

def check_point_bewteen_points(p1, p2, cross) :
    x1 = min(p1[0], p2[0])
    x2 = max(p1[0], p2[0])
    y1 = min(p1[1], p2[1])
    y2 = max(p1[1], p2[1])
    return x1 <= cross[0] <= x2 and y1 <= cross[1] <= y2