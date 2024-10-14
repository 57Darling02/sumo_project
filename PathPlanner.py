import itertools
import networkx as nx
import DebugFile
import Init
import numpy as np
import math
from math import *
from environment import accuracy
from DebugFile import log_debug, log_info

accuracy = accuracy
lane_list = Init.lane_list

def PathPlan(car:Init.Vehicle.Veh, barrier_pos_list, prediction_time = 3):

    # get one full path of all the path
    fullpath = car.full_path_list[0]
    barrier_frenet_pos_list = []
    # catch the sampling points of the full path
    sampling_points = []
    for id in fullpath:
        sampling_points.extend(Init.lane_dict[id].SamplingPoints)
    # but we not need all the sampling points, we just need some of them
    nearest_index, nearest_point ,nearest_distance = find_nearest_point(sampling_points, car.posX, car.posY)
    start_index = 0 if nearest_index-10 <= 0 else nearest_index-10
    temp = int(car.speed * prediction_time / accuracy)
    end_index = len(sampling_points)-1 if nearest_index+temp >= len(sampling_points)-1 else nearest_index+temp
    sampling_points = sampling_points[start_index:end_index]

    for barrier_pos in barrier_pos_list:
        _ , nearest_point ,_ = find_nearest_point(sampling_points, barrier_pos[0], barrier_pos[1])
        s, l = Cartrsian2Frenet(sampling_points,barrier_pos[0], barrier_pos[1])
        if not s and not l:
            continue
        barrier_frenet_pos_list.append((s,l))

    # calculate the frenet coordinate of now position
    # now_s ,now_l = Cartrsian2Frenet(sampling_points,now_position[0], now_position[1])
    # create the position list
    s_step = 3
    length = accuracy*len(sampling_points)
    l_step = 0.2
    log_info(f'generate sampling points:length:{length},s_step:{s_step},l_step:{l_step}')
    width = lane_list[0].width
    l_values = [0]  # 从中心0开始
    left = -l_step
    right = l_step
    # 不断扩展左右两边，直到差值大于等于width
    while right - left < width+4:
        l_values.insert(0, left)  # 将左边值插入到列表前面
        l_values.append(right)  # 将右边值追加到列表后面
        left -= l_step
        right += l_step
    s_value = [x for x in np.arange(0, length, s_step)]

    # all_path = generate_all_path(l_values, s_value)
    # but need too much time to calculate all the path
    # so we use a* to find the best path
    G = nx.Graph()
    for i, s in enumerate(s_value):
        for l in l_values:
            G.add_node((s, l), cost=cost_calculator(s, l, barrier_frenet_pos_list))

    max_angle = math.pi / 6
    steering_weight = 1
    for i in range(len(s_value) - 1):
        # connect between the previous point and the next point
        s1 = s_value[i]
        s2 = s_value[i + 1]
        for l1 in l_values:
            for l2 in l_values:
                cost1 = G.nodes[(s1, l1)]['cost']
                cost2 = G.nodes[(s2, l2)]['cost']
                avg_cost = (cost1 + cost2) / 2
                angle_cost = abs(math.atan2(l2 - l1, s2 - s1)) * steering_weight #
                total_cost = avg_cost + angle_cost
                if angle_cost <= max_angle * steering_weight:
                    G.add_edge((s1, l1), (s2, l2), weight=total_cost)
    # find the nearest point to the now position

    start_s, start_l = Cartrsian2Frenet(sampling_points, car.posX, car.posY)
    _ , nearest_point ,_ = find_nearest_point([(s_value[0],i) for i in l_values], start_s, start_l)
    start_node = (nearest_point[0], nearest_point[1])

    end_nodes = [(s, l) for l in l_values for s in [s_value[-1]]]
    # Use A* algorithm to find the best path
    best_path = None
    for end_node in end_nodes:
        try:
            path = nx.astar_path(G, start_node, end_node,
                                 heuristic=lambda u, v: np.linalg.norm(np.array(u) - np.array(v)))
            if best_path is None or len(path) < len(best_path):
                best_path = path
        except nx.NetworkXNoPath:
            continue

    print(f'best path:{best_path}')
    Result = []
    for node in best_path:
        x, y = Frenet2Cartesian(sampling_points, node[0], node[1])
        Result.append((x, y))
    DebugFile.planrundata = Result
    return Result

def choose_next_point(path, x, y):
    listA = np.array(path)
    differences = listA - np.array([x, y])
    distances_squared = np.sum(differences ** 2, axis=1)
    nearest_index = np.argmin(distances_squared)
    nearest_point = path[nearest_index]
    if nearest_index+1 < len(path):
        next_point = path[nearest_index+1]
    elif (x,y) == path[-1]:
        return None
    else:
        return path[nearest_index]
    # 提取点的坐标
    x2, y2 = nearest_point
    x3, y3 = next_point
    direction1 = (x - x2) * (x3 - x2) + (y - y2) * (y3 - y2)
    # 计算向量的点积
    if  direction1> 0:
        return next_point
    else:
        return nearest_point



def cost_calculator(s, l, barrier_pos_list, g=1, o=3000, sigma=1):
    # 重力场代价
    gravity_cost = g * (l ** 2)

    # 障碍物场代价 (Gaussian-based repulsive potential field)
    obstacle_cost = 0
    for barrier_pos in barrier_pos_list:
        # 使用高斯函数形式的障碍物势场来模拟排斥
        distance = math.sqrt((s - barrier_pos[0]) ** 2 + (l - barrier_pos[1]) ** 2)
        # 高斯势场函数：障碍物中心位置处势场值最大，远离障碍物后迅速衰减
        obstacle_cost += o * math.exp(- (distance ** 2) / (2 * sigma ** 2))
    # 计算总代价
    total_cost = gravity_cost + obstacle_cost
    return total_cost



def generate_all_path(l_values, s_value):
    # 使用itertools.product生成所有可能的(l1, l2, ..., lN)组合，卡死了动不了！！!不可行，换一个
    log_info('generate all path')
    all_paths = list(itertools.product(l_values, repeat=len(s_value)))
    # 将(l1, l2, ..., lN)与(s1, s2, ..., sN)对应生成路径
    candidate_paths = []
    for path in all_paths:
        candidate_path = [(s_value[i], path[i]) for i in range(len(s_value))]
        candidate_paths.append(candidate_path)
    # 输出路径
    for i, path in enumerate(candidate_paths):
        log_info(f"路径 {i + 1}: {path}")


def Cartrsian2Frenet(SamplingPoints, x, y):
    if SamplingPoints == None:
        raise ValueError("please use fuction samplingXY first")
    nearest_index, nearest_point ,_ = find_nearest_point(SamplingPoints, x, y)
    rs = accuracy * (nearest_index)
    if nearest_index == 0:
        PreviousPiont = SamplingPoints[0]
        NextPiont = SamplingPoints[nearest_index + 1]
    elif nearest_index == len(SamplingPoints) - 1:
        PreviousPiont = SamplingPoints[nearest_index - 1]
        NextPiont = SamplingPoints[-1]
    else:
        PreviousPiont = SamplingPoints[nearest_index - 1]
        NextPiont = SamplingPoints[nearest_index + 1]
    dy = NextPiont[1] - PreviousPiont[1]
    dx = NextPiont[0] - PreviousPiont[0]
    # log_debug(f"dy = {dy},dx={dx}")

    rtheta = atan2(dy, dx)
    s, l = cartesian_to_frenet1D(rs=rs, rx=nearest_point[0], ry=nearest_point[1], rtheta=rtheta,  x=x, y=y)
    if math.sqrt((nearest_point[0] - x) ** 2 + (nearest_point[1] - y) ** 2) - l > accuracy:
        return None ,None
    return s, l


def find_nearest_point(SamplingPoints, x, y):
    points = np.array(SamplingPoints)
    target = np.array([x, y])
    distances_squared = np.sum((points - target) ** 2, axis=1)
    nearest_index = np.argmin(distances_squared)
    nearest_point = points[nearest_index]
    min_distance = np.sqrt(distances_squared[nearest_index])
    log_debug(f"最近的点是: {nearest_point}, 索引为: {nearest_index}, 距离为: {min_distance}")
    return nearest_index, nearest_point ,min_distance


def Frenet2Cartesian(SamplingPoints, s, l):
    nearest_index = math.ceil(s / accuracy)
    nearest_point = SamplingPoints[nearest_index]
    rs = nearest_index * accuracy
    if nearest_index == 0:
        PreviousPiont = SamplingPoints[0]
        NextPiont = SamplingPoints[nearest_index + 1]
    elif nearest_index == len(SamplingPoints) - 1:
        PreviousPiont = SamplingPoints[nearest_index - 1]
        NextPiont = SamplingPoints[-1]
    else:
        PreviousPiont = SamplingPoints[nearest_index - 1]
        NextPiont = SamplingPoints[nearest_index + 1]
    dy = NextPiont[1] - PreviousPiont[1]
    dx = NextPiont[0] - PreviousPiont[0]
    # log_debug(f"dy = {dy},dx={dx}")


    rtheta = atan2(dy, dx)

    x ,y = frenet_to_cartesian1D(rs=rs,rx=nearest_point[0],ry=nearest_point[1],s_condition=s,d_condition=l,rtheta=rtheta)

    return x, y


def cartesian_to_frenet1D(rs, rx, ry, rtheta, x, y):
    s_condition = np.zeros(1)
    d_condition = np.zeros(1)

    dx = x - rx
    dy = y - ry

    cos_theta_r = cos(rtheta)
    sin_theta_r = sin(rtheta)

    cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx
    d_condition = copysign(sqrt(dx * dx + dy * dy), cross_rd_nd)

    s_condition = rs

    return s_condition, d_condition


def frenet_to_cartesian1D(rs, rx, ry, rtheta, s_condition, d_condition):
    if fabs(rs - s_condition) >= 1.0e-6:
        print("The reference point s and s_condition[0] don't match")

    cos_theta_r = cos(rtheta)
    sin_theta_r = sin(rtheta)

    x = rx - sin_theta_r * d_condition
    y = ry + cos_theta_r * d_condition

    return x, y
