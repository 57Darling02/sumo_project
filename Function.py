import random
import time
import math
from math import *
import traci
import sumolib
import numpy as np
from scipy.interpolate import interp1d
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import matplotlib

matplotlib.use('TkAgg')
from numpy.polynomial import Polynomial


class SUMO:
    def __init__(self, config_file="test.sumocfg", time_gap=0.01):
        self.config_file = config_file
        self.traci = traci
        self.traci.start(["sumo-gui", "-c", self.config_file])
        self.time_gap = time_gap

        self.lastAngel = 0
        self.end_time = traci.simulation.getEndTime()
        self.maxTurnAngel = 0.6


    def running(self):
        return self.getTime() < self.end_time

    def getTime(self):
        return self.traci.simulation.getTime()

    def close(self):
        self.traci.close()

    def step(self):
        self.traci.simulationStep()
        time.sleep(self.time_gap)

    def get_vehicle_position(self, vid):
        try:
            position = self.traci.vehicle.getPosition(vid)
            self.angel = self.traci.vehicle.getAngle(vid)
            print(f"now angel = {self.angel}")
            return position
        except self.traci.exceptions.TraCIException as e:
            # 如果车辆不存在或者发生了其他TraCI错误，将会执行这里的代码
            return None

    def movetoXY(self, vid, x, y, speed=None):
        position = self.get_vehicle_position(vid)
        if position is not None:
            # 归一化计算目标点的方向
            next_x = x - position[0]
            next_y = y - position[1]
            length = math.sqrt((next_x) ** 2 + next_y ** 2)
            next_x = next_x / length
            next_y = next_y / length
            direction = math.degrees(math.atan2(next_x, next_y)) % 360
            if speed is None:
                speed = self.traci.vehicle.getSpeed(vid)
                if speed is None or speed == 0:
                    speed = 10
            self.maxTurnAngel = math.degrees(speed * self.time_gap / 10)
            if abs(direction - self.angel) > self.maxTurnAngel:
                if direction - self.angel > 0:
                    direction = self.angel + self.maxTurnAngel
                else:
                    direction = self.angel - self.maxTurnAngel
            rangel = math.radians(direction)
            x_speed = speed * next_x
            y_speed = speed * next_y
            detal_x = speed * self.time_gap * math.sin(rangel)
            detal_y = speed * self.time_gap * math.cos(rangel)
            print(f"set angel :{direction},now position:{position}")
            self.traci.vehicle.moveToXY(vid, "", -1, position[0] + detal_x, position[1] + detal_y, angle=direction,
                                        keepRoute=2)
        else:
            pass


class veh:
    def __init__(self, sumo: SUMO, vid):
        self.vid = vid
        self.sumo = sumo
        self.aimedges = "E2"
        self.startcontrol = False
        self.StayTime = 0
        self.BreakDownCount = 0
        try:
            self.posX, self.posY = self.get_position()
        except:
            print("can't get the position")

    def SetSpeed(self, speed):
        self.sumo.traci.vehicle.setSpeed(self.vid, speed)

    def move(self, x, y, speed=None):
        self.sumo.movetoXY(self.vid, x, y, speed)

    def if_need_control(self, flag):
        if flag:
            self.startcontrol = True
        if self.sumo.traci.vehicle.getRoadID(self.vid) == "E2" and self.startcontrol:
            self.startcontrol = False
            return False
        else:
            return self.startcontrol

    def get_position(self):
        x, y = self.sumo.get_vehicle_position(self.vid)
        self.posX = x
        self.posY = y
        return x, y

    def BreakDownSimulator(self, flag, time_gap, StayTime=5):
        # when the car is in intersection ,flag = Ture
        BreakDownProbability = 10  # 0~100
        if flag and self.BreakDownCount == 0:
            if random.randint(1, 500) <= BreakDownProbability:
                self.BreakDownCount += 1
                self.StayTime = StayTime
        if self.StayTime / time_gap > 0:
            self.StayTime = self.StayTime - time_gap
            self.SetSpeed(0)
            print("speed0")
        else:
            self.SetSpeed(10)


class lane:
    def __init__(self, id):
        self.accuracy = 0.1
        self.id = id
        self.fucxy = None  # the function of dicacoordinate
        self.length = None
        self.samplingpoints = None
        self.NonVertical = True
        self.SamplingPoints = None

    def samplingXY(self, x, y, degree):  # 根据数据计算表达式 0，1是线性拟合
        x_data = np.array(x)
        y_data = np.array(y)
        # 多项式拟合
        # plt.figure(figsize=(8, 6))
        # plt.scatter(x_data, y_data, label='Data Points')
        if np.any(np.diff(x_data) == 0):
            if np.all(np.diff(x_data) == 0):  # 所有x值都相同，即完全垂直的线段
                self.NonVertical = False
                p = VerticalLineSegment(x_data, y_data)
        else:
            if degree == 0:
                p = interp1d(x_data, y_data, kind='linear', fill_value="extrapolate")  # 拟合折线
            else:
                p = Polynomial.fit(x_data, y_data, degree)  # 拟合曲线
        self.fucxy = p
        if len(x) != len(y):
            raise ValueError(f"wrong x y input\nx:{x}\ny:{y}")
        # calculate the length of Lane and sampling points
        # set accuracy
        SamplingPointsList = []
        accuracy = self.accuracy
        self.length = 0.0
        for i in range(len(x) - 1):
            # calculate distance between this point and next piont
            dx = x[i + 1] - x[i]
            dy = y[i + 1] - y[i]
            # using pythagorean theorem
            distance = math.sqrt(dx ** 2 + dy ** 2)
            # add up the distance
            self.length += distance
            if self.NonVertical:
                x_fit = np.linspace(x[i], x[i + 1], int(distance / accuracy))
                y_fit = p(x_fit)
            else:
                y_fit = np.linspace(y[i], y[i + 1], int(distance / accuracy))
                x_fit = np.full(y_fit.shape, x[0])
            SamplingPointsList.extend(list(zip(x_fit, y_fit)))
            # (x,y)
            self.SamplingPoints = SamplingPointsList
            # print(SamplingPointsList)
        return self.fucxy

    def DicaToFrenet(self, x, y):
        if self.SamplingPoints == None:
            raise ValueError("please use fuction samplingXY first")
        listA = np.array(self.SamplingPoints)
        differences = listA - np.array([x, y])
        distances_squared = np.sum(differences ** 2, axis=1)
        nearest_index = np.argmin(distances_squared)
        min_distance = np.sqrt(distances_squared[nearest_index])
        nearest_point = self.SamplingPoints[nearest_index]
        print(f"最近的点是: {nearest_point}, 索引为: {nearest_index}, 距离为: {min_distance}")
        rs = self.accuracy * (nearest_index)
        if nearest_index == 0:
            PreviousPiont = self.SamplingPoints[0]
            NextPiont = self.SamplingPoints[nearest_index + 1]
        elif nearest_index == len(self.SamplingPoints) - 1:
            PreviousPiont = self.SamplingPoints[nearest_index - 1]
            NextPiont = self.SamplingPoints[-1]
        else:
            PreviousPiont = self.SamplingPoints[nearest_index - 1]
            NextPiont = self.SamplingPoints[nearest_index + 1]
        dy = NextPiont[1] - PreviousPiont[1]
        dx = NextPiont[0] - PreviousPiont[0]
        # print(f"dy = {dy},dx={dx}")
        rtheta = atan2(dy, dx)
        # print(rtheta)
        # rkappa=0，rdkkapa=0是因为拟合的方式是一阶线性，v,a,theta,kappa用来计算s,l的一二阶导数，好像还没用，也不好做。
        s, l = self.cartesian_to_frenet(rs=rs, rx=nearest_point[0], ry=nearest_point[1], rtheta=rtheta, rkappa=0,
                                        rdkappa=0, x=x, y=y, v=0, a=0, theta=0, kappa=0)
        # s=(s,s',s'')l=(l,l',l'')
        return s[0], l[0]

    def FrenetToDica(self, s, l):
        nearest_index = int(s / self.accuracy)

    def cartesian_to_frenet(self, rs, rx, ry, rtheta, rkappa, rdkappa, x, y, v, a, theta, kappa):
        # 初始化车辆轨迹点在Frenet坐标系下的状态向量，即和
        s_condition = np.zeros(3)
        l_condition = np.zeros(3)
        # 分别计算的分量
        dx = x - rx
        dy = y - ry
        # 计算匹配点的方向切向量和法向量的分量，即，#
        cos_theta_r = cos(rtheta)
        sin_theta_r = sin(rtheta)
        # 计算在法向量方向上的投影，即
        # 作者更倾向于认为cross_rd_nd为frenet坐标系下的l值
        cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx
        l_condition[0] = copysign(sqrt(dx * dx + dy * dy), cross_rd_nd)
        # 计算
        delta_theta = theta - rtheta
        tan_delta_theta = tan(delta_theta)
        cos_delta_theta = cos(delta_theta)

        # 根据式（25)可得
        one_minus_kappa_r_l = 1 - rkappa * l_condition[0]
        l_condition[1] = one_minus_kappa_r_l * tan_delta_theta

        kappa_r_l_prime = rdkappa * l_condition[0] + rkappa * l_condition[1]

        # 根据式（33）计算
        l_condition[2] = (-kappa_r_l_prime * tan_delta_theta +
                          one_minus_kappa_r_l / cos_delta_theta / cos_delta_theta *
                          (kappa * one_minus_kappa_r_l / cos_delta_theta - rkappa))

        # 根据式（22）和式（42）分别计算
        s_condition[0] = rs
        s_condition[1] = v * cos_delta_theta / one_minus_kappa_r_l

        delta_theta_prime = one_minus_kappa_r_l / cos_delta_theta * kappa - rkappa
        s_condition[2] = ((a * cos_delta_theta -
                           s_condition[1] * s_condition[1] *
                           (l_condition[1] * delta_theta_prime - kappa_r_l_prime)) / one_minus_kappa_r_l)
        return s_condition, l_condition


class VerticalLineSegment:
    def __init__(self, x, y):
        self.x = x[0]
        self.y = y[0]

    def __call__(self, x_values):
        x_values = np.asarray(x_values)
        result = np.full_like(x_values, np.nan)
        matches = x_values == self.x
        result[matches] = self.y
        return result
