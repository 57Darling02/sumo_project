import math
import numpy as np
from scipy.interpolate import interp1d
from numpy.polynomial import Polynomial
import matplotlib
matplotlib.use('TkAgg')
from DebugFile import log_info, log_debug
accuracy = 0.1

class Path:
    def __init__(self,fullpath,lane_list):
        self.fuction = None
        self.sampling_points = []
        for lane in lane_list:
            log_debug(f"{lane.id} ? {fullpath}")
            if lane.id in fullpath:
                self.sampling_points.extend(lane.SamplingPoints)
        log_debug(f'points:{self.sampling_points}')



class Lane:
    def __init__(self, id):
        self.accuracy = accuracy
        self.id = id
        self.fucxy = None  # the function of dicacoordinate
        self.length = None
        self.samplingpoints = None
        self.NonVertical = True
        self.SamplingPoints = None
        self.width = None
        log_debug(f'Lane{self.id} is created.')
    def samplingXY(self, x, y, degree):  # 根据数据计算表达式 0，1是线性拟合
        p = None
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
        if p:
            self.fucxy = p
        log_debug(f'lane {self.id} load function.')
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
        log_debug(SamplingPointsList)
        return self.fucxy



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



def get_lane_list(data):
    lane_result = []
    for i in data:
        xlist = []
        ylist = []
        # 示例：解析shape并打印每个点
        points = i['points']
        for point in points:
            x, y = map(float, point.split(','))
            # log_debug(f"  Point: {x}, {y}")
            xlist.append(x)
            ylist.append(y)
        Sumolane = Lane(f"{i['id']}")
        Sumolane.samplingXY(xlist, ylist, 0)
        lane_result.append(Sumolane)
        # 生成拟合图
    #     if True:
    #         x_fit = [point[0] for point in Sumolane.SamplingPoints]
    #         y_fit = [point[1] for point in Sumolane.SamplingPoints]
    #         plt.plot(x_fit, y_fit, marker='o', markersize=2, label=f'Lane {id}')
    # plt.show()
    return lane_result
class Intersection:
    def __init__(self):
        self.car_list = None
        self.lane_list = None
        self.Test = None

