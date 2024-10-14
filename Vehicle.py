import DebugFile
from SumoComntroller import SumoComntroller
from DebugFile import log_info, log_debug
class Veh:
    def __init__(self, sumo: SumoComntroller, vid):
        self.vid = vid
        self.sumoAPI = sumo
        self.config = None
        self.aimedges = None
        self.BreakDownCount = 0
        self.exist = False
        self.posX = None
        self.posY = None
        self.angle = None
        self.speed = None
        self.lane = None
        self.full_path_list = self.get_vehicle_full_lane_path()
        self.choose_path = self.full_path_list[0]
        self.path = []
        self.break_down = False

    def setAcceleration(self, acc , duration=1):
        """设置车辆加速度"""
        if self.exist:
            self.sumoAPI.traci.vehicle.setAcceleration(self.vid, acc, duration)


    def set_speed(self, speed):
        """设置车辆速度"""
        if self.exist:
            self.sumoAPI.traci.vehicle.setSpeed(self.vid, speed)

    def move(self, x, y, speed=None):
        """移动车辆到指定位置"""
        if self.exist and not self.break_down:
            self.sumoAPI.move_to_xy(self.vid, x, y, speed)
            if self.vid == 'vehicle2':
                DebugFile.controlrundata.append((x, y))

    def get_position(self):
        """获取车辆当前位置"""
        if self.exist:
            x, y, angle = self.sumoAPI.get_vehicle_position(self.vid)
            self.posX = x
            self.posY = y
            self.angle = angle
            return x, y, angle
        return None

    def break_down_simulator(self, stay_time=5):
        """模拟车辆故障"""
        if self.exist:
            time_gap = self.sumoAPI.time_gap
            self.BreakDownCount += 1
            if self.BreakDownCount == 1:
                self.break_down = True
                self.StayTime = stay_time
                log_info(f"Vehicle {self.vid} breaks down")
            elif self.StayTime / time_gap > 0:
                self.StayTime -= time_gap
                self.set_speed(0)
            else:
                self.set_speed(5.5)

    def set_config(self, config):
        """设置车辆的配置"""
        self.config = config

    def update(self):
        """更新车辆的当前状态（位置、速度、车道等）"""
        if self.vid not in self.sumoAPI.traci.vehicle.getIDList():
            self.exist = False
            return False
        else:
            self.exist = True
        self.posX, self.posY, self.angle = self.get_position()
        if self.path and self.path[-1][0] - self.posX<3 and self.path[-1][1] - self.posY<3:
            self.sumoAPI.traci.vehicle.remove(self.vid)
        self.speed = self.sumoAPI.traci.vehicle.getSpeed(self.vid)
        self.lane = self.sumoAPI.traci.vehicle.getLaneID(self.vid)
        if self.vid == 'vehicle2':
            DebugFile.realrundata.append((self.posX,self.posY))
        # if not self.path:
        #     self.path = self.choose_path[0]
        return True

    # 获取车辆的完整车道路径
    def get_vehicle_full_lane_path(self):
        """获取车辆的完整车道路径，包括中间的交叉口车道"""
        vehicle_id = self.vid
        # 获取车辆的完整路段路径
        route_edges = self.sumoAPI.traci.vehicle.getRoute(vehicle_id)
        lane_path = self.get_lane_combinations(route_edges)
        return lane_path

    # 获取路段上的所有车道ID
    def get_lanes_for_edge(self, edge_id):
        """获取路段上的所有车道ID"""
        lane_number = self.sumoAPI.traci.edge.getLaneNumber(edge_id)
        lanes = [f"{edge_id}_{i}" for i in range(lane_number)]  # 构建 lane ID
        return lanes

    # 获取从源路段到目标路段的所有车道组合，包括中间的交叉口车道
    def get_lane_combinations(self, route_edges):
        """获取从源路段到目标路段的所有车道组合，包括中间的交叉口车道"""
        if len(route_edges) < 2:
            return []  # 确保有足够的路段

        valid_combinations = []  # 用于存储所有有效的 lane 组合

        for i in range(len(route_edges) - 1):
            source_edge = route_edges[i]
            target_edge = route_edges[i + 1]

            source_lanes = self.get_lanes_for_edge(source_edge)
            target_lanes = self.get_lanes_for_edge(target_edge)
            log_debug(f"target:{target_lanes}")

            # 遍历 source_edge 的每一条车道
            for source_lane in source_lanes:
                # 获取 source_lane 的所有连接
                lane_links = self.sumoAPI.traci.lane.getLinks(source_lane)
                log_debug(f"{source_lane}:{lane_links}")
                # 遍历连接，查找能连接到 target_edge 的车道
                for link in lane_links:
                    target_lane = link[0]  # 连接的目标车道ID
                    connect_juction = link[4]
                    if target_lane in target_lanes:
                        # 如果连接的目标车道属于 target_edge，添加到组合列表中
                        valid_combinations.append((source_lane, connect_juction,target_lane))

        return valid_combinations
    def set_path(self,path):
        self.path = path