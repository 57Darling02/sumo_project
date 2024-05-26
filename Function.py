import time
import math
import traci
import sumolib
# 启动SUMO仿真
class SUMO:
    def __init__(self,config_file= "test.sumocfg",time_gap=0.01):
        self.config_file  = config_file
        self.traci = traci
        self.traci.start(["sumo-gui", "-c", self.config_file])
        self.time_gap = time_gap
        self.lastAngel = 0
        self.end_time = traci.simulation.getEndTime()
        self.maxTurnAngel = 0.6
    def running(self):
        return self.getTime()<self.end_time
    def getTime(self):
        return self.traci.simulation.getTime()
    def close(self):
        self.traci.close()
    def step(self):
        self.traci.simulationStep()
        time.sleep(self.time_gap)
    def get_vehicle_position(self,vid):
        try:
            position = self.traci.vehicle.getPosition(vid)
            self.angel = self.traci.vehicle.getAngle(vid)
            print(f"now angel = {self.angel}")
            return position
        except self.traci.exceptions.TraCIException as e:
            # 如果车辆不存在或者发生了其他TraCI错误，将会执行这里的代码
            return None
    def movetoXY(self,vid,x,y,speed = None):
        position = self.get_vehicle_position(vid)
        if position is not None:
            #归一化计算目标点的方向
            next_x = x - position[0]
            next_y = y - position[1]
            length = math.sqrt((next_x) ** 2 + next_y ** 2)
            next_x = next_x/length
            next_y = next_y/length
            direction = math.degrees(math.atan2(next_x, next_y)) % 360
            if speed is None:
                speed = self.traci.vehicle.getSpeed(vid)
                if speed is None or speed == 0:
                    speed = 10
            self.maxTurnAngel = math.degrees(speed*self.time_gap/10)
            if abs(direction - self.angel) > self.maxTurnAngel:
                if direction - self.angel>0:
                    direction = self.angel+self.maxTurnAngel
                else:
                    direction = self.angel-self.maxTurnAngel
            rangel = math.radians(direction)
            x_speed = speed*next_x
            y_speed = speed*next_y
            detal_x = speed*self.time_gap*math.sin(rangel)
            detal_y = speed*self.time_gap*math.cos(rangel)
            print(f"set angel :{direction},now position:{position}")
            self.traci.vehicle.moveToXY(vid, "", -1, position[0]+detal_x, position[1]+detal_y,angle=direction,
                                   keepRoute=2)
        else:
            pass
class veh:
    def __init__(self,sumo:SUMO,vid):
        self.vid = vid
        self.sumo = sumo
        self.aimedges = "E2"
        self.startcontrol = False
    def move(self,x,y,speed = None):
        self.sumo.movetoXY(self.vid,x,y,speed)
    def if_need_control(self,In_intersection):
        if In_intersection:
            self.startcontrol = True
        if self.sumo.traci.vehicle.getRoadID(self.vid) =="E2" and self.startcontrol:
            self.startcontrol = False
            return False
        else:
            return self.startcontrol
