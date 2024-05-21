import time
import math
import traci
import sumolib
# 启动SUMO仿真
traci.start(["sumo-gui", "-c", "test.sumocfg"])
time_gap = 0.01
# 获取SUMO实例
# sumo_instance = sumolib.checkBinary("sumo")
# 等待车辆到达十字路口
traci.simulationStep()  # 仿真开始

# 假设我们知道车辆ID和它应该停留的边缘（edge）
vehicle_id = "vehicle1"
# stop_edge_id = ":J0_22"
end_time = traci.simulation.getEndTime()
#设置最终的目标点
aimx = 4
aimy = 20
edgeidn = traci.vehicle.getRoadID(vehicle_id)
# 仿真主循环
while traci.simulation.getTime() < end_time:
    # 避免时间超过仿真范围
    try:
        print(f"Current simulation time: {traci.simulation.getTime()}")
        print(f"Current simulation routeid: {traci.vehicle.getRoadID(vehicle_id)}")
    except traci.exceptions.TraCIException as e:
        # 如果车辆不存在或者发生了其他TraCI错误，将会执行这里的代码
        # print(f"Vehicle {vehicle_id} does not exist: {e}")
        break
    #计算当前位置和目标点的直线方向
    position = traci.vehicle.getPosition(vehicle_id)
    x = aimx - position[0]
    y = aimy - position[1]
    length = math.sqrt(x ** 2 + y ** 2)
    if length == 0:
        length = 1
    # 当小车进入路口后，开始向直线连接目标点的方向移动
    if position[0]>-10 and position[0]<0 and True:
        print(traci.vehicle.getLaneID(vehicle_id))
        traci.vehicle.moveToXY(vehicle_id, "", -1, position[0]+0.08*x/length,position[1]+0.08*y/length)
    traci.simulationStep()
    time.sleep(time_gap)# 执行仿真的下一步
# 结束仿真
traci.close()
