import Function
time_gap = 0.01
sumo = Function.SUMO("test.sumocfg",time_gap)
vehicle_id = "vehicle1"
#设置最终的目标点
aimx = 4.3
aimy = 49
# 仿真主循环
sumo.step()
while sumo.running():
    try:
        position = sumo.get_vehicle_position(vehicle_id)
    except:
        print("end....")
    if position[0]>-10 and position[0]<4 and True:
        sumo.movetoXY(vehicle_id,aimx,aimy,10)
        # print(f"now angel:({traci.vehicle.getAngle(vehicle_id)});next position({position[0]+0.08*x/length},{position[1]+0.08*y/length})")
        # traci.vehicle.moveToXY(vehicle_id, "", -1, position[0]+0.08*x/length,position[1]+0.08*y/length,keepRoute=2)
    sumo.step()
sumo.close()
