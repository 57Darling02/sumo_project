import Function
time_gap = 0.01
sumo = Function.SUMO("test.sumocfg",time_gap)
vehicle_id = "vehicle1"
#设置最终的目标点
aimx = 4.3
aimy = 50
# 仿真主循环
sumo.step()
car1 = Function.veh(sumo,vehicle_id)
while sumo.running():
    try:
        # sumo.traci.vehicle.changeTarget(vehicle_id,"E2")
        print()
        position = sumo.get_vehicle_position(vehicle_id)
        In_intersection = position[0] > -20 and position[0] < 20 and position[1] > -20 and position[1] < 20
        if car1.if_need_control(In_intersection):
            car1.move(aimx, aimy, 10)
    except:
        print("end....")
        break
    sumo.step()
sumo.close()
