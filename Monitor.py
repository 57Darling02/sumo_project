# Interaction And Monitoring
import DebugFile
import PathPlanner
from DebugFile import log_info, log_debug
import Init

simulation_counter = 0
planner_counter = 0

log_info('Initializing sumo controller')
sumo = Init.sumo_controller
log_info('Initializing static variables')
lane_list = Init.lane_list
lane_dict = Init.lane_dict
vehicle_list = Init.vehicle_list
break_down_car_list = Init.break_down_car_list
barrier_pos_list = []
blocked_lane_list = []
barrier_list = []


def Monitor_update():
    adjust_car_list = []

    global barrier_pos_list,barrier_list,blocked_lane_list,simulation_counter,planner_counter
    simulation_counter += 1



    # Update the vehicle from sumo
    for car in vehicle_list:
        # update vehicle data
        car.update()
        if not car.exist:
            continue
        # check if the car is blocked
        if blocked_lane_list:
            car_path_set = set(car.choose_path)  # 将 car.choose_path 转换为集合，查找效率更高
            for b_lane in blocked_lane_list:
                if b_lane in car_path_set:  # 使用集合的查找操作
                    adjust_car_list.append(car)
                    break  # 找到相同元素后，立即跳出循环，提高效率

        next_point = PathPlanner.choose_next_point(car.path, car.posX, car.posY)
        if car.path and next_point:
            x, y = next_point
            car.move(x, y)

        # monitor and flag when cars are break down
        if car.break_down:
            log_debug(f'{car.vid} is break down at {car.lane}')
            if car.vid not in barrier_list:
                barrier_list.append(car.vid)

        else:
            if car in barrier_list:
                barrier_list.remove(car)


        # Simulate breakdown for cars within certain coordinates
        if car.vid in break_down_car_list:
            if -14 < car.posX < 14 and -14 < car.posY < 14:
                car.break_down_simulator(100)

    if barrier_list:
        temp = []
        temp2 = []
        for car in barrier_list:
            temp.append((car.posX,car.posY))
            temp2.append(car.lane)
        barrier_pos_list = temp
        blocked_lane_list = temp2

    # run each 1s
    if simulation_counter % 100 == 0:
        for car in adjust_car_list:
            log_info(f'{car.vid} planing new path')
            car.set_path(PathPlanner.PathPlan(car ,barrier_pos_list))
