# Interaction And Monitoring
import DebugFile
import PathPlanner
from DebugFile import log_info, log_debug
import Init

simulation_counter = 0 # 定义一个仿真计数器
planner_counter = 0 # 定义一个规划计数器

log_info('Initializing sumo controller')
sumo = Init.sumo_controller
log_info('Initializing static variables')
lane_list = Init.lane_list
lane_dict = Init.lane_dict
vehicle_list = Init.vehicle_list
break_down_car_list = Init.break_down_car_list
barrier_pos_list = [] # [(posX, posY)]
blocked_lane_list = []
barrier_list = []


def Monitor_update():
    # 允许调用以下全局变量
    global barrier_pos_list,barrier_list,blocked_lane_list,simulation_counter,planner_counter
    simulation_counter += 1 # 仿真计数器加1

    # Update the vehicle from sumo
    for car in vehicle_list:
        # update vehicle data
        car.update()
        if not car.exist:
            continue
        if car.vid != 'vehicle1':
            car.set_path([(-21.040883977900553, -1.6),(-13, -1.6),
                          (-10.719592623100075, -4.727572418658747), (-8.687334558583943, -4.52327134338993),
                          (-6.655076494067816, -4.318970268121113), (-4.622818429551687, -4.114669192852295),
                          (-1.7962450740627662, -3.3468318421320458), (0.0009605185291601259, -2.3709152631831016),
                          (1.4869884814887973, -0.6579990351050484), (2.973016444448434, 1.0549171929730035),
                          (4.860412330171232, 2.6933050347927443), (6.949231441886049, 3.601871862923396),(8.0, 50.0)])
        if simulation_counter % 50 == 0:
            # for car in adjust_car_list:
            #     log_info(f'{car.vid} planing new path')
            # 为了应付中期答辩，手动设计的路径
            log_info(f'{car.vid} pos is {car.posX, car.posY}\n path is {car.path}')

            # car.set_path(PathPlanner.PathPlan(car, barrier_pos_list))
        # check if the car is blocked
        # if blocked_lane_list:
        #     car_path_set = set(car.choose_path)  # 将 car.choose_path 转换为集合，查找效率更高
        #     for b_lane in blocked_lane_list:
        #         if b_lane in car_path_set:  # 使用集合的查找操作
        #             adjust_car_list.append(car)
        #             break  # 找到相同元素后，立即跳出循环，提高效率

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

    if barrier_list and simulation_counter % 100 == 0:
        temp = []
        temp2 = []
        for car_id in barrier_list:
            car = [car for car in vehicle_list if car.vid == car_id][0]
            temp.append((car.posX,car.posY))
            temp2.append(car.lane)
        barrier_pos_list = temp
        blocked_lane_list = temp2


