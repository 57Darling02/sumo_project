from ParseFile import parse_sumocfg
# from DebugFile import log_info,DEBUG
from SumoComntroller import *
import ParseFile
import environment
import Vehicle

# this module is used to initialize the simulation, loading the static data like network and vehicles_id

config_file = "test.sumocfg"
config_data = parse_sumocfg(config_file)
sumo_controller = SumoComntroller(config_file=config_file, config_data=config_data)

# Parsing lane data and drawing the network
lanedata = ParseFile.parse_net_xml(sumo_controller.config_data['net-file'])
lane_list = environment.get_lane_list(lanedata)
lane_dict = {i.id: i for i in lane_list}
for lane in lane_list:
    lane.width = sumo_controller.traci.lane.getWidth(lane.id)
log_info(f'Lanes List: {lane_dict}')

# Parsing route data
log_info('Importing route XML')
config_route_data = ParseFile.parse_routes_xml(sumo_controller.config_data['route-files'])


# data processing
vehicle_list = []
break_down_car_list = []
# intersection = environment.Intersection()

# Creating vehicle objects
for vehicle in config_route_data["vehicles"]:
    car = Vehicle.Veh(sumo=sumo_controller, vid=vehicle['id'])
    car.set_config(vehicle)
    log_debug(f"Vehicle appended: {vehicle['id']}")
    log_info(f'{car.vid} full path list: {car.full_path_list},choose path:{car.choose_path}')
    for lane_id in car.choose_path:
        for point in sumo_controller.traci.lane.getShape(lane_id):
            if point not in car.path:
                car.path.append(point)
    log_info(f'{car.vid} path:{car.path}')

    if car.config['type'] == "BreakDowncar":
        log_info(f"{car.vid} is a breakdown car")
        break_down_car_list.append(car.vid)
    vehicle_list.append(car)
log_info(f'Vehicle List: {[car.vid for car in vehicle_list]}')

# intersection.lane_list = lane_list
# intersection.car_list = vehicle_list


