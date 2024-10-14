import xml.etree.ElementTree as ET
from DebugFile import log_info
def parse_sumocfg(file_path):
    # 解析 sumocfg 文件
    tree = ET.parse(file_path)
    root = tree.getroot()
    # 提取参数
    config_data = {}
    # 提取 input 子节点
    input_node = root.find('input')
    if input_node is not None:
        config_data['net-file'] = input_node.find('net-file').attrib.get('value')
        config_data['route-files'] = input_node.find('route-files').attrib.get('value')
        additional_files_node = input_node.find('additional-files')
        if additional_files_node is not None:
            config_data['additional-files'] = additional_files_node.attrib.get('value')

    # 提取 time 子节点
    time_node = root.find('time')
    if time_node is not None:
        config_data['begin'] = time_node.find('begin').attrib.get('value')
        config_data['end'] = time_node.find('end').attrib.get('value')
        config_data['step-length'] = time_node.find('step-length').attrib.get('value')

    # 提取其他全局参数
    param_node = root.find('param')
    if param_node is not None:
        for param in param_node.findall('param'):
            param_name = param.attrib.get('name')
            param_value = param.attrib.get('value')
            config_data[param_name] = param_value
    return config_data

def parse_net_xml(filename):
    log_info('parse net')
    tree = ET.parse(filename)
    root = tree.getroot()
    lanes = root.findall('.//lane')
    lane_result = []
    for lane in lanes:
        shape = lane.get('shape')
        points = shape.split()
        lane_data = {
            "id": lane.get('id'),
            "shape": lane.get('shape'),
            "points": points
        }
        # log_debug(f"Lane load {lane_data}")
        lane_result.append(lane_data)
    return lane_result


def parse_routes_xml(file_path):
    # 解析 routes 文件
    tree = ET.parse(file_path)
    root = tree.getroot()

    # 存储解析结果的字典
    config_data = {
        "vTypes": [],
        "routes": [],
        "vehicles": []
    }

    # 解析 <vType> 元素，获取车辆类型
    for vtype in root.findall('vType'):
        vtype_data = {
            "id": vtype.get('id'),
            "vClass": vtype.get('vClass'),
            "color": vtype.get('color'),
            "accel": vtype.get('accel'),
            "decel": vtype.get('decel'),
            "length": vtype.get('length'),
            "width": vtype.get('width'),
            "height": vtype.get('height'),
            "minGap": vtype.get('minGap'),
            "maxSpeed": vtype.get('maxSpeed')
        }
        config_data["vTypes"].append(vtype_data)

    # 解析 <route> 元素，获取路线信息
    for route in root.findall('route'):
        route_data = {
            "id": route.get('id'),
            "edges": route.get('edges')
        }
        config_data["routes"].append(route_data)

    # 解析 <vehicle> 元素，获取车辆信息
    for vehicle in root.findall('vehicle'):
        vehicle_data = {
            "id": vehicle.get('id'),
            "type": vehicle.get('type'),
            "route": vehicle.get('route'),
            "depart": vehicle.get('depart'),
            "departLane": vehicle.get('departLane'),
            "departPos": vehicle.get('departPos'),
            "departSpeed": vehicle.get('departSpeed'),
            "parameters": {}
        }

        # 获取 <parameter> 元素
        for param in vehicle.findall('parameter'):
            key = param.get('key')
            value = param.get('value')
            vehicle_data["parameters"][key] = value

        config_data["vehicles"].append(vehicle_data)

    # for vtype in config_data["vTypes"]:
    #     print(f"Vehicle Type: {vtype}")
    #
    # for route in config_data["routes"]:
    #     print(f"Route: {route}")



    return config_data
