import traci
import math
import time
from DebugFile import log_info, DEBUG, log_debug


# this is a control module for controlling SUMO simulation
class SumoComntroller:
    """
    SUMO class controls SUMO simulation, providing vehicle control functionalities.
    """
    def __init__(self, config_file,config_data):
        """
        Initializes the SUMO simulation with the given configuration file and time step interval.
        Args:
            config_file (str): Path to the SUMO config file.
        """
        self.config_file = config_file
        self.config_data = config_data
        log_info('import the config data sccessful!')
        log_debug(f"config:{self.config_data}")
        self.traci = traci
        self.start()
        self.time_gap = float(self.config_data['step-length'])
        self.end_time = self.traci.simulation.getEndTime()


    def start(self):
        self.traci.start(["sumo-gui", "-c", self.config_file])


    def running(self):
        """
        Checks if the simulation is still running.

        Returns:
            bool: True if simulation time is less than the end time.
        """
        return self.get_time() < self.end_time

    def get_time(self):
        """
        Gets the current simulation time.

        Returns:
            float: The current time in the simulation.
        """
        return self.traci.simulation.getTime()

    def close(self):
        """
        Closes the SUMO simulation.
        """
        self.traci.close()

    def step(self):
        """
        Advances the simulation by one step.
        """
        self.traci.simulationStep()
        time.sleep(self.time_gap)

    def get_vehicle_position(self, vehicle_id):
        """
        Retrieves the position and angle of a specific vehicle.

        Args:
            vehicle_id (str): Vehicle ID.

        Returns:
            tuple or None: Vehicle position (x, y ,angle) or None if vehicle doesn't exist.
        """
        try:
            position = self.traci.vehicle.getPosition(vehicle_id)
            angle = self.traci.vehicle.getAngle(vehicle_id)
            # log_info(f"vehid:{vehicle_id} position: {(position[0], position[1], angle)}")
            position = (position[0],position[1],angle)
            return position
        except Exception as e:
            log_info(e)
            return None

    def move_to_xy(self, vehicle_id, x, y, speed=None):
        """
        Moves a vehicle to a specified (x, y) coordinate, adjusting its direction gradually.

        Args:
            vehicle_id (str): Vehicle ID.
            x (float): Target x-coordinate.
            y (float): Target y-coordinate.
            speed (float, optional): Speed of the vehicle. Defaults to None, using the current speed.
        """
        position = self.get_vehicle_position(vehicle_id)
        angle = position[2]
        if position is None:
            log_info(f"Can't find vehicle :{vehicle_id}")
            return

        # Calculate the direction towards the target point
        next_x, next_y = x - position[0], y - position[1]
        direction = math.degrees(math.atan2(next_x, next_y)) % 360
        # log_info(f'{vehicle_id} :{direction}')


        # If speed is not provided, use the vehicle's current speed
        if speed is None:
            speed = self.traci.vehicle.getSpeed(vehicle_id) or 5

        # Calculate the maximum turn angle based on speed and time gap
        max_turn_angle = math.degrees(speed * self.time_gap / 8)

        # Adjust the direction to not exceed the maximum turn angle
        # Difference between target angle and current angle
        angle_diff = direction - angle
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
        if abs(angle_diff) > max_turn_angle:
            direction = angle + (max_turn_angle if angle_diff > 0 else -max_turn_angle)

        # Calculate movement deltas
        delta_x = speed * self.time_gap * math.sin(math.radians(direction))
        delta_y = speed * self.time_gap * math.cos(math.radians(direction))

        # log_info(f"vehical:{vehicle_id}  Set angle: {direction}, Current position: {position} ,target position: {(x, y)}")

        # Move the vehicle to the new position
        self.traci.vehicle.moveToXY(
            vehicle_id, "", -1,
            position[0] + delta_x, position[1] + delta_y,
            angle=direction, keepRoute=2
        )
