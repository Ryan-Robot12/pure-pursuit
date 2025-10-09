import json
import math
import time

from PIDController import PIDController

earth_radius_m = 6378137

MAX_DRIVE_SPEED_M_S = math.pi
MAX_TURN_SPEED_RADS_SEC = math.pi / 4
ACCEPTABLE_TURN_ERR_TO_DRIVE = math.pi / 3

with open("oval_points.json") as file:
    all_points = json.load(file)

with open("points.json") as file:
    path_points = json.load(file)


def clamp(val, min_, max_):
    return min(max(val, min_), max_)


def deadzone(val, deadzone_=1e-5):
    return 0 if abs(val) <= deadzone_ else val


def rotate(l, n):
    return l[n:] + l[:n]


def get_node_loc(node_id: int):
    matches = []
    if node_id < 0:
        for node in all_points["nodes"]:
            if node["id"] == node_id:
                matches.append(node["latlon"])
    else:
        for node in all_points["points"]:
            if node["id"] == node_id:
                matches.append(node["latlon"])
    if len(matches) == 0:
        raise ValueError("No matching nodes were found.")
    return matches[0]


def input_modulus(input_, minimumInput, maximumInput):
    modulus = maximumInput - minimumInput

    num_max = (input_ - minimumInput) // modulus
    input_ -= num_max * modulus

    num_min = (input_ - maximumInput) // modulus
    input_ -= num_min * modulus

    return input_ % maximumInput


def calculate_heading(start_loc, target):
    """
    Calculates the heading to a given position
    :param start_loc: current pos in [lat, lon]
    :param target: target position in [lat, lon]
    :return: heading needed in radians, clockwise from north
    """
    lat1 = start_loc[0] * math.pi / 180
    lon1 = start_loc[1] * math.pi / 180
    lat2 = target[0] * math.pi / 180
    lon2 = target[1] * math.pi / 180
    delta_lon = lon2 - lon1
    x_c = math.sin(delta_lon) * math.cos(lat2)
    y_c = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    beta = math.atan2(x_c, y_c)
    # beta_deg = beta * 360 / 2 / math.pi
    return beta


def haversine_dist(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance in meters between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    # sometimes a is a very small number (e.g. 1e-17) and it flips to -1e-17
    if a < 0:
        a = 0
    c = 2 * math.asin(math.sqrt(a))
    r = 6371
    return c * r * 1000


class SimRobot:
    def __init__(self, pos: list):
        # sim stuff
        self.current_pos = pos
        self.current_heading = 0  # CW from North

        # PID stuff
        # negative P because current=distance, goal=0, goal-current < 0
        # self.drive_p = -1
        self.drive_p = -0.6
        self.drive_i = 0
        self.drive_d = 0
        self.drive_controller = PIDController(self.drive_p, self.drive_i, self.drive_d)

        self.turn_p = 1
        self.turn_i = 0
        self.turn_d = 0
        self.turn_controller = PIDController(self.turn_p, self.turn_i, self.turn_d)
        self.turn_controller.enable_continuous_input(0, 2 * math.pi)
        self._max_turn_err = math.pi  # 2pi / 2

        # dt tracking
        self._last_time = time.time()

        self._log = {
            "current_pos": [],
            "target_pos": [],
            "drive_err": [],
            "turn_err": [],
            "desired_heading": [],
            "current_heading": [],
            "drive_out": [],
            "turn_out": [],
            "dt": []
        }

    def drive_to_node(self, node_id: int):
        # where we going
        target_loc = get_node_loc(node_id)
        # distance there
        dist = self._distance_to_node(node_id)

        # desired heading
        desired_heading = calculate_heading(self.current_pos, target_loc)
        # change to east being 0, CCW positive for math, and clamp down to -2pi, 2pi
        desired_heading = (math.pi / 2 - desired_heading) % (2 * math.pi)
        if desired_heading < 0:
            desired_heading += 2 * math.pi

        # heading_err = abs(desired_heading - self.current_heading)
        # TODO: something about the turn stuff here is wrong
        heading_err = input_modulus(desired_heading - self.current_heading, -self._max_turn_err, self._max_turn_err)
        # PID calcs
        drive_output = self.drive_controller.calculate(dist, 0) if heading_err < ACCEPTABLE_TURN_ERR_TO_DRIVE else 0
        # print(round(heading_err, 2), round(drive_output, 2), round(self.current_heading, 2))
        turn_output = self.turn_controller.calculate(self.current_heading, desired_heading)
        # deadzone
        # drive_output = deadzone(drive_output)
        # turn_output = deadzone(turn_output)
        # clamp to robot constraints
        drive_output = clamp(drive_output, 0, MAX_DRIVE_SPEED_M_S)
        turn_output = clamp(turn_output, -1 * MAX_TURN_SPEED_RADS_SEC, MAX_TURN_SPEED_RADS_SEC)
        # debugging
        # print(f"drive err: {dist}, turn err: {desired_heading - self.current_heading}")
        self._log["current_pos"].append(self.current_pos)
        self._log["target_pos"].append(target_loc)
        self._log["drive_err"].append(dist)
        self._log["turn_err"].append(desired_heading - self.current_heading)
        self._log["desired_heading"].append(desired_heading)
        self._log["current_heading"].append(self.current_heading)
        self._log["drive_out"].append(drive_output)
        self._log["turn_out"].append(turn_output)
        self._log["dt"].append(time.time() - self._last_time)

        # simulate movement
        self.simulate_movement(drive_output, turn_output)

    def save_log(self, filename):
        with open(filename, "w") as file:
            json.dump(self._log, file, indent=2)

    def simulate_movement(self, forward_power, turn_power):
        # ---------------------------
        # handling forward/backward:
        dt = time.time() - self._last_time

        dx = forward_power * math.cos(self.current_heading) * dt  # east/west
        dy = forward_power * math.sin(self.current_heading) * dt
        # print(f"dist: {dist}, dt: {dt}, heading: {heading}, dx: {dx}, dy: {dy}")
        # self._log.append({"dist": dist, "heading": heading, "dx": dx, "dy": dy, "pos": self.pos, "target": target_loc})
        # dx, dy are in meters
        # need to transform back to latlon

        # Convert latitude and longitude from degrees to radians
        lat_rad = math.radians(self.current_pos[0])
        lon_rad = math.radians(self.current_pos[1])

        # Offset in radians
        dlat = dy / earth_radius_m
        dlon = dx / (earth_radius_m * math.cos(lat_rad))

        # update current pos
        self.current_pos[0] = math.degrees(lat_rad + dlat)
        self.current_pos[1] = math.degrees(lon_rad + dlon)
        # ---------------------------------------
        # handling turn:
        # handle dt * speed
        # TODO: when simulating, loops were so fast dt was 0 and nothing was happening
        if dt == 0:
            dt = 0.001
        self.current_heading += turn_power * dt
        self.current_heading %= 2 * math.pi
        if self.current_heading < 0:
            self.current_heading += 2 * math.pi
        self._last_time = time.time()
        # ---------------------------------------

    def _distance_to_node(self, node_id):
        """
        Gets the distance to a given node
        :param node_id: ID of a node
        :return: distance in meters
        """
        target_loc = get_node_loc(node_id)
        return haversine_dist(self.current_pos[0], self.current_pos[1], target_loc[0], target_loc[1])

    def drive_path(self, points: list, acceptable_err: float=0.25):
        """
        Drives along a given list of nodes
        :param points: list of node IDs
        :param acceptable_err: acceptabale error in meters
        """
        num_pts = len(points)
        for i, point in enumerate(points):
            print(f"Heading towards node {point}, point {i + 1}/{num_pts}")
            print(f"Target: {get_node_loc(point)}, distance: {self._distance_to_node(point)}")
            steps = 0
            d = self._distance_to_node(point)
            last_distances = [d for _ in range(20)]
            amt = len(last_distances)
            while sum(last_distances) / amt > acceptable_err:
                self.drive_to_node(point)
                steps += 1
                time.sleep(0.1)
                last_distances = rotate(last_distances, 1)
                last_distances[-1] = self._distance_to_node(point)
                # print(self._distance_to_node(point))
            print(f"Made it to node {point} in {steps} steps")


def main():
    robot = SimRobot(get_node_loc(path_points[0]))
    try:
        robot.drive_path(path_points[1:3])
    except KeyboardInterrupt:
        pass
    robot.save_log("data.json")


if __name__ == "__main__":
    main()
