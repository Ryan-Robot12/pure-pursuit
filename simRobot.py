import json
import math
import time

from PIDController import PIDController

earth_radius_m = 6378137

with open("oval_points.json") as file:
    all_points = json.load(file)

with open("points.json") as file:
    path_points = json.load(file)


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


def calculate_heading(start_loc, target):
    """
    Calculates the heading to a given position
    :param start_loc: current pos in [lat, lon]
    :param target: target position in [lat, lon]
    :return: heading needed in radians, clockwise from north
    """
    lat1 = start_loc[0] * 2 * math.pi / 360
    lon1 = start_loc[1] * 2 * math.pi / 360
    lat2 = target[0] * 2 * math.pi / 360
    lon2 = target[1] * 2 * math.pi / 360
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
    # TODO: find why a=-1.1102230246251565e-16 sometimes
    if abs(a) < 1e-15:
        a = 0
    c = 2 * math.asin(math.sqrt(a))
    r = 6371
    return c * r * 1000


def clamp(val, min_, max_):
    return max(min(val, max_), min_)


class SimRobot:
    def __init__(self, pos: list):
        self.current_pos = pos
        self.current_heading = 0  # CW from North

        self.drive_p = -0.1
        # self.drive_p = 0
        self.drive_i = 0
        self.drive_d = 0
        self.drive_controller = PIDController(self.drive_p, self.drive_i, self.drive_d)

        self.turn_p = 178
        self.turn_i = 0
        self.turn_d = 0
        self.turn_controller = PIDController(self.turn_p, self.turn_i, self.turn_d)

        self._last_time = time.time()

    def drive_to_node(self, node_id: int):
        target_loc = get_node_loc(node_id)
        dist = haversine_dist(self.current_pos[0], self.current_pos[1], target_loc[0], target_loc[1])

        desired_heading = calculate_heading(self.current_pos, target_loc)
        desired_heading = (math.pi / 2 - desired_heading) % (2 * math.pi)  # change to east being 0, CCW positive for math

        drive_output = self.drive_controller.calculate(dist, 0)
        turn_output = self.turn_controller.calculate(self.current_heading, desired_heading)
        # print(f"drive err: {dist}, current heading: {self.current_heading}, turn err: {desired_heading - self.current_heading}")

        # TODO: drive(drive_out, turn_out)
        # print(f"Forward power: {drive_output}, turn power: {turn_output}")
        self.simulate_movement(drive_output, turn_output)

    def simulate_movement(self, forward_power, turn_power):
        # ---------------------------
        # handling forward/backward:
        dx = forward_power * math.cos(self.current_heading)  # east/west
        dy = forward_power * math.sin(self.current_heading)
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

        self.current_pos[0] = math.degrees(lat_rad + dlat)
        self.current_pos[1] = math.degrees(lon_rad + dlon)
        # ---------------------------------------
        # handling turn:
        dt = time.time() - self._last_time
        if dt == 0:
            dt = 0.001
        self.current_heading += turn_power * dt
        self.current_heading %= 2 * math.pi
        self._last_time = time.time()
        # ---------------------------------------


    def _distance_to_node(self, node_id):
        target_loc = get_node_loc(node_id)
        return haversine_dist(self.current_pos[0], self.current_pos[1], target_loc[0], target_loc[1])

    def drive_path(self, points: list, acceptable_err: float=0.05):
        num_pts = len(points)
        for i, point in enumerate(points):
            print(f"Heading towards node {point}, point {i + 1}/{num_pts}")
            print(f"Target: {get_node_loc(point)}, distance: {self._distance_to_node(point)}")
            while self._distance_to_node(point) > acceptable_err:
                self.drive_to_node(point)
            print(f"Made it to node {point}")


def main():
    robot = SimRobot(get_node_loc(path_points[0]))
    robot.drive_path(path_points[1:])
    # counter = 0
    # t1 = time.time()
    # while robot._distance_to_node(1) > 0.05:
    #     counter += 1
    #     robot.drive_to_node(1)
    #     time.sleep(0.001)
    #     # print(f"Distance: {robot._distance_to_node(1)}")
    # print(f"Drove to node 1 in {counter} iterations over {round(time.time() - t1, 2)} seconds")


if __name__ == "__main__":
    main()
