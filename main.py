import json
import math
import time

earth_radius_m = 6378137

with open("oval_points.json") as file:
    all_points = json.load(file)

with open("points.json") as file:
    path_points = json.load(file)

end_point = path_points[-1]
first_point = path_points[0]
current_node = 0


class Robot:
    def __init__(self):
        self.pos = [0, 0]
        self._last = time.time()
        self._log = []

    # def move(self, speed: float, heading: float):
    #     """
    #     Simulates robot motion, accounting for time since last call
    #     :param speed: speed in m/s
    #     :param heading: heading, in degrees
    #     """
    #     heading /= 180
    #     heading *= math.pi
    #     self.pos[0] += ((time.time() - self._last) * speed) * math.cos(heading)
    #     self.pos[1] += ((time.time() - self._last) * speed) * math.sin(heading)
    #     self._last = time.time()

    def move_towards(self, node: int, speed: float):
        """
        Simulates robot motion, accounting for time since last call
        :param node: node ID
        :param speed: speed (m/s) to move
        """
        target_loc = get_node_loc(node)
        dist = haversine_dist(self.pos[0], self.pos[1], target_loc[0], target_loc[1])
        # TODO: optimize by passing in pos so node does not need to be located twice
        heading = calculate_heading(self.pos, node)
        heading = (math.pi / 2 - heading) % (2 * math.pi)
        dt = time.time() - self._last
        # TODO: account for rotation time to desired heading
        dx = dist * math.cos(heading)  # east/west
        dy = dist * math.sin(heading)
        # TODO: check
        dx *= dt * speed
        dy *= dt * speed
        # print(f"dist: {dist}, dt: {dt}, heading: {heading}, dx: {dx}, dy: {dy}")
        self._log.append({"dist": dist, "heading": heading, "dx": dx, "dy": dy, "pos": self.pos, "target": target_loc})
        # dx, dy are in meters
        # need to transform back to latlon

        # Convert latitude and longitude from degrees to radians
        lat_rad = math.radians(self.pos[0])
        lon_rad = math.radians(self.pos[1])

        # Offset in radians
        dlat = dy / earth_radius_m
        dlon = dx / (earth_radius_m * math.cos(lat_rad))

        self.pos[0] = math.degrees(lat_rad + dlat)
        self.pos[1] = math.degrees(lon_rad + dlon)

        self._last = time.time()

    def distance_to_node(self, node_id: int):
        node_loc = get_node_loc(node_id)
        return haversine_dist(self.pos[0], self.pos[1], node_loc[0], node_loc[1])

    def save_log(self, filename: str):
        with open(filename, "w") as file:
            json.dump(self._log, file, indent=2)


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


def calculate_heading(start_loc, node):
    """
    Calculates the heading to a given position
    :param start_loc: current pos
    :param node: desired node
    :return: heading needed in radians, clockwise from north
    """
    lat2, lon2 = get_node_loc(node)
    lat1 = start_loc[0] * 2 * math.pi / 360
    lon1 = start_loc[1] * 2 * math.pi / 360
    lat2 = lat2 * 2 * math.pi / 360
    lon2 = lon2 * 2 * math.pi / 360
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
    c = 2 * math.asin(math.sqrt(a))
    r = 6371
    return c * r * 1000


robot = Robot()
robot.pos = get_node_loc(0)
goal_loc = get_node_loc(1)
# print(robot.distance_to_node(1))
# time.sleep(1)
# robot.move_towards(1, 1)
# print(robot.distance_to_node(1))
while robot.distance_to_node(1) > 0.1:
    robot.move_towards(1, 0.5)
    print(robot.pos, robot.distance_to_node(1))
    # print(robot.pos)
    time.sleep(0.1)

robot.save_log("out.json")
