import json
import math
import time

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

    def move(self, speed: float, heading: float):
        """
        Simulates robot motion, accounting for time since last call
        :param speed: speed in m/s
        :param heading: heading, in degrees
        """
        heading /= 180
        heading *= math.pi
        self.pos[0] += ((time.time() - self._last) * speed) * math.cos(heading)
        self.pos[1] += ((time.time() - self._last) * speed) * math.sin(heading)
        self._last = time.time()

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
    print(matches)
    return matches[0]


def calculate_heading(start_loc, node):
    lat2, lon2 = get_node_loc(node)
    lat1 = start_loc[0] * 2 * math.pi / 360
    lon1 = start_loc[1] * 2 * math.pi / 360
    lat2 = lat2 * 2 * math.pi / 360
    lon2 = lon2 * 2 * math.pi / 360
    delta_lon = lon2 - lon1
    x_c = math.sin(delta_lon) * math.cos(lat2)
    y_c = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    beta = math.atan2(x_c, y_c)
    beta_deg = beta * 360 / 2 / math.pi
    return beta_deg


robot = Robot()
time.sleep(1)
robot.pos = [0, 0]
robot.move(100, 180)
print(robot.pos)
