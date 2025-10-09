import json
from copy import deepcopy

import matplotlib.pyplot as plt

with open("data.json") as file:
    data = json.load(file)

"""
"current_pos": self.current_pos,
"target_pos": target_loc,
"drive_err": dist,
"turn_err": desired_heading - self.current_heading,
"desired_heading": desired_heading,
"current_heading": self.current_heading,
"drive_out": drive_output,
"turn_out": turn_output,
"dt": time.time() - self._last_time
"""
blank_data = {
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
split_by_nodes = [deepcopy(blank_data)]
last_tgt_loc = data["target_pos"][0]
keys = list(blank_data.keys())
for i in range(len(data["current_pos"])):
    if data["target_pos"][i] != last_tgt_loc:
        split_by_nodes.append(deepcopy(blank_data))
    for key in keys:
        split_by_nodes[-1][key].append(data[key][i])
    last_tgt_loc = data["target_pos"][i]


size = 8

# for node in range(len(split_by_nodes)):
#     x = [i for i in range(len(split_by_nodes[node]["drive_err"]))]
#     # plt.scatter(x, data["current_heading"], c="red")
#     # plt.scatter(x, data["desired_heading"], c="orange")
#     plt.scatter(x, split_by_nodes[node]["drive_err"][:], c="red", s=size)
#     plt.scatter(x, split_by_nodes[node]["drive_out"][:], c="orange", s=size)
#     plt.scatter(x, split_by_nodes[node]["turn_err"], c="green", s=size)
#     plt.scatter(x, split_by_nodes[node]["turn_out"], c="blue", s=size)
#     plt.savefig(f"graphs/node_{node + 1}.png")
#     plt.clf()
x = [i for i in range(len(data["drive_err"]))]

plt.scatter(x, data["drive_err"][:], c="red", s=size)
plt.scatter(x, data["drive_out"][:], c="orange", s=size)
plt.scatter(x, data["turn_err"], c="green", s=size)
plt.scatter(x, data["turn_out"], c="blue", s=size)

plt.savefig("graph.png")