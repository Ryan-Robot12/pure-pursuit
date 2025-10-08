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
x = [i for i in range(1, len(data["drive_err"]) + 1)]
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
for i in range(len(data)):
    if data["target_pos"][i][0] != last_tgt_loc[0]:
        split_by_nodes.append(deepcopy(blank_data))
    for key in keys:
        split_by_nodes[-1][key].append(data[key][i])
    last_tgt_loc = data["target_pos"][i]


print(split_by_nodes[0])
# plt.scatter(x, data["current_heading"], c="red")
# plt.scatter(x, data["desired_heading"], c="orange")
plt.scatter(x, split_by_nodes[0]["drive_err"][:], c="red", s=1)
plt.scatter(x, split_by_nodes[0]["drive_out"][:], c="orange", s=1)
plt.scatter(x, split_by_nodes[0]["turn_err"], c="green", s=1)
plt.scatter(x, split_by_nodes[0]["turn_out"], c="blue", s=1)
plt.savefig("graph.png")
