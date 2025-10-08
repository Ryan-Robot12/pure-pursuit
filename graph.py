import json

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

# plt.scatter(x, data["current_heading"], c="red")
# plt.scatter(x, data["desired_heading"], c="orange")
plt.scatter(x, data["drive_err"][:], c="red")
plt.scatter(x, data["drive_out"][:], c="orange")
plt.scatter(x, data["turn_err"], c="green")
plt.scatter(x, data["turn_out"], c="blue")
plt.savefig("graph.png")
