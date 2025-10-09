def clamp(val, min_, max_):
    return min(max(val, min_), max_)


def input_modulus(input_, minimumInput, maximumInput):
    diff = maximumInput - minimumInput
    while input_ > maximumInput:
        input_ -= diff
    while input_ < minimumInput:
        input_ += diff
    return input_


print(input_modulus(2 - 1.5, -1, 1))
# data = {
#     "desired": [],
#     "current": [],
#     "err": [],
#     "output": []
# }
#
# controller = PIDController(1, 0, 0)
# controller.enable_continuous_input(0, 2)
# desired = 0
# current = 0.7
# last_time = time.time()
# max_err = 1
#
# try:
#     while abs(current - desired) > 0.01:
#         output = controller.calculate(current, desired)
#         current += output * (time.time() - last_time)
#         current %= 2
#         clamp(output, -1, 1)
#         heading_err = input_modulus(desired - current, -max_err, max_err)
#         data["desired"].append(desired)
#         data["current"].append(current)
#         data["err"].append(heading_err)
#         print(f"err: {heading_err}")
#         data["output"].append(output)
#         time.sleep(0.1)
# except KeyboardInterrupt:
#     pass
#
# with open("out.json", "w") as file:
#     json.dump(data, file, indent=2)
