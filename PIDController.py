import time


class PIDController:
    def __init__(self, p: float, i: float, d: float):
        """
        Creates a PID controller with the given P/I/D values
        :param p:
        :param i:
        :param d:
        """
        self.p = p
        self.i = i
        self.d = d
        self._current_setpoint = 0
        self._prev_err = 0
        self._accumulated_err = 0
        self._last_time = time.time()

    def calculate(self, current, goal):
        """
        Calculates the PID output for a given goal using the given setpoint. Updates the stored setpoint
        :param current: current position
        :param goal: desired position
        :return: PID output
        """
        self.set_goal(goal)
        error = goal - current
        dt = time.time() - self._last_time
        if dt == 0:
            dt = 0.000001
        # TODO: this can grow infinitely if we never reach the setpoint, clamp
        self._accumulated_err += error * dt
        p_out = self.p * error
        i_out = self.i * self._accumulated_err
        d_out = self.d * (error - self._prev_err) / dt
        self._prev_err = error
        # shhhh this won't be off by some tiny amount
        self._last_time = time.time()
        return p_out + i_out + d_out

    def reset(self):
        """
        Resets the current setpoint, previous error, and accumulated error.
        """
        self._current_setpoint = 0
        self._prev_err = 0
        self._accumulated_err = 0
        self._last_time = time.time()

    def set_goal(self, setpoint):
        """
        Sets the desired setpoint. Called by calculate() automatically
        :param setpoint: the desired setpoint
        """
        self._current_setpoint = setpoint

    # def calculate(self, goal):
    #     """
    #     Calculates the PID output for a given goal using the stored setpoint
    #     :param goal: the desired position
    #     :return: PID output
    #     """
    #     return self.calculate(self._current_setpoint, goal)
