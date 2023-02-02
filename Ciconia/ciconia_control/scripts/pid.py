#! /usr/bin/python3

class pid:

    def __init__(self, p, i, d, output_constraints = [None, None], anti_windup = None):
        self.kp = p
        self.ki = i
        self.kd = d

        self.error = 0
        self.error_prev  = 0
        self.error_sum = 0

        self.anti_windup = anti_windup
        self.cons = output_constraints


    def calculate_control_input(self, ref, feedback, dt):
        error           = ref - feedback
        self.error_sum += (error + self.error_prev) / 2 * dt
        error_rate      = (error - self.error_prev) / dt
        if self.anti_windup != None:
            self.error_sum  = self.integral_anti_windup(self.error_sum * self.ki)
        self.error_prev = error

        signal =  error * self.kp + self.error_sum * self.ki + error_rate * self.kd
        return self.check_constraints(signal)


    def integral_anti_windup(self, integral_error):
        if integral_error > self.anti_windup:
           return self.anti_windup
        elif integral_error < -self.anti_windup:
           return -self.anti_windup
        else:
           return integral_error / self.ki


    def check_constraints(self, signal):
        if self.cons[0] != None and signal < self.cons[0]:
            signal = self.cons[0]

        if self.cons[1] != None and signal > self.cons[1]:
            signal = self.cons[1]

        return signal


