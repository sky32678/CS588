#!/usr/bin/python

import time
import numpy as np


class PID:
    def __init__(
        self,
        Kp=0.0,
        Ki=0.0,
        Kd=0.0,
        set_point=0.0,
        sample_time=0.01,
        out_limits=(None, None),
    ):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0

        self.set_point = set_point

        self.sample_time = sample_time

        self.out_limits = out_limits

        self.last_err = 0.0

        self.last_time = time.time()

        self.output = 0.0

    def update(self, feedback_val):
        """Compute PID control value based on feedback_val.
        """
        self.p_term = self.Kp * feeback_val
        self.i_term += self.Ki * feedback_val * self.sample_time
        self.d_term = self.Kd * ((feedback_val - self.last_err) / self.sample_time)

        if control_law > self.out_limits[1]:
            control_law = self.out_limits[1]
        elif control_law < self.out_limits[0]:
            control_law = self.out_limits[0]
        else:
            self.i_term = self.i_term

        control_law = self.p_term + self.i_term + self.d_term

        self.last_err = feedback_val
        self.output = control_law

        return control_law


    def __call__(self, feeback_val):
        return self.update(feeback_val)
