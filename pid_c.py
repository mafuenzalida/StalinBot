#!/usr/bin/env python
#-*- coding: utf-8 -*-

class PIDController:

	def __init__(self, Kp, Ki = 0, Kd = 0):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.Perror = 0
		self.Ierror = 0
		self.Derror = 0
		self.vel = 0

	def update_PID(self, error):
		self.Perror = error

		return self.Kp * error
