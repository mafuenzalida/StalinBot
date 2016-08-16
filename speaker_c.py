#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
from sound_play.msg import SoundRequest

class SpeakerController:

	def __init__(self):
		self.pub = rospy.Publisher('robotsound', SoundRequest)

	def speak(self, msg):
		s = SoundRequest()
		s.sound = s.SAY
		s.command = s.PLAY_ONCE
		s.arg = msg
		self.pub.publish(s)

	def speak_direction(self, msg, direction):
		s = SoundRequest()
		s.sound = s.SAY
		s.command = s.PLAY_ONCE
		aux = ""
		if(direction == 0):
			aux = "UP"
		elif(direction == 1):
			aux = "LEFT"
		elif(direction == 2):
			aux = "DOWN"
		else:
			aux = "RIGHT"
		s.arg = msg + " " + aux
		self.pub.publish(s)

