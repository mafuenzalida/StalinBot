#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
from move_c import MoveController
from vision_c import VisionController
from speaker_c import SpeakerController

class TurtleBot:
	def __init__(self):
		rospy.init_node('robot', anonymous=True)
		self.move = MoveController()
		self.vision = VisionController()
		self.speaker = SpeakerController()
