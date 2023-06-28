#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Robot controller
----------------

Manual controller script for robot controller testing.

https://github.com/turingbirds/robot-driver

Q: increase velocity M1
A: decrease velocity M1
Z: enable M1

W: increase velocity M2
S: decrease velocity M2
X: enable M2

E: increase velocity M3
D: decrease velocity M3
C: enable M3

Copyright 2023
Apache License 2.0
"""

from typing import List

import json
import math
import datetime
import pygame
import time
import socket
import sys


FPS_CAP = 20

UDP_IP = "192.168.1.184" #"125 184 229
UDP_PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(.09) # timeout in seconds

screen_w = 800
screen_h = 600
sc = pygame.display.set_mode((800, 600))

pygame.font.init()
font = pygame.font.Font("OpenSans_Italic.ttf", 32)
font_small = pygame.font.Font("OpenSans_Italic.ttf", 24)
blue = (0, 0, 255)
white=(255, 255, 255)
yellow=(255, 255, 0)
green=(0, 255, 255)
orange=(255, 100, 0)

# global state variables
state = {"vel_L": 0., "vel_C": 0., "vel_R": 0., "aux_pos_1": 90, "aux_pos_2": 90, "frame_idx": 0, "t": 0.}
exercise = False
enable = {"vel_L": False, "vel_C": False, "vel_R": False}

# velocity increment
delta_vel = 1

# servo angle increment
delta_ang = 5

# exercise
exercise_camera_amplitude_1 = 30.  # [deg]
exercise_camera_amplitude_2 = 30.  # [deg]
exercise_camera_freq_1 = .2  # [Hz]
exercise_camera_freq_2 = .2  # [Hz]

# start main loop
prev_udp_message = state.copy()
for k in state.keys():
	prev_udp_message[k] = float("inf")
clock = pygame.time.Clock()

prev = None

while True:

	sc.fill((0,255,0))

	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			sys.exit(0)

		if event.type == pygame.KEYUP:
			if event.key == pygame.K_SPACE:
				exercise = not exercise
				if not exercise:
					# exercise was stopped
					state["vel_L"] = 0.
					state["vel_C"] = 0.
					state["vel_R"] = 0.
					enable["vel_L"] = False
					enable["vel_C"] = False
					enable["vel_R"] = False

	keys = pygame.key.get_pressed()

	if keys[pygame.K_q]:
			state["vel_L"] += delta_vel
	if keys[pygame.K_a]:
			state["vel_L"] -= delta_vel
	if keys[pygame.K_z]:
			enable["vel_L"] = True
	else:
			enable["vel_L"] = False

	if keys[pygame.K_w]:
			state["vel_C"] += delta_vel
	if keys[pygame.K_s]:
			state["vel_C"] -= delta_vel
	if keys[pygame.K_x]:
			enable["vel_C"] = True
	else:
			enable["vel_C"] = False

	if keys[pygame.K_e]:
			state["vel_R"] += delta_vel
	if keys[pygame.K_d]:
			state["vel_R"] -= delta_vel
	if keys[pygame.K_c]:
			enable["vel_R"] = True
	else:
			enable["vel_R"] = False

	if keys[pygame.K_UP]:
			state["aux_pos_2"] += delta_ang
	if keys[pygame.K_DOWN]:
			state["aux_pos_2"] -= delta_ang
	if keys[pygame.K_LEFT]:
			state["aux_pos_1"] += delta_ang
	if keys[pygame.K_RIGHT]:
			state["aux_pos_1"] -= delta_ang

	if exercise:
		state["frame_idx"] += 1
		state["t"] = state["frame_idx"] / 30.

		on_ramp = 1.
		if state["frame_idx"] < 120:
			on_ramp = state["frame_idx"] / 120.

		state["aux_pos_1"] = 90. + on_ramp * exercise_camera_amplitude_1 * math.sin(2 * math.pi * exercise_camera_freq_1 * state["t"])
		state["aux_pos_2"] = 90. + on_ramp * exercise_camera_amplitude_2 * math.cos(2 * math.pi * exercise_camera_freq_2 * state["t"])
		#
		# if state["frame_idx"] % 40 == 0:
		# 	state["vel_L"] = 128. - state["vel_L"]
		#
		# if state["frame_idx"] % 50 == 0:
		# 	state["vel_R"] = 128. - state["vel_R"]

		if state["frame_idx"] % 2 == 0:
			state["vel_L"] = 128. - state["vel_L"]
			state["vel_C"] = 128. - state["vel_C"]
			state["vel_R"] = 128. - state["vel_R"]

		enable["vel_L"] = state["vel_L"] != 0
		enable["vel_R"] = state["vel_R"] != 0
		enable["vel_C"] = state["vel_C"] != 0

	text = font.render("Exercise: " + str(exercise), True, green, blue)
	textRect = text.get_rect()
	textRect.center = (screen_w // 2, screen_h // 2 - 140)
	sc.blit(text, textRect)

	text = font_small.render("press [space] to start/stop", False, green, blue)
	textRect = text.get_rect()
	textRect.center = (screen_w // 2, screen_h // 2 - 100)
	sc.blit(text, textRect)

	for topic in ["aux_pos_1", "aux_pos_2"]:
		state[topic] = max(state[topic], 0)
		state[topic] = min(state[topic], 180)

	for i, topic in enumerate(["vel_L", "vel_C", "vel_R"]):
		text = font.render("Motor " + topic + ": " + str(state[topic]), True, green, blue)
		textRect = text.get_rect()
		textRect.center = (screen_w // 2, screen_h // 2 + 60 * i)
		sc.blit(text, textRect)

	for i, topic in enumerate(["aux_pos_1", "aux_pos_2"]):
		text = font.render("Servo " + topic + ": " + str(state[topic]), True, green, blue)
		textRect = text.get_rect()
		textRect.center = (screen_w // 2, screen_h // 2 + 60 * (3 + i))
		sc.blit(text, textRect)

	udp_message = {}
	for topic in ["vel_L", "vel_C", "vel_R", "aux_pos_1", "aux_pos_2"]:
		value = 0
		if topic.startswith("vel"):
			if enable[topic]:
				udp_message[topic] = state[topic]
			else:
				udp_message[topic] = 0
		if topic.startswith("aux_pos"):
			udp_message[topic] = state[topic]

	#if str(prev_udp_message) != str(udp_message):   # only send a UDP packet when the state changed
	if True:  # send a message every frame -- prevent triggering the network timeout safety feature on the robot
		print("Sending: " + str(json.dumps(udp_message)))

		sock.sendto(bytes(json.dumps(udp_message), "utf-8"), (UDP_IP, UDP_PORT))

		try:
			data, addr = sock.recvfrom(1024)   # receive up to 1024 bytes of data
			print("received data" + str(data))
		except socket.error:
			pass

		prev_udp_message = udp_message.copy()

	pygame.display.flip()


	now = datetime.datetime.now()

	dt = clock.tick(FPS_CAP) # cap to this many fps

	if prev:
		print("Time per frame: " + str((now - prev).microseconds / 1000) + " ms (measured) = " + str(dt) + " (reported by pygame)")

	prev = now
