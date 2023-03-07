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

import math
import paho.mqtt.client as mqtt
import pygame
import time
import sys

mqtt_host = "localhost"

client = mqtt.Client(client_id="robot_controller_manual", clean_session=True, userdata=None, protocol= mqtt.MQTTv311, transport="tcp")
client.connect(mqtt_host, port=1883, keepalive=60, bind_address="")

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

mqtt_prefix: str = "vr/"

# global state variables
state = {"vel_L": 0., "vel_C": 0., "vel_R": 0., "aux_pos_1": 90, "aux_pos_2": 90, "frame_idx": 0, "t": 0., "exercise": False}
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
prev_state = state.copy()
for k in state.keys():
  prev_state[k] = float("inf")
clock = pygame.time.Clock()

while True:
  clock.tick(30) # cap to 30 fps

  sc.fill((0,255,0))

  keys = pygame.key.get_pressed()
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      sys.exit(0)

    if event.type == pygame.KEYUP:
      if event.key == pygame.K_SPACE:
        state["exercise"] = not state["exercise"]
        if not state["exercise"]:
          # exercise was stopped
          state["vel_L"] = 0.
          state["vel_C"] = 0.
          state["vel_R"] = 0.
          enable["vel_L"] = False
          enable["vel_C"] = False
          enable["vel_R"] = False

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

  if state["exercise"]:
    state["frame_idx"] += 1
    state["t"] = state["frame_idx"] / 30.

    on_ramp = 1.
    if state["frame_idx"] < 120:
      on_ramp = state["frame_idx"] / 120.

    state["aux_pos_1"] = 90. + on_ramp * exercise_camera_amplitude_1 * math.sin(2 * math.pi * exercise_camera_freq_1 * state["t"])
    state["aux_pos_2"] = 90. + on_ramp * exercise_camera_amplitude_2 * math.cos(2 * math.pi * exercise_camera_freq_2 * state["t"])

    if state["frame_idx"] % 40 == 0:
      state["vel_L"] = 128. - state["vel_L"]

    if state["frame_idx"] % 50 == 0:
      state["vel_R"] = 128. - state["vel_R"]

    if state["frame_idx"] % 60 == 0:
      state["vel_C"] = 128. - state["vel_C"]

    enable["vel_L"] = state["vel_L"] != 0
    enable["vel_R"] = state["vel_R"] != 0
    enable["vel_C"] = state["vel_C"] != 0

  text = font.render("Exercise: " + str(state["exercise"]), True, green, blue)
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

  for topic in ["vel_L", "vel_C", "vel_R", "aux_pos_1", "aux_pos_2"]:
    value = 0
    if topic.startswith("vel") and enable[topic]:
      value = state[topic]
    if topic.startswith("aux_pos"):
      value = state[topic]

    if value != prev_state[topic]:
      prev_state[topic] = value
      client.publish(mqtt_prefix + topic, payload=value, qos=0, retain=False)

  pygame.display.flip()
