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

import paho.mqtt.client as mqtt
import pygame
import time
import sys

mqtt_host = "10.15.2.73"

client = mqtt.Client(client_id="robot_controller_manual", clean_session=True, userdata=None, protocol= mqtt.MQTTv311, transport="tcp")
client.connect(mqtt_host, port=1883, keepalive=60, bind_address="")

screen_w = 800
screen_h = 600
sc = pygame.display.set_mode((800, 600))

pygame.font.init()
font = pygame.font.Font('OpenSans_Italic.ttf', 32)
blue = (0, 0, 255)
white=(255, 255, 255)
yellow=(255, 255, 0)
green=(0, 255, 255)
orange=(255, 100, 0)

# global state variables
vel: List[int] = 3 * [0]
enable: List[bool] = 3 * [False]
servo_ang: List[int] = 2 * [90]

# velocity increment
delta_vel = 1

# servo angle increment
delta_ang = 1

# start main loop
clock = pygame.time.Clock()

while True:
  clock.tick(30) # cap to 30 fps

  sc.fill((0,255,0))

  keys = pygame.key.get_pressed()
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      sys.exit(0)

  if keys[pygame.K_q]:
      vel[0] += delta_vel
  if keys[pygame.K_a]:
      vel[0] -= delta_vel
  if keys[pygame.K_z]:
      enable[0] = True
  else:
      enable[0] = False

  if keys[pygame.K_w]:
      vel[1] += delta_vel
  if keys[pygame.K_s]:
      vel[1] -= delta_vel
  if keys[pygame.K_x]:
      enable[1] = True
  else:
      enable[1] = False

  if keys[pygame.K_e]:
      vel[2] += delta_vel
  if keys[pygame.K_d]:
      vel[2] -= delta_vel
  if keys[pygame.K_c]:
      enable[2] = True
  else:
      enable[2] = False

  if keys[pygame.K_UP]:
      servo_ang[0] += delta_ang
  if keys[pygame.K_DOWN]:
      servo_ang[0] -= delta_ang
  if keys[pygame.K_LEFT]:
      servo_ang[1] += delta_ang
  if keys[pygame.K_RIGHT]:
      servo_ang[1] -= delta_ang
  for i in range(2):
      servo_ang[i] = max(servo_ang[i], 0)
      servo_ang[i] = min(servo_ang[i], 180)

  for i in range(3):
    text = font.render('Motor ' + str(i) + ': ' + str(vel[i]), True, green, blue)
    textRect = text.get_rect()
    textRect.center = (screen_w // 2, screen_h // 2 + 60 * i)
    sc.blit(text, textRect)

  for i in range(2):
    text = font.render('Servo ' + str(i) + ': ' + str(servo_ang[i]), True, green, blue)
    textRect = text.get_rect()
    textRect.center = (screen_w // 2, screen_h // 2 + 60 * (3 + i))
    sc.blit(text, textRect)

  topic = "vr/vel_L"
  value = 0
  if enable[0]:
    value = vel[0]
  client.publish(topic, payload=value, qos=0, retain=False)

  topic = "vr/vel_C"
  value = 0
  if enable[1]:
    value = vel[1]
  client.publish(topic, payload=value, qos=0, retain=False)

  topic = "vr/vel_R"
  value = 0
  if enable[2]:
    value = vel[2]
  client.publish(topic, payload=value, qos=0, retain=False)

  topic = "vr/aux_pos_1"
  value = servo_ang[0]
  client.publish(topic, payload=value, qos=0, retain=False)

  topic = "vr/aux_pos_2"
  value = servo_ang[1]
  client.publish(topic, payload=value, qos=0, retain=False)

  pygame.display.flip()
