#!/usr/bin/env python3
"""
Shows how to toss a capsule to a container.
"""
from re import T
from mujoco_py import load_model_from_path, MjSim, MjViewer
import os
import math
import keyboard

model = load_model_from_path("/home/bobzhu/Unitree/Mujoco/unitree_mujoco_git/unitree_mujoco/data/a1/urdf/a1.xml")
sim = MjSim(model)

viewer = MjViewer(sim)

sim_state = sim.get_state()

i = 0
if_increase = True

desired_speed = [0.0] * 12
desired_pos = [0.0] * 12
Kp_vel= 7.0
Ki_vel = 0.8

Kp_pos= 10.0

q_vel_former = [0.0] * 12

speed_command = [0.0] * 12
speed_error = [0.0] * 12
speed_error_former = [0.0] * 12

angle_error = [0.0] * 12


k = 0
increase = True
while True:

    #sim.set_state(sim_state)
    
    # if k > 50:
    #     increase = False
    # elif k < -50:
    #     increase = True
    
    # if increase:
    #     k+=1
    # else:
    #     k-=1
     
    # # print(desired_pos)
    # for i in range(12):
    #     desired_pos[i] = 0.0
        
    #     q_pos_meas = sim.data.get_joint_qpos(sim.model.joint_names[i])
        
    #     angle_error[i] = desired_pos[i] - q_pos_meas
        
    #     if angle_error[i] < - math.pi:
    #         angle_error[i] = 2 * math.pi + angle_error[i]
    #     elif angle_error[i] >= math.pi:
    #         angle_error[i] = angle_error[i] - 2 * math.pi
        
    #     desired_speed[i] = Kp_pos * angle_error[i]
        
    #     q_vel_meas = sim.data.get_joint_qvel(sim.model.joint_names[i])
    #     #print(sim.data.get_joint_qvel(sim.model.joint_names[0]))

    #     speed_error[i] = desired_speed[i] - q_vel_meas
    #     speed_command[i] += Kp_vel * (speed_error[i] - speed_error_former[i]) + Ki_vel * speed_error[i]
    #     speed_error_former[i] = speed_error[i]
        
    #     if speed_command[i] > 33.5:
    #         speed_command[i] = 33.5
    #     elif speed_command[i] < -33.5:
    #         speed_command[i] = -33.5
        
    #     sim.data.ctrl[i] = speed_command[i]
        
    #sim.data.ctrl[:] = k / 10

    #print(joint_name, sim.data.get_joint_qpos(joint_name), sim.data.get_joint_qvel(joint_name))
    sim.step()
    viewer.render()
    #print(" ")
    
    if os.getenv('TESTING') is not None:
        break
