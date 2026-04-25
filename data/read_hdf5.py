import h5py
import matplotlib.pyplot as plt
import numpy as np

path = '/media/hjx/PSSD/hjx_ws/Dual_servo_gimbal/data/control_test/episode_0.hdf5'

obj = h5py.File(path)
print(obj.keys())
print(obj['action'])
print(obj['action'].keys())
print(obj['action']['target_pos'])
print('-------------------------')
print(obj['observations'])
print(obj['observations'].keys())
print('-------------------------')
print(obj['observations']['qpos'])
print(obj['observations']['qvel'])
print('-------------------------')

# print(obj['observations']['images'].keys())
# print(obj['observations']['images']['cam_high'])
# print('-------------------------')
