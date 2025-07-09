import os
import json

# Sample values for user 
model='AVIA' # 'AVIA', 'MID360'
return_mode='First single' # 'First single', 'Strongest single', 'Dual'
scan_pattern='Non-repetitive' #  'Repetitive', 'Non-repetitive'
tty='tty' 

mode = 1
pattern = 1
# model = $model
# return_mode = $return_mode 
# scan_pattern = $scan_pattern

# Lidar AVIA
if model == 'AVIA':
    dir_path = os.path.dirname(os.path.realpath(__file__))
    path = os.path.join(dir_path, "scripts/slam-ros2/fast_lio_slam/avia_config.json") 
    print(path)
    if return_mode == 'First single':
        mode = 0
    elif return_mode == 'Strongest single':
        mode = 1
    elif return_mode == 'Dual':
        mode = 2
    
    if scan_pattern == 'Repetitive':
        pattern = 1
    else:
        pattern = 0

with open(path, 'r') as f:
    data = json.load(f)
    
    data['lidar_config'][0]['return_mode'] = mode
    print(mode)
    data['lidar_config'][0]['scan_pattern'] = pattern
    print(pattern)

with open(path, 'w') as f:
    json.dump(data, f, indent=2)
