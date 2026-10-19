#!/usr/bin/env python3
"""Write autoware_light.rviz: the stock autoware.rviz with lighter pointcloud/image displays.

- PointCloudMap         -> /display/pointcloud_map (voxel copy, published once by display_map_pub.py)
- ConcatenatePointCloud -> /display/lidar, NoGroundPointCloud -> /display/obstacle_pointcloud
  (throttled + subsampled by display_cloud_throttle.py: rviz2 leaks GPU memory per cloud message)
- NDT debug clouds (NDT pointclouds, NDTLoadedPCDMap, Initial, Aligned) and the occupancy grid Map display off
Edits the text line by line (a YAML round-trip turns values like 'Off' into booleans and RViz then
rejects the display config). Re-run after autoware.rviz changes:  python3 make_light_rviz.py
"""
import os
import re

SRC = os.path.expanduser('~/autoware.mach_e/src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz')
DST = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'autoware_light.rviz')

# name -> (topic, durability, reliability)
RETOPIC = {'PointCloudMap': ('/display/pointcloud_map', 'Transient Local', 'Reliable'),
           'ConcatenatePointCloud': ('/display/lidar', 'Volatile', 'Best Effort'),
           'NoGroundPointCloud': ('/display/obstacle_pointcloud', 'Volatile', 'Best Effort')}
OFF_NAMES = {'NDT pointclouds', 'NDTLoadedPCDMap', 'Initial', 'Aligned'}
# rviz2 Humble's Map display leaks GPU memory on the moving occupancy grid (~1.5 GB/min on 2026-10-15)
OFF_MAPS = {'/perception/occupancy_grid_map/map'}
OFF_CLASSES = set()  # disabling the Image/Camera displays (dock panels) makes RViz fail with 'Could not load display config: Invalid argument'

lines = open(SRC).read().split('\n')
indent = lambda s: len(s) - len(s.lstrip(' '))


def display_blocks():
    """Yield (start, end) line ranges of every list item containing a 'Class:' key."""
    for i, line in enumerate(lines):
        m = re.match(r'^( *)(- )?Class: (\S+)', line)
        if not m:
            continue
        # the list item starts at the nearest '- ' line at or above with indentation <= this key
        key_ind = indent(line) + (2 if m.group(2) else 0)
        start = i
        while not re.match(r'^ *- ', lines[start]) or indent(lines[start]) + 2 != key_ind:
            start -= 1
            if start < 0 or indent(lines[start]) + 2 < key_ind and not lines[start].lstrip().startswith('- '):
                start = None
                break
        if start is None:
            continue
        end = start + 1
        while end < len(lines) and (lines[end].strip() == '' or indent(lines[end]) >= key_ind):
            end += 1
        yield start, end, m.group(3)


changes = []
for start, end, cls in display_blocks():
    key_ind = indent(lines[start]) + 2
    own = lambda j: indent(lines[j]) == key_ind or (j == start)
    name = None
    for j in range(start, end):
        m = re.match(r'^ *(- )?Name: (.*)$', lines[j])
        if m and own(j):
            name = m.group(2).strip()
    if cls == 'rviz_default_plugins/PointCloud2' and name in RETOPIC:
        topic, dur, rel = RETOPIC[name]
        for j in range(start, end):
            s = lines[j]
            if own(j) and re.match(r'^ *Decay Time:', s):
                lines[j] = re.sub(r'Decay Time: .*', 'Decay Time: 0', s)
            if re.match(r'^ *Topic:\s*$', s) and own(j):
                k = j + 1
                while k < end and indent(lines[k]) > key_ind:
                    t = lines[k]
                    t = re.sub(r'Value: .*', f'Value: {topic}', t)
                    t = re.sub(r'Depth: .*', 'Depth: 1', t)
                    t = re.sub(r'Durability Policy: .*', f'Durability Policy: {dur}', t)
                    t = re.sub(r'Reliability Policy: .*', f'Reliability Policy: {rel}', t)
                    lines[k] = t
                    k += 1
        changes.append(f'{name} -> {topic}')
    elif cls == 'rviz_default_plugins/Map' and any(t in '\n'.join(lines[start:end]) for t in OFF_MAPS):
        for j in range(start, end):
            if own(j) and re.match(r'^ *Enabled: true', lines[j]):
                lines[j] = lines[j].replace('Enabled: true', 'Enabled: false')
                changes.append(f'{name} (occupancy grid Map) off')
    elif (cls == 'rviz_default_plugins/PointCloud2' and name in OFF_NAMES) or cls in OFF_CLASSES:
        for j in range(start, end):
            if own(j) and re.match(r'^ *Enabled: true', lines[j]):
                lines[j] = lines[j].replace('Enabled: true', 'Enabled: false')
                changes.append(f'{name} ({cls}) off')

open(DST, 'w').write('\n'.join(lines))
print('\n'.join(changes))
print('wrote', DST)
