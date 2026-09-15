#!/usr/bin/env python3
"""直接拿已导出的 toolpaths.json 去调 /generate_motion_plan，复现那个 OOM。

绕开 /plan_tool_path（它依赖 RViz 里手工放置的 located vector）。
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node

from snp_msgs.srv import GenerateMotionPlan
from snp_msgs.msg import ToolPath as SnpToolPath
from geometry_msgs.msg import PoseArray, Pose

SRC = '/workspace/snp_home/toolpaths.json'
MOTION_GROUP = 'manipulator'
TCP_FRAME = 'sand_tcp'


def mem_mb():
    with open('/sys/fs/cgroup/memory.current') as f:
        return int(f.read()) / 1024 / 1024


def top(n=4):
    out = []
    for p in os.listdir('/proc'):
        if not p.isdigit():
            continue
        try:
            kv = {}
            with open(f'/proc/{p}/status') as f:
                for line in f:
                    if line.startswith(('VmRSS:', 'Name:')):
                        k, v = line.split(':', 1)
                        kv[k] = v.strip()
            if 'VmRSS' not in kv:
                continue
            out.append((int(kv['VmRSS'].split()[0]) / 1024, int(p), kv.get('Name', '?')))
        except OSError:
            continue
    out.sort(reverse=True)
    return ' | '.join(f'{nm}({pid}) {mb:.0f}M' for mb, pid, nm in out[:n])


rclpy.init()
node = Node('mpchk')

with open(SRC) as f:
    dump = json.load(f)['tool_paths']

paths = []
npts = 0
for entry in dump:
    tp = SnpToolPath()
    for seg in entry['segments']:
        pa = PoseArray()
        pa.header.frame_id = 'base_link'
        for w in seg:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = w['p']
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = w['q']
            pa.poses.append(pose)
        tp.segments.append(pa)
        npts += len(seg)
    paths.append(tp)

print(f'从 {SRC} 载入 {len(paths)} 条路径 / {npts} 点')
print(f'基线容器内存 {mem_mb():.0f} MiB / 3072 MiB')
print()

cli = node.create_client(GenerateMotionPlan, '/generate_motion_plan')
if not cli.wait_for_service(timeout_sec=30):
    print('!! /generate_motion_plan 不可用')
    sys.exit(1)

req = GenerateMotionPlan.Request()
req.tool_paths = paths
req.motion_group = MOTION_GROUP
req.tcp_frame = TCP_FRAME

fut = cli.call_async(req)
t0 = time.time()
last = 0.0
while not fut.done():
    rclpy.spin_until_future_complete(node, fut, timeout_sec=0.5)
    el = time.time() - t0
    if el - last >= 1.0:
        last = el
        print(f'  {el:6.1f}s  容器 {mem_mb():7.0f} MiB   {top()}', flush=True)
    if el > 180:
        print('!! 超过 180s')
        break

print(f'\n总用时 {time.time() - t0:.1f}s')
if fut.done() and fut.result() is not None:
    r = fut.result()
    print(f'success = {r.success}   message = {r.message!r}')
    for nm in ('approach', 'process', 'departure'):
        tr = getattr(r, nm)
        print(f'  {nm:10s}: {len(tr.points)} 个轨迹点')
else:
    print('>>> future 未完成（服务很可能已死）')
rclpy.shutdown()
