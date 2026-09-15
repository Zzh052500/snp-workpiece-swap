#!/usr/bin/env python3
"""只挑一条指定的光栅路径去做运动规划。

用法: oneplan.py <tpp配置> <网格> <路径下标> [路径下标...]
用来把「姿态是否一致」和「规划器本身是否爆内存」两个变量切开。
"""
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from noether_ros.srv import PlanToolPath
from snp_msgs.srv import GenerateMotionPlan
from snp_msgs.msg import ToolPath as SnpToolPath

CFG = sys.argv[1]
MESH = sys.argv[2]
WANT = [int(a) for a in sys.argv[3:]] or [0]
CG = '/sys/fs/cgroup'


def cg(p):
    try:
        return open(f'{CG}/{p}').read().strip()
    except OSError:
        return '?'


def mem_mb():
    v = cg('memory.current')
    return int(v) / 1024 / 1024 if v.isdigit() else -1.0


PEAK = [0.0]
STOP = threading.Event()


def sampler():
    while not STOP.is_set():
        m = mem_mb()
        if m > PEAK[0]:
            PEAK[0] = m
        STOP.wait(0.25)


def node_rss():
    for p in __import__('os').listdir('/proc'):
        if not p.isdigit():
            continue
        try:
            with open(f'/proc/{p}/comm') as f:
                if f.read().strip() != 'snp_motion_plan':
                    continue
            with open(f'/proc/{p}/status') as f:
                for ln in f:
                    if ln.startswith('VmRSS'):
                        return int(ln.split()[1]) / 1024
        except OSError:
            pass
    return -1.0


rclpy.init()
node = Node('oneplan')
threading.Thread(target=sampler, daemon=True).start()

cli = node.create_client(PlanToolPath, '/plan_tool_path')
cli.wait_for_service(timeout_sec=25)
req = PlanToolPath.Request()
req.config = CFG
req.mesh_file = MESH
req.mesh_frame = 'base_link'
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=300)
res = fut.result()
if res is None or not res.success:
    print('!! 刀路失败:', None if res is None else res.message)
    raise SystemExit(1)

groups = []
for g in res.tool_paths:
    for p in g.tool_paths:
        st = SnpToolPath()
        st.segments = p.segments
        groups.append(st)

sel = [groups[i] for i in WANT if i < len(groups)]
npts = sum(len(s.poses) for p in sel for s in p.segments)
print(f'共 {len(groups)} 条路径，选中 {WANT} -> {len(sel)} 条 / {npts} 点')

m = node.create_client(GenerateMotionPlan, '/generate_motion_plan')
m.wait_for_service(timeout_sec=25)
mreq = GenerateMotionPlan.Request()
mreq.tool_paths = sel
mreq.motion_group = 'manipulator'
mreq.tcp_frame = 'sand_tcp'

rss0 = node_rss()
t0 = time.time()
mf = m.call_async(mreq)
while not mf.done():
    rclpy.spin_until_future_complete(node, mf, timeout_sec=1.0)
    el = time.time() - t0
    if el > 240:
        print('!! 超时')
        break
STOP.set()
time.sleep(0.3)

rss1 = node_rss()
print(f'用时 {time.time()-t0:.1f}s')
print(f'节点 RSS {rss0:.0f} -> {rss1:.0f} MiB' + (f'   存活' if rss1 > 0 else '   ✗ 死了'))
print(f'容器峰值 {PEAK[0]:.0f} MiB   memory.events={cg("memory.events")}')
if mf.done() and mf.result() is not None:
    r = mf.result()
    print(f'success = {r.success}   message = {r.message!r}')
    for nm in ('approach', 'process', 'departure'):
        tr = getattr(r, nm)
        print(f'  {nm:10s}: {len(tr.points)} 点')
rclpy.shutdown()
