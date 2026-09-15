#!/usr/bin/env python3
"""运动规划预检 —— 复现行为树 Progress 60~80 那一步，但不碰 UI。

流程（与 config/snp_automate.xml 完全一致的参数）：
  1. /plan_tool_path          -> noether_ros/ToolPaths[]
  2. /generate_motion_plan    -> approach / process / departure
     motion_group = manipulator, tcp_frame = sand_tcp

全程每 0.25s 采样容器 cgroup 内存，实时打印，以便在逼近 3GiB 上限前手动叫停。
"""
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from noether_ros.srv import PlanToolPath
from snp_msgs.srv import GenerateMotionPlan
from snp_msgs.msg import ToolPath as SnpToolPath

CFG = ('/opt/snp_automate_2023/install/snp_automate_2023/share/'
       'snp_automate_2023/config/tpp.yaml')
MESH = '/workspace/snp_home/snp/meshes/results_mesh.ply'

if len(sys.argv) > 1:
    CFG = sys.argv[1]
if len(sys.argv) > 2:
    MESH = sys.argv[2]
MESH_FRAME = 'base_link'
MOTION_GROUP = 'manipulator'
TCP_FRAME = 'sand_tcp'

CG = '/sys/fs/cgroup'


def cg(path):
    try:
        with open(os.path.join(CG, path)) as f:
            return f.read().strip()
    except OSError:
        return '?'


def mem_mb():
    v = cg('memory.current')
    return int(v) / 1024 / 1024 if v.isdigit() else -1.0


PEAK = [0.0]
STOP = threading.Event()


def top_procs(n=3):
    """容器内 RSS 最大的 n 个进程 -> [(MB, pid, name), ...]（PID 命名空间是容器的）"""
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
    return out[:n]


def mem_report():
    tp = top_procs()
    return ' | '.join(f'{nm}({pid}) {mb:.0f}M' for mb, pid, nm in tp)


def sampler():
    while not STOP.is_set():
        m = mem_mb()
        if m > PEAK[0]:
            PEAK[0] = m
        STOP.wait(0.25)


def call(node, srv_type, name, req, timeout):
    """同步调用，但每 5s 打印一次心跳 + 内存，避免看起来像卡死。"""
    cli = node.create_client(srv_type, name)
    if not cli.wait_for_service(timeout_sec=25):
        print(f'!! 服务不可用: {name}')
        return None
    fut = cli.call_async(req)
    t0 = time.time()
    last = 0.0
    while not fut.done():
        rclpy.spin_until_future_complete(node, fut, timeout_sec=1.0)
        el = time.time() - t0
        if el - last >= 2.0:
            last = el
            print(f'   ... {name} 运行中 {el:6.1f}s   容器内存 {mem_mb():7.0f} MiB '
                  f'(峰值 {PEAK[0]:.0f})   最大进程: {mem_report()}', flush=True)
        if el > timeout:
            print(f'!! {name} 超过 {timeout}s 仍未返回，放弃等待')
            return None
    print(f'   {name} 用时 {time.time() - t0:.1f}s', flush=True)
    return fut.result()


def main():
    rclpy.init()
    node = Node('prechk')
    th = threading.Thread(target=sampler, daemon=True)
    th.start()

    print('=' * 68)
    print('基线：容器内存 %.0f MiB / 上限 %s MiB' %
          (mem_mb(), int(cg('memory.max')) // 1024 // 1024 if cg('memory.max').isdigit() else '?'))
    print('=' * 68, flush=True)

    # ---- 第 1 步：出刀路 -------------------------------------------------
    print('\n[1/2] /plan_tool_path  —— 生成工具路径')
    req = PlanToolPath.Request()
    req.config = CFG
    req.mesh_file = MESH
    req.mesh_frame = MESH_FRAME
    res = call(node, PlanToolPath, '/plan_tool_path', req, timeout=300)
    if res is None:
        print('!! 刀路规划未返回')
        return 2
    if not res.success:
        print(f'!! 刀路规划失败: {res.message!r}')
        return 2

    snp_paths = []
    for group in res.tool_paths:          # noether_ros/ToolPaths
        for path in group.tool_paths:     # noether_ros/ToolPath
            st = SnpToolPath()
            st.segments = path.segments   # 两者字段结构相同
            snp_paths.append(st)

    # 可选：只取前 N 条路径做运动规划，用来判断内存是否随点数线性缩放
    if len(sys.argv) > 3:
        keep = int(sys.argv[3])
        print(f'   !! 实验：只取前 {keep} 条路径（共 {len(snp_paths)} 条）')
        snp_paths = snp_paths[:keep]

    npts = sum(len(s.poses) for p in snp_paths for s in p.segments)
    print(f'   组数={len(res.tool_paths)}  路径数={len(snp_paths)}  总点数={npts}')
    for i, p in enumerate(snp_paths):
        k = sum(len(s.poses) for s in p.segments)
        print(f'     路径{i}: {k} 点 / {len(p.segments)} 段')

    # ---- 第 2 步：运动规划（就是会压死机器的那一步）------------------------
    print(f'\n[2/2] /generate_motion_plan  —— 对 {npts} 个点做运动规划')
    print(f'   motion_group={MOTION_GROUP}  tcp_frame={TCP_FRAME}')
    print(f'   容器内存上限 3072 MiB，swap 为 0；超限会被 OOM kill。\n', flush=True)

    mreq = GenerateMotionPlan.Request()
    mreq.tool_paths = snp_paths
    mreq.motion_group = MOTION_GROUP
    mreq.tcp_frame = TCP_FRAME
    mres = call(node, GenerateMotionPlan, '/generate_motion_plan', mreq, timeout=1800)

    STOP.set()
    time.sleep(0.3)

    print('\n' + '=' * 68)
    print(f'容器内存峰值 = {PEAK[0]:.0f} MiB   (上限 3072 MiB)')
    print(f'cgroup memory.peak = {cg("memory.peak")}')
    print(f'cgroup memory.events = {cg("memory.events")}')
    print('=' * 68)

    if mres is None:
        print('>>> 运动规划未返回（超时或被 OOM kill）')
        return 3

    print(f'success = {mres.success}')
    print(f'message = {mres.message!r}')
    for nm in ('approach', 'process', 'departure'):
        tr = getattr(mres, nm)
        n = len(tr.points)
        dur = tr.points[-1].time_from_start.sec + tr.points[-1].time_from_start.nanosec / 1e9 if n else 0.0
        print(f'  {nm:10s}: {n:5d} 个轨迹点, 时长 {dur:.1f}s')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
