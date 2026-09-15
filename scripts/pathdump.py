#!/usr/bin/env python3
"""把 /plan_tool_path 的输出完整导出成 JSON，并打印每条路径的形态特征，
用于判断「杂乱路径」到底是几何上的什么。"""
import json

import rclpy
from rclpy.node import Node

from noether_ros.srv import PlanToolPath

CFG = ('/opt/snp_automate_2023/install/snp_automate_2023/share/'
       'snp_automate_2023/config/tpp.yaml')
MESH = '/workspace/snp_home/snp/meshes/results_mesh.ply'
OUT = '/workspace/snp_home/toolpaths.json'

rclpy.init()
node = Node('pathdump')
cli = node.create_client(PlanToolPath, '/plan_tool_path')
if not cli.wait_for_service(timeout_sec=30):
    print('!! /plan_tool_path 不可用')
    raise SystemExit(1)

req = PlanToolPath.Request()
req.config = CFG
req.mesh_file = MESH
req.mesh_frame = 'base_link'
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=300)
res = fut.result()
if res is None or not res.success:
    print('!! 失败:', None if res is None else res.message)
    raise SystemExit(1)

dump = []
paths = []
for group in res.tool_paths:
    for path in group.tool_paths:
        segs = []
        pts = []
        for seg in path.segments:
            s = []
            for p in seg.poses:
                q = p.orientation
                s.append({
                    'p': [p.position.x, p.position.y, p.position.z],
                    'q': [q.x, q.y, q.z, q.w],
                })
                pts.append((p.position.x, p.position.y, p.position.z))
            segs.append(s)
        paths.append(pts)
        dump.append({'segments': segs})

with open(OUT, 'w') as f:
    json.dump({'tool_paths': dump}, f)
print(f'已写出 {OUT}  ({len(dump)} 条路径)')

print()
print('=' * 78)
for i, pts in enumerate(paths):
    n = len(pts)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    # z 的"下探"次数：从 >0.25 掉到 <0.15 的次数，即贴着腿往下走的次数
    dips = sum(1 for a, b in zip(zs, zs[1:]) if a > 0.25 and b < 0.15)
    rises = sum(1 for a, b in zip(zs, zs[1:]) if a < 0.15 and b > 0.25)
    # 相邻点的三维间距，找跳变（相邻点距离远大于点距 0.015 说明有跳跃）
    import math
    jumps = [math.dist(pts[k], pts[k + 1]) for k in range(n - 1)]
    big = [round(d, 3) for d in jumps if d > 0.05]
    print(f'路径{i}: {n:3d}点  x[{min(xs):.3f},{max(xs):.3f}] '
          f'y[{min(ys):.3f},{max(ys):.3f}] z[{min(zs):.3f},{max(zs):.3f}]')
    print(f'        z 下探(坐面->腿){dips}次  回升{rises}次   段数={len(dump[i]["segments"])}')
    if big:
        print(f'        跳跃(>50mm) {len(big)} 处: {big[:8]}')
print('=' * 78)

rclpy.shutdown()
