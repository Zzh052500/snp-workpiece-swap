#!/usr/bin/env python3
"""确认重建后推给 RViz 的是坐面。

/industrial_reconstruction_mesh 的类型是 visualization_msgs/Marker
（TRIANGLE_LIST），不是 shape_msgs/Mesh —— 这是 industrial_reconstruction
的约定，RViz 用 Marker 显示网格。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker

TRIANGLE_LIST = Marker.TRIANGLE_LIST

rclpy.init()
node = Node('meshcheck')

qos = QoSProfile(depth=1)
qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
qos.reliability = ReliabilityPolicy.RELIABLE
qos.history = HistoryPolicy.KEEP_LAST

got = []


def cb(msg):
    got.append(msg)
    pts = msg.points
    kind = 'TRIANGLE_LIST' if msg.type == TRIANGLE_LIST else 'type=%d' % msg.type
    print('收到 Marker: type=%s  ns=%r  id=%d' % (kind, msg.ns, msg.id))
    print('  frame_id = %r' % msg.header.frame_id)
    print('  点数 = %d  ->  三角面 %d' % (len(pts), len(pts) // 3))
    print('  颜色 = rgba(%.2f,%.2f,%.2f,%.2f)' % (msg.color.r, msg.color.g, msg.color.b, msg.color.a))
    if pts:
        xs = [p.x for p in pts]
        ys = [p.y for p in pts]
        zs = [p.z for p in pts]
        print('  bbox x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'
              % (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)))
        if min(zs) > 0.26:
            print('  >>> 是坐面（z 全部 > 0.26，没有腿）')
        else:
            print('  >>> 含腿（z 低到 %.3f）' % min(zs))


node.create_subscription(Marker, '/industrial_reconstruction_mesh', cb, qos)
print('等待 /industrial_reconstruction_mesh ...')
for _ in range(60):
    rclpy.spin_once(node, timeout_sec=0.5)
    if got:
        break
if not got:
    print('!! 没收到（可能已过 latched 有效期，重新触发一次 stop_reconstruction 即可）')
rclpy.shutdown()
