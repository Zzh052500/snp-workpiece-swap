#!/usr/bin/env python3
"""在容器里盯着 snp_motion_planning_node，0.15s 抓一次 /proc/PID/maps。

目的：判断那 ~2.1GB 是「一整块匿名映射」在涨（一次性大分配），
还是「映射数量暴增」（泄漏/碎片）。顺带看有没有文件映射在涨。
"""
import collections
import os
import sys
import time

LOG = sys.argv[1]
out = open(LOG, 'a', buffering=1)


def find_pid():
    for p in os.listdir('/proc'):
        if not p.isdigit():
            continue
        try:
            with open('/proc/%s/comm' % p) as f:
                if f.read().strip() == 'snp_motion_plan':
                    return p
        except OSError:
            pass
    return None


while True:
    pid = find_pid()
    if pid:
        agg = collections.Counter()
        nmaps = 0
        rss = 0
        try:
            with open('/proc/%s/maps' % pid) as f:
                for ln in f:
                    parts = ln.split(None, 5)
                    a, b = parts[0].split('-')
                    sz = (int(b, 16) - int(a, 16)) / 1048576.0
                    nm = parts[5].strip() if len(parts) > 5 else '[anon]'
                    # 把带偏移的同一文件归并
                    nm = nm.split(' (deleted)')[0]
                    agg[nm] += sz
                    nmaps += 1
            with open('/proc/%s/status' % pid) as f:
                for ln in f:
                    if ln.startswith('VmRSS'):
                        rss = int(ln.split()[1]) / 1024
            top = '  '.join('%s=%.0fM' % (k, v) for k, v in agg.most_common(5))
            out.write('RSS=%6.0fM  maps=%5d  %s\n' % (rss, nmaps, top))
        except OSError:
            pass
    time.sleep(0.15)
