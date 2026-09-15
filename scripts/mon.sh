#!/bin/bash
# 内存监视器：0.2s 粒度记录容器 memory.current + RSS 前 4 名进程。
# 由外部脚本用 `exec bash mon.sh` 启动，PID 写在 mon.pid，停的时候 kill 那个 PID。
LOG=${1:-/workspace/snp_home/lvs_test.log}
while true; do
  echo "$(cat /sys/fs/cgroup/memory.current) | $(ps -eo rss,pid,comm --sort=-rss --no-headers | head -4 | tr '\n' ' ')"
  sleep 0.2
done >> "$LOG" 2>&1
