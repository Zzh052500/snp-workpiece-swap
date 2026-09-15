#!/bin/bash
# 复现 OOM 并录下逐进程内存曲线。
# 要点：轮询等待规划节点就绪（最多 80s），一就绪立刻开火，不硬等固定时长。
C=snp_automate_2023_sim
R=/home/zzh/snp-automate-2023-polishing-simulation
CFG=/opt/snp_automate_2023/install/snp_automate_2023/share/snp_automate_2023/config/tpp.yaml
MESH=/workspace/snp_home/snp/meshes/results_mesh.ply

node_alive() {
  docker exec $C bash -lc 'for d in /proc/[0-9]*; do [ "$(cat $d/comm 2>/dev/null)" = "snp_motion_plan" ] && exit 0; done; exit 1'
}

echo "=== 重启容器 ==="
docker restart $C >/dev/null

echo "=== 轮询等待规划节点（每 2s 一次，最多 80s）==="
ready=0
for i in $(seq 1 40); do
  if node_alive; then
    echo "  第 $((i*2))s：规划节点已出现"
    ready=1
    break
  fi
  sleep 2
done
if [ "$ready" = "0" ]; then
  echo "  80s 内没等到，放弃"
  exit 1
fi

# 再给它 12s 完成环境配置
sleep 12

echo "=== 启动 0.2s 粒度内存监视器 ==="
docker exec $C bash -lc 'pkill -f "while true"; echo "=== OOM 现场 ===" > /workspace/snp_home/memwatch.log'
docker exec -d $C bash -lc 'while true; do echo "$(cat /sys/fs/cgroup/memory.current) | $(ps -eo rss,pid,comm --sort=-rss --no-headers | head -5 | tr "\n" " ")"; sleep 0.2; done >> /workspace/snp_home/memwatch.log 2>&1'
sleep 2

echo "=== 开火：真实配置 + 坐面网格 -> 运动规划 ==="
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; source /opt/snp/install/setup.bash; export ROS_DOMAIN_ID=42; timeout 90 python3 -u /workspace/snp_home/prechk.py $CFG $MESH" 2>&1 | tail -30

echo "=== 完成 ==="
