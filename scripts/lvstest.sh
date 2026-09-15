#!/bin/bash
# 测试 contact_check_lvs_distance 对 /generate_motion_plan 内存的影响。
# 用法: lvstest.sh [lvs值]   默认 0.25
C=snp_automate_2023_sim
CFG=/opt/snp_automate_2023/install/snp_automate_2023/share/snp_automate_2023/config/tpp.yaml
MESH=/workspace/snp_home/snp/meshes/results_mesh.ply
LVS=${1:-0.25}
LOG=/workspace/snp_home/lvs_test.log

node_alive() {
  docker exec $C bash -lc 'for d in /proc/[0-9]*; do [ "$(cat $d/comm 2>/dev/null)" = "snp_motion_plan" ] && exit 0; done; exit 1'
}

echo "=== 重启容器 ==="
docker restart $C >/dev/null

echo "=== 轮询等节点（每 2s，最多 60s）==="
ready=0
for i in $(seq 1 30); do
  if node_alive; then echo "  第 $((i*2))s：节点就绪"; ready=1; break; fi
  sleep 2
done
[ "$ready" = "1" ] || { echo "  等不到，放弃"; exit 1; }
sleep 8

echo "=== 设置 contact_check_lvs_distance = $LVS ==="
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=42; timeout 20 ros2 param set /snp_planning_server contact_check_lvs_distance $LVS" 2>&1 | tail -1
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=42; timeout 15 ros2 param get /snp_planning_server contact_check_lvs_distance" 2>&1 | tail -1 | sed 's/^/  确认: /'

echo "=== 启动监视器（PID 落盘）==="
docker exec $C bash -lc "rm -f /workspace/snp_home/mon.pid; echo '=== lvs=$LVS ===' > $LOG"
docker exec -d $C bash -lc "echo \$\$ > /workspace/snp_home/mon.pid; exec bash /workspace/snp_home/mon.sh $LOG"
sleep 2

echo "=== 开火：7 条路径 / 294 点 ==="
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; source /opt/snp/install/setup.bash; export ROS_DOMAIN_ID=42; timeout 240 python3 -u /workspace/snp_home/prechk.py $CFG $MESH" 2>&1 | grep -vE "运行中" | tail -28

echo "=== 停监视器（只杀 mon.pid）==="
docker exec $C bash -lc 'kill "$(cat /workspace/snp_home/mon.pid)" 2>/dev/null; echo stopped'

echo "=== 节点还活着吗 ==="
if node_alive; then echo "  ✓ 存活"; else echo "  ✗ 死了"; fi
