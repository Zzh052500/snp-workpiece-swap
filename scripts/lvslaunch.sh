#!/bin/bash
# 在「节点启动前」把 contact_check_lvs_distance 设成指定值，再测运动规划内存。
#
# 为什么要覆盖 launch 文件而不是 ros2 param set：
#   start.launch.xml 引入 planning_server.launch.xml 时没传 contact_check_lvs_distance，
#   所以它取 planning_server.launch.xml 自己的默认值。运行时 param set 可能改不动
#   已经构造好的 profile，必须改默认值让它在启动时就带上。
#
# 用 docker cp + docker restart（restart 保留容器文件系统），不改 compose，完全可逆。
C=snp_automate_2023_sim
CFG=/opt/snp_automate_2023/install/snp_automate_2023/share/snp_automate_2023/config/tpp.yaml
MESH=/workspace/snp_home/snp/meshes/results_mesh.ply
LVS=${1:-0.5}
HOSTDIR=/home/zzh/snp-automate-2023-polishing-simulation/runtime/snp_home
LAUNCH=/opt/snp/install/snp_motion_planning/share/snp_motion_planning/launch/planning_server.launch.xml
LOG=/workspace/snp_home/lvslaunch.log

node_alive() {
  docker exec $C bash -lc 'for d in /proc/[0-9]*; do [ "$(cat $d/comm 2>/dev/null)" = "snp_motion_plan" ] && exit 0; done; exit 1'
}

echo "=== [1] 取出原文件并备份 ==="
docker cp "$C:$LAUNCH" "$HOSTDIR/planning_server.launch.xml.orig" 2>&1 | tail -1
cp "$HOSTDIR/planning_server.launch.xml.orig" "$HOSTDIR/planning_server.launch.xml.new"
grep -o 'name="contact_check_lvs_distance" default="[^"]*"' "$HOSTDIR/planning_server.launch.xml.new" | sed 's/^/  原值: /'

echo "=== [2] 改默认值 -> $LVS ==="
sed -i "s|\(name=\"contact_check_lvs_distance\" default=\"\)[^\"]*|\1$LVS|" "$HOSTDIR/planning_server.launch.xml.new"
grep -o 'name="contact_check_lvs_distance" default="[^"]*"' "$HOSTDIR/planning_server.launch.xml.new" | sed 's/^/  新值: /'

echo "=== [3] 放进容器 ==="
docker cp "$HOSTDIR/planning_server.launch.xml.new" "$C:$LAUNCH"

echo "=== [4] 重启容器 ==="
docker restart $C >/dev/null
for i in $(seq 1 30); do
  if node_alive; then echo "  第 $((i*2))s：节点就绪"; break; fi
  sleep 2
done
sleep 8

echo "=== [5] 确认参数在新实例里是 $LVS ==="
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=42; timeout 15 ros2 param get /snp_planning_server contact_check_lvs_distance" 2>&1 | tail -1 | sed 's/^/  /'

echo "=== [6] 监视器 ==="
docker exec $C bash -lc "rm -f /workspace/snp_home/mon.pid; echo '=== lvslaunch=$LVS ===' > $LOG"
docker exec -d $C bash -lc "echo \$\$ > /workspace/snp_home/mon.pid; exec bash /workspace/snp_home/mon.sh $LOG"
sleep 2

echo "=== [7] 开火 ==="
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; source /opt/snp/install/setup.bash; export ROS_DOMAIN_ID=42; timeout 240 python3 -u /workspace/snp_home/prechk.py $CFG $MESH" 2>&1 | grep -vE "运行中|^  File|^    " | tail -22

echo "=== [8] 停监视器 ==="
docker exec $C bash -lc 'kill "$(cat /workspace/snp_home/mon.pid)" 2>/dev/null; echo stopped'

echo "=== [9] 节点还活着吗 ==="
if node_alive; then echo "  ✓ 存活"; else echo "  ✗ 死了"; fi
