#!/usr/bin/env bash
# 把本项目（工件替换 + 容器加固）覆盖到 SNP 仿真仓库上。
#
# 背景：仿真仓库里那 40 多个改动（tpp.yaml、compose、网格、脚本）都在本机工作区，
# 并没有提交上去。新机器上 git clone 下来的是原厂状态，必须用本脚本覆盖。
#
# 用法: install-into-sim.sh [仿真仓库路径]
#       默认 $HOME/snp-automate-2023-polishing-simulation
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM="${1:-$HOME/snp-automate-2023-polishing-simulation}"
STAMP="$(date +%Y%m%d-%H%M%S)"

[ -d "$SIM/docker" ] || { echo "!! 找不到仿真仓库: $SIM"; exit 1; }
echo "本项目   : $HERE"
echo "仿真仓库 : $SIM"
echo

bak() { [ -f "$1" ] && cp -v "$1" "$1.bak-$STAMP"; }

echo "==> 1/4 tpp.yaml"
mkdir -p "$SIM/config"; bak "$SIM/config/tpp.yaml"
cp -v "$HERE/config/tpp.yaml" "$SIM/config/tpp.yaml"

echo "==> 2/4 docker/compose.sim.yml（含 3g 上限等加固）"
mkdir -p "$SIM/docker"; bak "$SIM/docker/compose.sim.yml"
cp -v "$HERE/docker/compose.sim.yml" "$SIM/docker/compose.sim.yml"

echo "==> 3/4 坐面网格（两处都要，缺一不可）"
mkdir -p "$SIM/meshes" "$SIM/runtime/snp_home/snp/meshes"
bak "$SIM/meshes/part_scan.ply"
bak "$SIM/runtime/snp_home/snp/meshes/results_mesh.ply"
cp -v "$HERE/artifacts/seat_only.ply" "$SIM/meshes/part_scan.ply"
cp -v "$HERE/artifacts/seat_only.ply" "$SIM/runtime/snp_home/snp/meshes/results_mesh.ply"
echo "    md5 = $(md5sum "$HERE/artifacts/seat_only.ply" | cut -d' ' -f1)"
echo "    这个文件是 ASCII PLY，2256 顶点 / 4082 面，z 0.270~0.317"

echo "==> 4/4 诊断脚本 -> runtime/snp_home/"
mkdir -p "$SIM/runtime/snp_home"
for f in prechk.py pathdump.py tpptest.py meshcheck.py meshz.py seatcut.py \
         orient.py oneplan.py prof.py mon.sh cfgtest.sh abtest.sh yamltest.sh profrun.sh; do
  [ -f "$HERE/scripts/$f" ] && cp -v "$HERE/scripts/$f" "$SIM/runtime/snp_home/"
done

cat <<EOF

================================================================
完成。
下一步：
  cd $SIM
  ./scripts/restart_demo.sh        # docker rm -f + compose up -d

脚本里引用仿真仓库路径时用 SNP_SIM_DIR，默认就是
  \$HOME/snp-automate-2023-polishing-simulation
放在别处的话先 export SNP_SIM_DIR=<路径>
================================================================
EOF
