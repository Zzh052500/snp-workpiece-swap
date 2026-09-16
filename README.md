# SNP 打磨仿真：替换工件为「坐面」

在 SNP Automate 2023 打磨仿真（ROS 2 Jazzy + Tesseract）里，把默认工件换成一张**凳子坐面板**，
并让刀路生成变得**干净、自动、可复现**；同时记录一个运动规划 OOM 故障的**定位过程与结论**。

> **一句话结论**：刀路那半边已经做到了「7 条纯坐面光栅线 / 294 点 / 零跳变 / 逐点可复现」。
>
> **运动规划（`/generate_motion_plan`）的内存开销至今未能收敛**——
> 它至少要吃掉 **5.2 GB 以上**，而且看起来**没有上界**。
>
> - 3 GB 上限 → 节点涨到 2.3 GB 撞墙（[§4.2](#42-关键证据一与刀路点数完全无关)）
> - 6 GB 上限 → 节点涨到 **5.2 GB** 撞墙，**斜率全程不降**（**§4.12**）
>
> ⚠️ **早先「这是有界的正常开销、换台大内存机器即可解决」的结论已作废。**
> 那个判断建立在「3 GB 截断处读到的 2.3 GB 就是峰值」的误读上；
> 上限一翻倍，读数跟着翻倍，**说明它一直在撞墙，从没到过峰值**。
> 上游 `docs/DEMO_RESULT.md` 那句"抛光运动规划完成"是文字声明、无内存数据，
> 且发生在本项目加固之前（当时是**吃光 2 GB swap、整机卡死**才跑完的）。
>
> **现在"换大内存机器到底行不行"本身是未知数，必须上服务器实测。**
> 本机 7.5 GB 已经没有余量再往上试了。方法见 [§9](#9-在另一台机器上跑大内存)。
>
> 附带结论一：界面上 `remove_scan_link` 报 unreachable **有两种成因**，
> 排查别只看一种 —— ① 节点被 OOM 打死后服务一起消失（[§4.10](#410-remove_scan_link-unreachable-是同一个-oom-的下游症状)）；
> ② 真机容器与仿真容器**共享 DDS 域**导致服务名冲突（**§10.2**）。
>
> 附带结论二：`RPly: Error reading 'vertex_indices'` + 节点 `exit -11` 段错误
> **与内存无关**，是 `results_mesh.ply` 被截断（**§10.1**）。
> **上服务器前务必先确认这个文件是好的**，否则会把它误当成 OOM 继续查。

> **最后更新：2026-09-16。** 本机（7.5 GB 内存、swap 已用 1.7 GB）实测 6 GB 上限仍不够，
> **判定实验已无法在本机完成**。上服务器后按 [§9](#9-在另一台机器上跑大内存) 跑一次 32g 收敛实验。

---

## 目录

- [1. 背景](#1-背景)
- [2. 工件替换](#2-工件替换)
- [3. tpp.yaml 的两处修改](#3-tppyaml-的两处修改)
- [4. 运动规划 OOM：现象、定位与结论](#4-运动规划-oom现象定位与结论)
- [5. 容器加固](#5-容器加固)
- [6. 复现步骤](#6-复现步骤)
- [7. 文件清单](#7-文件清单)
- [8. 环境](#8-环境)
- [9. 在另一台机器上跑（大内存）](#9-在另一台机器上跑大内存)
- [10. 2026-09-16 新增的两个故障模式（上服务器前必读）](#10-2026-09-16-新增的两个故障模式上服务器前必读)

---

## 1. 背景

仿真跑在一个 Docker 容器里（`snp_automate_2023_sim`，`ROS_DOMAIN_ID=42`）。
整条链路是：

```
点云扫描 → industrial_reconstruction 重建网格 → /plan_tool_path 出刀路 → /generate_motion_plan 做运动规划
   ↑ 仿真模式（SNP_SIM_VISION=true）直接给网格，不需要真相机
```

目标（用户原话）：**「先把座面能够稳定打磨并且可以复现」**。
「可复现」是硬要求，这直接决定了下面两处配置修改的取舍。

---

## 2. 工件替换

### 2.1 为什么原来的工件不行

原工件是一个架在桌面上的**球拱面**。换成凳子后，橙色的 `ROISelection` 圈选失效了：

`ROISelection`（`snp_tpp/roi_selection_mesh_modifier.h`）读 RViz 多边形选择工具的圈选结果，
交给 noether 的 `ExtrudedPolygonSubsetExtractor`。后者的算法是
**拟合平面 → 投影边界 → 沿拟合平面法向把多边形拉成棱柱 → 取棱柱内顶点**。

也就是说它取的是**柱体**，不是你在屏幕上圈出来的那块曲面。
对球拱面没问题（竖直柱体切出来的正好是一块干净曲面），但对凳子：

> 坐面下方的竖直柱体**必然把四条腿一起包进来**。
> 实测圈选坐面时柱内顶点 z 从 0.087 贯穿到 0.317，坐面 + 四条腿全在里面。

栅格规划器在「带腿柱体」上生成不出刀路 → 返回空 `tool_paths` → 行为树下游对空 vector 取 `at(0)`
→ `std::vector::_M_range_check` → `Behavior tree did not complete successfully`。

只要腿在坐面下方，**这是几何上的死结**，调 `min_cluster_size` / `plane_distance_threshold` 都无解。

### 2.2 切出坐面

用 z 阈值把腿切掉（`scripts/seatcut.py`）：

```python
SRC = '/workspace/snp_home/snp/meshes/results_mesh.ply'
DST = '/workspace/snp_home/seat_only.ply'
THR = 0.27
keep = (V[F][:, :, 2] >= THR).all(axis=1)   # 三个顶点都在阈值之上的面才保留
```

阈值是看 z 分布定的（`scripts/meshz.py`）：

| z 区间 | 内容 |
|---|---|
| 0.087 | 桌面接触点（360 个点） |
| 0.087 – 0.269 | 四条腿 |
| 0.279 – 0.317 | 坐面（0.298 处峰值，945 个点） |

不同阈值下的面数：

| 阈值 | 0.20 | 0.25 | **0.27** | 0.28 | 0.29 |
|---|---|---|---|---|---|
| 面数 | 7262 | 4724 | **4082** | 3242 | 2702 |

**切割结果（实测）**：

```
原网格  6463 顶点 / 12032 面
坐面板  2256 顶点 /  4082 面     文件 125494 字节  md5 9af94bbe191e3278890a6422b573e047
bbox    x[0.658, 0.938]  y[-0.095, 0.115]  z[0.270, 0.317]
尺寸    0.280 × 0.210 × 0.047 m
z < 0.26 的顶点数：0        ← 确认没有腿
```

### 2.3 装进仿真

坐面网格要同时放到**两个**位置，并各自备份原工件：

```bash
meshes/part_scan.ply                                  # 原始工件（刀路生成的输入）
runtime/snp_home/snp/meshes/results_mesh.ply          # 重建输出的落点
```

```bash
cp meshes/part_scan.ply                       meshes/part_scan.ply.bak-stool            # 376780 字节
cp runtime/snp_home/snp/meshes/results_mesh.ply runtime/.../results_mesh.ply.bak-stool
```

改完**必须重启容器**（`docker restart snp_automate_2023_sim`）。

### 2.4 验证 RViz 里推的确实是坐面

调用 `/stop_reconstruction` 把网格推到 RViz，然后用 `scripts/meshcheck.py` 订阅
`/industrial_reconstruction_mesh` 检查。

> **踩坑**：这个话题的类型是 `visualization_msgs/msg/Marker`（`TRIANGLE_LIST`），
> **不是** `shape_msgs/Mesh`。这是 `industrial_reconstruction` 的约定——RViz 用 Marker 显示网格。
> 用 `shape_msgs/Mesh` 订阅会静默收不到任何消息。

实测输出：

```
收到 Marker: type=TRIANGLE_LIST  ns=''  id=1
  frame_id = 'base_link'
  点数 = 12246  ->  三角面 4082
  bbox x[0.658,0.938] y[-0.095,0.115] z[0.270,0.317]
  >>> 是坐面（z 全部 > 0.26，没有腿）
```

12246 / 3 = 4082，与 `seat_only.ply` 的面数、bbox 完全一致。

---

## 3. tpp.yaml 的两处修改

`config/tpp.yaml` 是刀路生成的配置。改了**两处**，**两个修改是独立的**，各自解决一个不同的问题：

### 3.1 停用 `ROISelection`

去掉「只打磨圈选区域」的柱体提取（理由见 2.1）。因为腿已经切掉了，
「竖直柱体必然包住腿」的死结不存在了。

保留的 modifier：`NormalsFromMeshFaces`。规划器参数未动：
`line_spacing` / `point_spacing` = 0.03，`min_hole_size` / `min_segment_size` = 0.1，
`bidirectional: true`。

### 3.2 `LocatedVectorDirection` → `FixedDirection`

**这是「可复现」的关键。** 原配置用 `LocatedVectorDirection`，它的坐标来自 RViz 的
`LocatedVector` 工具，而 `config/app.rviz` 里只保存了线颜色和服务名，**没有保存起点终点**：

```yaml
- Class: noether_ros/LocatedVector
  Line color: 128; 128; 0
  Render as overlay: true
  Service name: get_located_vector
```

后果：**容器/RViz 每次重启，located vector 就丢了**，必须手工再拖一次；没拖之前
`/plan_tool_path` 一律报：

```
Error invoking tool path planner ... Start point has not been set
```

这是整条链路里**唯一的纯手工、不可复现环节**，所以换成固定方向：

```yaml
direction_generator:
  name: FixedDirection
  direction:
    x: 1.0
    y: 0.0
    z: 0.0
```

> **YAML 形状陷阱**：`direction` 必须是 `{x, y, z}` **映射**。
> 写成 `[1.0, 0.0, 0.0]` 会报 `Required member 'x' not found in YAML node`。

`x: 1.0` 与原来手拖出来的方向一致（手拖时 source→target 实测约 `(+0.126, -0.013, -0.001)`）。

### 3.3 验证结果

`config/` 是**只读挂载**到容器里的，noether 服务器每次 `plan_tool_path` 都会重读配置，
所以**改完不需要重启**。

```
config = .../snp_automate_2023/config/tpp.yaml
mesh   = /workspace/snp_home/snp/meshes/results_mesh.ply
success = True  message = ''

路径0:  39点  x[0.659,0.936] y[0.099,0.099] z[0.274,0.319]  跳变0
路径1:  43点  x[0.658,0.934] y[0.069,0.069] z[0.282,0.319]  跳变0
路径2:  44点  x[0.658,0.933] y[0.039,0.039] z[0.281,0.319]  跳变0
路径3:  43点  x[0.658,0.934] y[0.009,0.009] z[0.282,0.319]  跳变0
路径4:  43点  x[0.658,0.934] y[-0.021,-0.021] z[0.282,0.319]  跳变0
路径5:  43点  x[0.658,0.934] y[-0.051,-0.051] z[0.281,0.319]  跳变0
路径6:  39点  x[0.659,0.937] y[-0.081,-0.081] z[0.275,0.318]  跳变0

合计 7 条路径 / 294 点
```

特征：沿 **+X 平行**、每条路径 **y 恒定**、z 只在 **0.274–0.319**（纯坐面高度）、
**零跳变**。跑两遍逐点相同 ⟹ **可复现**。

---

## 4. 运动规划 OOM：现象、定位与结论

> **状态（2026-09-16 更新）：现象与范围已完全查清，但"是不是 bug"仍未定论。**
>
> ⚠️ 本节写于 2026-09-15，当时判断「这是正常开销、不是 bug、有界」。
> **该判断已在 2026-09-16 被 6 GB 上限实测推翻（见 §4.12）**：
> 内存随上限翻倍而线性上涨、斜率不收敛，更像**无界增长**而非正常开销。
>
> 下面是**现象与实测数据**，仍然有效且准确；**但结论请以 §4.12 为准**。

### 4.1 现象

在 **3 GiB 上限**下，任何 `/generate_motion_plan` 请求都会让 `snp_motion_planning_node`
在约 6 秒内从 ~130 MiB 涨到 **2.3 GB**，撞顶，被 OOM kill（`exit code -9`）。

**截至 2026-09-15：14 次请求、14 次死亡，无一幸免——但全部是在 3 GiB 上限下。**

⚠️ 「无一幸免」这句话**只在有上限时成立**。去掉上限它在这台机器上跑通过一次（§4.7 证据 4、
§4.11）。读本节时请始终带着这个前提。

```
Received motion planning request   14 次
Motion Planner process succeeded   14 次
process has died (exit -9)         15 次   ← 多出的一次是启动早期的另一次崩溃
```

即每一次规划请求都**确实规划成功了**，然后在返回结果的阶段把内存吃爆被杀掉——
不是随机故障，是确定性故障。

容器本身活着，但**节点死了 ⟹ 它提供的所有服务一起消失**，RViz 侧表现为：

```
[ Remove Scan Link ]        ->  FAILED  Service 'remove_scan_link' is unreachable
[ GenerateMotionPlanService ]-> FAILED  Service 'generate_motion_plan' is unreachable
```

### 4.2 关键证据一：与刀路点数**完全无关**

这是最能说明问题的一条。用 `scripts/prechk.py` 的缩放对照（第 3 个参数 = 只取前 N 条路径）：

| 输入 | 节点 RSS 起始 → 峰值 | 最陡上升 | 容器峰值 |
|---|---|---|---|
| **7 条路径 / 294 点** | 132 → **2358 MiB** | +2230 MiB / **5.8 s** | 3072（撞顶） |
| **1 条路径 / 39 点** | 130 → **2304 MiB** | +2174 MiB / **6.0 s** | 3072（撞顶） |

输入点数差了 **7.5 倍**，内存曲线几乎一模一样。

**结论：调 `point_spacing`、砍路径数、切网格都救不了。** 这条思路到此为止。

原始曲线在 `evidence/memwatch-7paths-294pts.log` 和 `evidence/memwatch-1path-39pts.log`
（0.2 秒粒度，逐进程 RSS，容器内每 0.2 s 采样一次 `memory.current` + `ps` 前 4 名）。

容器内存撞顶时的进程构成（`evidence/memwatch-7paths-294pts.log` 峰值那一行）：

```
容器 memory.current = 3072 MiB
   snp_motion_plan    pid=76     2331 MiB   ← 元凶
   python3            pid=74      278 MiB   ← 测试客户端，清白
   rviz2              pid=73      131 MiB
   joint_state_pub    pid=67       96 MiB
```

### 4.3 关键证据二：爆炸发生在**规划成功之后**

日志顺序是确定的（原始摘录见 `evidence/event-order.txt`）：

```
Received motion planning request
  → Failed to find or load library ... (×9)        ← 见 4.4，无害
  → KDL LMA Failed to calculate IK ... (×30)
  → Motion Planner process succeeded               ← 规划已经成功了
  → cache hit! ×664                                ← 内存在这里失控
  → process has died [pid 102, exit code -9]
```

字符串定位：

| 消息 | 出处 |
|---|---|
| `Environment, getKinematicGroup(manipulator, ) cache hit!` | `libtesseract_environment.so` |
| `Motion Planner process succeeded` | `tesseract_task_composer/planning/nodes/motion_planner_task.hpp` |
| `KDL LMA Failed to calculate IK, increment joints are tool small` | Tesseract KDL 求解器 |

### 4.4 已排除的干扰项

- ❌ **插件加载失败不是根因。**
  ```
  Failed to find or load library:
    /opt/tesseract_planning/install/tesseract_task_composer/lib/libsnp_motion_planning_tasks.so
    with error: Bad file descriptor
  ```
  库实际装在 `/opt/snp/install/snp_motion_planning/lib/`。这是 `boost_plugin_loader`
  **探测备用路径失败后继续**，插件最终正常加载，规划确实 `succeeded`。
- ❌ **不是测试客户端**：python3 全程 278–445 MiB。
- ❌ **不是坐面网格 / `FixedDirection`**：它们解决的是「刀路干净可复现」，与 OOM 无关。
  （这一点我一开始判断错了——曾以为换网格能顺带解决 OOM，实测证明不能。）

### 4.5 全参数消融：内存恒定在 ~2.1 GB

> 2026-09-15 补测。把能拧的旋钮逐个拧到底，节点内存增量**纹丝不动**。

| 变体 | 节点 RSS 增量 |
|---|---|
| 基线（坐面, lvs=0.05, Taskflow `threads: 8`） | **+2220 MiB** |
| 刀路缩到 1 条 / 39 点 | +2161 MiB |
| `contact_check_lvs_distance` = 0.25 | +2031 MiB |
| `contact_check_lvs_distance` = 0.50 | +1969 MiB |
| Taskflow 执行器 `threads: 8 → 1` | +2176 MiB |
| **工件换回原厂凳子**（`results_mesh.ply.bak-stool`） | +2168 MiB |
| `collision_object_type: convex_mesh → mesh`（跳过凸分解） | +2028 MiB |

三条独立结论：

1. **不是工件。** 换回未经任何改动的原厂凳子（6463 顶点 / 12032 面）照样炸，
   增量 +2168 vs 坐面 +2176，几乎一模一样。
   **这推翻了「是我们的网格触发的」这个假设**——此前一直认为坐面网格是嫌疑。
2. **不是碰撞检查精度。** `contact_check_lvs_distance` 粗 10 倍，内存只降 11%。
3. **不是并行度。** Taskflow 执行器线程 8→1 无变化。

流水线定义在
`/opt/snp/install/snp_motion_planning/share/snp_motion_planning/config/task_composer_plugins.yaml`
（**在镜像里，不在挂载的 `config/` 下**）：

```
SNPPipeline:
  FormatInputTask → SimpleMotionPlannerTask → DescartesMotionPlannerTask
                  → RasterMotionTask → TCPSpeedLimiterTask → Done
```

`RasterMotionTask` 内部对每段再跑 `SNPCartesianPipeline` / `SNPFreespacePipeline`，
这两个子流水线里每个 planner 任务后面都紧跟一个 `DiscreteContactCheckTask`。

⚠️ **注意一处推理陷阱**：`contact_check_lvs_distance` 只管**连续**碰撞检查的采样步长，
而 `DiscreteContactCheckTask` 做的是**离散**检查，**根本不读这个参数**。
所以「调 lvs 无效」**不能**用来排除接触检查任务——它至今仍是嫌疑之一，
只是「与输入无关」这一点更指向环境/求解器的固定开销。

复现脚本（都在 `scripts/`）：

| 脚本 | 作用 |
|---|---|
| `cfgtest.sh <sed表达式> <标签>` | 改 `planning_server.launch.xml` 的一个 arg 默认值 |
| `abtest.sh <网格路径> <标签>` | 换 `results_mesh.ply`（碰撞环境用的网格） |
| `yamltest.sh <sed表达式> <标签>` | 改 task composer 配置 |

三者都走 `docker cp` + `docker restart`，**不改镜像、不重建容器、完全可逆**。

### 4.6 关键证据三：内存形态是「一次性大分配」，不是泄漏

> 2026-09-15 补测。爬升期每 0.15 秒抓一次 `/proc/<pid>/maps`（`scripts/prof.py`）。

```
RSS=  257M  maps=1518  [anon]=2539M  libvtkCommonCore=10M  ...
RSS=  590M  maps=1518  [anon]=2827M  ...
RSS= 1254M  maps=1522  [anon]=3544M  ...
RSS= 2213M  maps=1526  [anon]=4597M  ...
RSS=    0M  maps=   0   ← 被杀
```

**爬升期间映射数只从 1518 涨到 1526（仅 +8 个），却涨了 1956 MiB。**

即：**8 个约 250 MB 的大匿名块在同时长大**，不是成千上万个小分配的泄漏。
`memory.current` 里增长的全部是 `[anon]`（匿名内存），没有文件映射在涨。

`nproc = 8` —— 每个核一个大缓冲。但 `threads: 1` 并没有改变总量（见 4.5），
说明开这些缓冲的不是 Taskflow 的线程池，而是**按硬件并发数自行分配的组件**：
节点直接链接了 `libgomp.so.1`，二进制里含 `GOMP_parallel` / `omp_set_num_threads`。

### 4.7 结论（2026-09-15）——⚠️ 已于 2026-09-16 更正

> ⚠️ **本节结论已被新证据推翻，请以 **§4.12** 为准。**
> 保留本节原文是为了留下判断的演进过程。核心更正一句话：
> 当时判「有界」，是因为只测到 **3 GB 上限截断处**的 2.3 GB，误把那道墙当成了终点；
> 上限放到 **6 GB** 后，节点一路**线性**涨到 **5.2 GB** 仍无任何收敛迹象。

> **最可能的解释：这 ~2.1 GB 是 TrajOpt + 完整碰撞环境的正常开销，不是 bug。**

支持这个判断的四条：

1. **与输入完全无关**（4.2、4.5）——是环境的固定开销，不随工作量增长。
2. **容器 3 GiB 上限是后加的加固措施**（见 §5 的 compose 注释）。加之前容器无限制，
   结果是**整台主机连同 2 GB swap 一起被拖死**。
   也就是说：这个规划步骤本来就要吃掉 **>3 GB**，多到能拖死一台 5.7 GB 的主机。
3. 上游 SNP Automate 2023 是 ROS-Industrial 的官方项目，**默认跑在工作台/工作站上**，
   内存通常是本机的 5–10 倍。
4. **★ 它在这台机器上真的跑通过一次。** 上游仓库的 `docs/DEMO_RESULT.md` 记录，
   截至 **2026-08-19**「抛光运动规划完成」，且是在 `SNP_BYPASS_EXECUTION=true` 下
   跑完了整条流水线。那一次用的就是**原厂无上限的 compose**（`git show HEAD:docker/compose.sim.yml`
   里只有一行 `image:`，没有任何 `mem_limit`）。

~~证据 4 的分量最重：**同一台 5.7 GB 的机器、同一份镜像，在无上限时能收敛**。
所以这 ~2.1 GB **是有界的**——这一点基本已经确定，不必等换机再验。~~

> ⚠️ **上面这句已经作废（2026-09-16）。** 两个问题：
> 1. **证据 4 本身不成立。** `docs/DEMO_RESULT.md` 是一句**文字声明**（全文 553 字节），
>    没有任何内存数据支撑，而且那份"成功"**发生在本项目加固之前**，
>    当时主机是靠吃掉 2 GB swap、整机卡死才"跑完"的——那不叫收敛，叫硬扛。
> 2. **实测反证。** 6 GB 上限下节点线性涨到 5.2 GB 被 kill，末段斜率是首段的 2 倍（详见 §4.12）。
>    真收敛的曲线会减速，不会越涨越快。

⚠️ **此处更正一个早先的错误判断。** 本文档此前说过「换一台大内存机器大概率也不会有改善」。
那个判断建立在「2.1 GB 是病态值」的假设上。**现有证据反过来指向它是正常开销——
所以那个回答是错的，换机器很可能确实有效。**

主机侧的硬约束（2026-09-15 实测）：

```
内存： total 5.7Gi   available 2.6Gi
交换： total 2.0Gi   used 2.0Gi   ← swap 已 100% 占满，实质没有余量
```

**要判定「换机器到底有没有用」，只需一个实验**：在有余量的机器上把容器上限放到 16 GB，
跑一次看爬升是否收敛。

- **收敛** ⟹ 开销有界，换机器直接解决。
- **不收敛** ⟹ 开销无界，换机器也救不了。

结论 4 已经把答案压到大概率的「有界」。但这条实验**仍然值得跑**——它给出的是
具体数值（峰值到底几 GB），而换机时要靠这个数去定 `mem_limit`。

---

### 4.8 尚未确认的疑点：刀路姿态

用姿态矩阵第三列（刀具轴）衡量每条路径的朝向一致性（`scripts/orient.py`，
标准差 0 表示整条路径刀具朝向完全稳定）：

| 路径 | 刀具轴均值 | 标准差 |
|---|---|---|
| 0 | (+0.006, +0.000, +0.053) | 0.9986 |
| 1 | (+0.020, +0.726, +0.341) | 0.5969 |
| **2** | **(-0.000, +1.000, -0.001)** | **0.0000** |
| 3 | (+0.018, +0.617, +0.337) | 0.7112 |
| 4 | (+0.021, +0.689, +0.351) | 0.6334 |
| 5 | (-0.024, +0.960, -0.074) | 0.2673 |
| 6 | (+0.007, +0.000, +0.051) | 0.9987 |

路径 2 完全一致，路径 0/6 几乎全乱——**同一条刀路内部差这么多，很可疑**。

**但这条不能下结论**：拿未经任何改动的原厂球（`part_scan.ply`）做对照，
同样每条路径标准差 ≈ 0.99。所以也可能只是对 `sand_tcp` 轴向约定的理解有误。

切开这个变量的实验已经写好但**未执行**（`scripts/oneplan.py`：只把姿态一致的
单条路径送去规划——能成功则姿态是元凶，照样炸则与姿态无关）。

### 4.9 诊断手法的坑（供参考）

容器里**没有** `gdb` / `eu-stack` / `pstack` / `perf`，抓不了栈。可用的手段只有：
cgroup 内存采样 + `ps` 逐进程 RSS + 日志字符串定位。

另外有一个**我犯过的错**值得记下来：用 `docker exec ... | tail -30` 看输出时，
前面的心跳行会被截掉，导致我一度把「节点已经被杀之后的回落期」当成「内存平稳」，
**误报了一次「OOM 没复现」**。看长输出不要用 `tail` 截断。

### 4.10 `remove_scan_link` unreachable 是同一个 OOM 的下游症状

界面上点 `[ Remove Scan Link ]` 报

```
[ Remove Scan Link ]  ->  FAILED
Service 'remove_scan_link' is unreachable
```

**这不是另一个 bug，是同一次节点死亡的第二现场。** 三个服务
`remove_scan_link` / `add_scan_link` / `generate_motion_plan` 都由**同一个进程**
`snp_motion_planning_node` 提供（可在容器内 `grep -l remove_scan_link` 于该二进制确认）。
节点被 OOM SIGKILL 之后，这三个服务一起消失。

`docker logs` 的最后几行把因果顺序钉死了：

```
[snp_motion_planning_node-11]: process has died [pid 102, exit code -9, ...]
[rviz2-8] [INFO]  Node [Remove Scan Link] created service client [remove_scan_link]
[rviz2-8] [ERROR] Remove Scan Link: Service with name 'remove_scan_link' is not reachable.
[rviz2-8] [INFO]  Node [Add Scan Link] created service client [add_scan_link]
[rviz2-8] [ERROR] Add Scan Link: Service with name 'add_scan_link' is not reachable.
```

**先死，后报不可达。** 时间戳上两者相隔不到 1 秒（`2026-09-15T09:39:17` 前后）。

> ⚠️ **仿真仓库自带的 `docs/TROUBLESHOOTING_CN.md` 在这一点上会误导人。** 它把这个现象
> 归因为「服务节点尚未就绪、点击太快」，建议 `./scripts/restart_demo.sh` 后重来。
> 在**原厂状态**下这么处理是对的（节点没死，只是没起来）。但在本项目的状态里，
> 重启只让节点**复活一次**——下一次 `/generate_motion_plan` 又会把它打死，
> `remove_scan_link` 于是再次不可达。**重启治标不治本，别在这上面反复耗时间。**

自查当前节点是死是活（比 `ros2 service list` 可靠，后者有发现缓存会残留过期条目）：

```bash
docker exec snp_automate_2023_sim bash -lc \
  'for d in /proc/[0-9]*; do c=$(cat $d/comm 2>/dev/null); \
   [ "$c" = "snp_motion_plan" ] && echo "存活 $(basename $d)"; done'
```

输出为空 = 已死。此时 RViz 里任何要调这三个服务的按钮都会 FAILED，属预期。

### 4.11 「为什么以前能跑，现在不能」——答案是我加的 3 GiB 上限

这是最容易被误解的一点，单独立一节。

| | 容器内存上限 | 结果 |
|---|---|---|
| **2026-08-19 及以前**（原厂 compose） | **无**（`mem_limit` 根本没写） | 规划**跑得完**；代价是主机 2 GB swap 被吃光、整机卡死 |
| **本项目加固后（旧版）** | `mem_limit: 3g` + `memswap_limit: 3g`（禁 swap） | 同一份内存需求 → 撞顶 → **SIGKILL**，必然死 |
| **本项目（当前版）** | `${SNP_MEM_LIMIT:-0}`，**默认不限**，与原厂一致 | 大内存机器上恢复成「能跑完」 |

六次实验的容器峰值全部精确压在 3 GiB 天花板上：

| 实验 | 容器峰值 |
|---|---|
| memwatch-7paths-294pts | **3072 MiB** |
| memwatch-1path-39pts | **3072 MiB** |
| tc_threads1 | 3035 MiB |
| ab_stool（换回原厂凳子） | 3028 MiB |
| lvslaunch | 2954 MiB |
| cfg_comesh | 2951 MiB |

3072 MiB = 3 GiB。**没有一次是「用不掉」，全都是「撞墙」**。

> **这是我引入的回归，必须写清楚。** §5 的加固把「整机僵死」换成了「容器内单进程被杀」，
> 诊断上确实更可控（否则连日志都拿不到）。但它同时把**「能跑完但拖死机器」变成了
> 「必然崩溃」**。这不是仿真本身变坏了——仿真一直需要那么多内存。

**已处理**：`docker/compose.sim.yml` 里的上限改为 `${SNP_MEM_LIMIT:-0}`，
即**默认不限制**（= 原厂行为）。大内存机器上什么都不用设就能跑。

> ⚠️ **但在这台 5.7 GB 的机器上，请显式设一个小上限再跑**，否则会重演整机卡死：
>
> ```bash
> export SNP_MEM_LIMIT=3g && docker compose -f docker/compose.sim.yml up -d
> ```
>
> swap 已经 100% 占满、可用内存只剩 2.6 GB，这里**没有余量去验证「不设上限能否跑完」**。
> 那个验证要留到明天的机器上做。

---

### 4.12 更正 2026-09-16 —— 6 GB 上限实测推翻了有界结论

把上限从 3 GB 提到 6 GB，重跑同一份 `converge.sh` 实验（7 条路径 / 294 点），拿到的是
一条**一路线性、始终不减速**的曲线：

| 采样段 | 斜率 |
|---|---|
| 前 1/3（节点还是空转） | ~0 |
| 中 1/3 | **+116 MiB / 0.2 s** |
| 后 1/3 | **+257 MiB / 0.2 s** ← 不但没坍缩，反而是首段的 2 倍多 |

节点 RSS 完整轨迹（MB，42 个采样点 / 8.4 秒）：

```
128 128 128 ... 128 129 179 527 852 901 1178 1419 1623 1627 1636 1894
2239 2541 2853 3167 3488 3798 4130 4200 4489 4668 4848 4979 5229 → 被 kill
```

相邻增量同样说明问题——全程在 +250～+350 MiB/采样 之间高位横盘，中途只有两次
短暂回落（+4、+9）就立刻反弹：

```
+50 +348 +325 +49 +278 +240 +205 +4 +9 +257 +346 +302 +312 +314
+321 +311 +332 +71 +289 +179 +180 +130 +250
```

**这是「无界增长」的形态，不是「有界一次性分配」。** 真正有界的分配会随着
工作集填充完毕而减速、趋于 0；这条曲线一直到撞墙那一刻都在全速爬。

容器侧证据：

```
OOMKilled    = true
memory.events: oom 1  oom_kill 1
memory.peak  = 6442450944   ← 正好 = 6 GiB 上限
退出码        = -9 (SIGKILL)
```

**两条曲线对照，能看出 2.3 GB 从来不是峰值：**

| 上限 | 节点 RSS | 爬到多快 | 结局 |
|---|---|---|---|
| 3 GB（§4.2 那次） | 130 M → **2.3 GB** | 6 秒 | 撞 3 GB 顶 |
| 6 GB（本次） | 128 M → **5.2 GB** | 8 秒 | 撞 6 GB 顶 |

上限翻倍，被截断的值也跟着翻倍，**斜率毫无变化**。所以：

> **§4.2 里那个「内存恒定在 ~2.1 GB、与输入无关」的结论要重新解读。**
> 它确实与输入无关——但原因不是「有界的固定开销」，而是
> **每次都在同一个墙前被截断**。墙的位置一动，读数就跟着动。
> §4.5 那张「全参数消融，增量纹丝不动 +1969～+2220 MiB」的表，
> 测的其实一直是「撞 3 GB 墙时爬到哪」，不是「峰值是多少」。

**对换机器意味着什么（这条最重要）：**

- 6 GB **不够**。换一台 8 GB 的机器同样会撞墙，只是撞得晚一点。
- 必须按「无界」来准备：上限给到 **32 GB 以上**，先跑一次看它到底会不会收敛。
  §9.3 的斜率判定脚本正是为这个设计的——**在那台机器上跑，才有意义**。
- 如果 32 GB 仍不收敛，就别再加内存了，走 §9.5 的方向：
  摘掉 `DiscreteContactCheckTask`（§4.5 已证明它与内存无关，说明它所在的子流水线
  另有问题），或改 `RasterMotionTask` 子流水线。

⚠️ **注意本机物理内存只有 7.5 GB、swap 2 GB 且已用 1.7 GB。**
6 GB 已经是这台机器能给出的极限（再高就会重演整机卡死）。
**「到底会不会收敛」这个问题，在本机已经无法回答，必须上服务器。**

---

## 5. 容器加固

`docker/compose.sim.yml` 的改动（加固前备份为 `docker/compose.sim.yml.bak-before-hardening`）：

```yaml
shm_size: 512m                      # 默认 64MB，RViz/DDS/Qt 都走 shm，加载网格和轨迹时会顶到上限
mem_limit: ${SNP_MEM_LIMIT:-0}      # 默认 0 = 不限制（= 原厂行为）
memswap_limit: ${SNP_MEM_LIMIT:-0}  # == mem_limit 即为禁用容器 swap
logging:
  driver: json-file
  options: { max-size: "10m", max-file: "3" }   # 节点会狂刷 cache hit，默认无上限会写满磁盘
```

**只有内存上限是可调的，其余两项无副作用、建议保留。**

**这台主机只有 5.8 GB 内存。** 不设上限时：规划一旦爆内存，内核会拿
**整个主机**去 swap，桌面连同所有窗口一起僵死——这就是「电脑直接卡死」的机制
（当时容器 `OOMKilled=false`、`RestartCount=0`，说明容器自己没被杀，是**主机被拖死了**）。

设上限之后，越界的是**容器内某个进程被 kill**（`docker start` 几秒就回来），
而不是整台机器失去响应。

> **但注意这两者的取舍**：设上限换来了整机安全，代价是规划**必然失败**（§4.11）。
> 内存紧张的机器上是「要么卡死、要么跑不成」，没有两全——**根治办法就是把内存加上去**。

设 `SNP_MEM_LIMIT=3g` 时已验证生效：

```
ShmSize=536870912 (512M)   Memory=3221225472 (3g)   MemSwap=3221225472
LogOpts={max-file:3 max-size:10m}   容器内 /dev/shm = 512M
memory.max = 3221225472   memory.swap.max = 0
```

---

## 6. 复现步骤

```bash
C=snp_automate_2023_sim
CFG=/opt/snp_automate_2023/install/snp_automate_2023/share/snp_automate_2023/config/tpp.yaml
MESH=/workspace/snp_home/snp/meshes/results_mesh.ply
```

### 6.1 看刀路（安全，0.1 秒，不会 OOM）

```bash
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; source /opt/snp/install/setup.bash; \
  export ROS_DOMAIN_ID=42; timeout 120 python3 -u /workspace/snp_home/tpptest.py $CFG $MESH"
```

### 6.2 复现 OOM（**会让节点死掉，之后需要 `docker restart`**）

> **前提：容器当前有内存上限**（本机默认 `SNP_MEM_LIMIT=3g`）。
> 在没有上限的机器上跑这个，节点不会死——取而代之的是**主机被拖进 swap**。
> 两条路都别在大内存机器上乱试。

```bash
docker exec $C bash -lc "source /opt/ros/jazzy/setup.bash; source /opt/snp/install/setup.bash; \
  export ROS_DOMAIN_ID=42; timeout 150 python3 -u /workspace/snp_home/prechk.py $CFG $MESH"
```

缩放对照（只取前 1 条路径）：

```bash
docker exec $C bash -lc "... python3 -u /workspace/snp_home/prechk.py $CFG $MESH 1"
```

一键复现 + 录内存曲线：`scripts/oomrepro.sh`（会先 `docker restart`，
**轮询**等节点就绪而不是硬等）。

### 6.3 恢复仿真

```bash
docker restart snp_automate_2023_sim
# 轮询等节点（每 2s 一次）
docker exec $C bash -lc 'for d in /proc/[0-9]*; do [ "$(cat $d/comm 2>/dev/null)" = "snp_motion_plan" ] && exit 0; done; exit 1'
```

> **判断节点死活有两个陷阱，方向正好相反：**
>
> 1. **假阳性**：`pgrep -f snp_motion_planning_node` 会匹配到自己的 bash 包装脚本。
>    必须用 `/proc/*/comm` 精确比对（`comm` 被截断到 15 字符，正好是 `snp_motion_plan`）。
> 2. **假阴性以外的另一种假象**：**`ros2 service list` 在节点死后会继续列出它的服务**——
>    那是 ROS 2 daemon 的发现缓存，不是真相。实测节点根本没起来，
>    `/generate_motion_plan` 却还在列表里，害得人以为服务正常。
>    **别用服务列表判断存活**，只看 `/proc`。

---

## 7. 文件清单

| 路径 | 说明 |
|---|---|
| `scripts/seatcut.py` | 按 z 阈值切出坐面 |
| `scripts/meshz.py` | z 分布分析，用来定阈值 |
| `scripts/meshcheck.py` | 验证 RViz 里推的是坐面（订阅 `/industrial_reconstruction_mesh`） |
| `scripts/tpptest.py` | 通用刀路测试：`tpptest.py <config.yaml> <mesh.ply>`，按路径打印点数/bbox/跳变 |
| `scripts/pathdump.py` | 把刀路导出成 JSON |
| `scripts/prechk.py` | 运动规划预检，带内存心跳；第 3 个参数可只取前 N 条路径做缩放对照 |
| `scripts/oneplan.py` | **只规划指定的若干条路径**：`oneplan.py <tpp> <mesh> <下标...>`，用来把「姿态」和「规划器」两个变量切开 |
| `scripts/orient.py` | 打印每条路径的刀具轴一致性与标准差（判断姿态是否稳定） |
| `scripts/oomrepro.sh` | OOM 复现 + 0.2s 粒度内存记录 |
| `scripts/mon.sh` | 0.2s 粒度内存监视器（写 `mon.pid`，**按 PID 停**，避免误杀节点） |
| `scripts/cfgtest.sh` | `cfgtest.sh '<sed表达式>' <标签>`：改 `planning_server.launch.xml` 的一个 arg 默认值后重启测量 |
| `scripts/abtest.sh` | `abtest.sh <网格路径> <标签>`：换 `results_mesh.ply`（碰撞环境用的网格）后重启测量 |
| `scripts/yamltest.sh` | `yamltest.sh '<sed表达式>' <标签>`：改 task composer 配置后重启测量 |
| `scripts/lvslaunch.sh` / `lvstest.sh` | `contact_check_lvs_distance` 的启动时/运行时改值对照 |
| `scripts/prof.py` + `profrun.sh` | 爬升期 0.15s 抓 `/proc/<pid>/maps`，判断是大块分配还是泄漏 |
| `docker/compose.sim.yml` | **加固后的 compose**（shm 512M、日志轮转、内存上限走 `${SNP_MEM_LIMIT:-0}` **默认不限制**）。§5 的全部改动都在这里 |
| `config/tpp.yaml` | 改好的配置（`FixedDirection`，ROISelection 已停用，含详细中文注释） |
| `config/tpp.yaml.bak-before-roiselection-removal` | 停用 ROISelection 之前的备份 |
| `config/tpp.yaml.bak-before-fixed-direction` | 换 FixedDirection 之前的备份 |
| `artifacts/seat_only.ply` | 坐面板网格（2256 顶点 / 4082 面，ASCII PLY） |
| `evidence/memwatch-7paths-294pts.log` | 7 条路径的逐进程内存曲线（0.2s 粒度） |
| `evidence/memwatch-1path-39pts.log` | 1 条路径的同上（缩放对照） |
| `evidence/ab_stool.log` | **原厂凳子**对照的内存曲线（+2168 MiB，证明与工件无关） |
| `evidence/tc_threads1.log` | Taskflow `threads: 1` 的内存曲线（+2176 MiB） |
| `evidence/cfg_comesh.log` | `collision_object_type: mesh` 的内存曲线（+2028 MiB） |
| `evidence/lvslaunch.log` | `contact_check_lvs_distance: 0.5` 启动时改值的曲线（+1969 MiB） |
| `evidence/prof.log` | 0.15s 粒度的 `/proc/<pid>/maps` 剖析（8 个大匿名块） |
| `evidence/event-order.txt` | 崩溃前的事件顺序原始摘录 |

---

## 8. 环境

| 项 | 值 |
|---|---|
| 容器 | `snp_automate_2023_sim` |
| ROS | ROS 2 Jazzy |
| `ROS_DOMAIN_ID` | 42 |
| 网络 | `network_mode: host` |
| 机器人 | Motoman HC10 |
| 仿真开关 | `SNP_SIM_ROBOT=true` `SNP_SIM_VISION=true` `SNP_BYPASS_EXECUTION=true` |
| 运动组 / TCP | `manipulator` / `sand_tcp` |
| mesh frame | `base_link` |
| 主机 | HP Zhan 66 Pro A G1 R MT，5.8 GB 内存 |
| 容器内调试器 | **无** `gdb` / `eu-stack` / `pstack` / `perf` |

---

## 9. 在另一台机器上跑（大内存）

> 2026-09-16 更新。目标：判定那 ~2.1 GB 是**有界**还是**无界**。
> ⚠️ 这个实验**在本机已经做不了**——6 GB 已是本机极限，而实测表明 6 GB 不够（§4.12）。
> **必须上服务器。**

### 9.1 为什么要换机

本机 7.5 GB 内存、swap 2 GB 已用 1.7 GB，**没有余量做这个判定实验**——
把容器上限调过 6 GB 就会把整机拖死（§5 记录的事故）。

> ⚠️ **注意：换机的理由在 2026-09-16 变了。**
>
> 旧版这里写的是「§4.7 已经回答了大内存行，换机只是去拿个数字」。
> **那个前提已经被推翻**（§4.12）：6 GB 上限下节点线性涨到 5.2 GB 才被 kill，
> 斜率全程不收敛。**现在"大内存到底行不行"本身就是一个未知数。**
>
> 所以这次上服务器要同时回答两个问题：
> 1. **会不会收敛？**（行不行）
> 2. **收敛在哪？**（峰值数字，用来定 `mem_limit`）
>
> **上服务器前的准备：**
> - 内存 **≥32 GB**。8 GB / 16 GB 机器会重演「撞墙」，只是墙高一点，得不到答案。
> - **把真机容器停掉**，或确认它不在 Domain 42（见 §10.2），否则服务名照样冲突。
> - 确认 `results_mesh.ply` 不是截断文件（见 §10.1）。

### 9.2 装起来

```bash
# 1) 依赖：docker + docker compose plugin，X11（RViz 要用）
xhost +local:

# 2) 仿真本体（原厂状态，不含本次改动）
git clone https://github.com/wjia051123-tech/snp-automate-2023-polishing-simulation.git
cd snp-automate-2023-polishing-simulation
docker pull ghcr.io/ros-industrial-consortium/snp_automate_2023:jazzy-master

# 3) 本项目（工件替换 + 加固 + 诊断脚本）
cd ~
git clone https://github.com/Zzh052500/snp-workpiece-swap.git

# 4) ★ 关键一步：把本项目的改动覆盖到仿真仓库上
#    （原厂仓库里没有 tpp.yaml 的修改、没有容器加固、没有坐面网格）
~/snp-workpiece-swap/scripts/install-into-sim.sh ~/snp-automate-2023-polishing-simulation

# 5) 起仿真
cd ~/snp-automate-2023-polishing-simulation
./scripts/restart_demo.sh
```

> 脚本里引用仿真仓库路径用 `SNP_SIM_DIR`，默认 `$HOME/snp-automate-2023-polishing-simulation`。
> 放在别处就先 `export SNP_SIM_DIR=<路径>`。
>
> **第 5 步不用设 `SNP_MEM_LIMIT`。** 大内存机器上默认就是「不限制」，
> 这正是 2026-08-19 那次能跑完的配置。设了反而会把规划掐死。

### 9.3 判定实验（一条命令）

```bash
export SNP_MEM_LIMIT=0                        # ★ 先做这步，见下方说明
~/snp-workpiece-swap/scripts/converge.sh 32g  # ★ 32g 起步，不是 16g
```

它会：`docker update` 把容器上限临时调到 32g（**可逆，不重建容器**）→ 重启 →
0.2s 粒度录内存 → 发起 7 条路径 / 294 点的规划 → **按爬升段的三段斜率判定**。

> ⚠️ **上限必须给足 32g，不要沿用旧的 16g。** 本机 6 GB 时节点在 8 秒内涨到
> 5.2 GB 才被截断，斜率毫无收敛迹象（§4.12）。给 16g 很可能照样撞墙，
> 白跑一轮重启（每轮 ~2 分钟）。**宁可给大，不要给小。**

| 判定输出 | 含义 | 下一步 |
|---|---|---|
| 末段斜率坍缩到首段 1/4 以下 | **有界** | ✅ 上限设成「实测峰值 ×1.5」固定下来，收工 |
| 斜率下降但未坍缩 | 有界但很大 | 再加倍上限重跑一次，直到坍缩 |
| 斜率基本没降 | **无界** | ❌ 加内存救不了。转 §9.5：摘掉 `DiscreteContactCheckTask` |

> ⚠️ **`export SNP_MEM_LIMIT=0` 必须在跑之前执行。** 脚本最后会把上限还原成
> `$SNP_MEM_LIMIT`，**不设则默认 3g**——在服务器上会把刚跑通的容器又掐死。
>
> **判「无界」时别急着下结论**：本机 6 GB 那次是「斜率不降」，但那是在
> **刚刚翻倍**（3g→6g）的情况下测的。上服务器第一轮就给它 32g，
> 如果斜率仍然不降，那才是真的无界。

### 9.4 如果判定成功（有界），怎么真的把坐面打磨出来

1. `mem_limit` **保持默认不限**（`${SNP_MEM_LIMIT:-0}` 不设即为 0）。
   如果想设死一个数，用 §9.3 测出的峰值的 1.5 倍，例如峰值 4 GB → `SNP_MEM_LIMIT=6g`
2. `./scripts/restart_demo.sh`
3. 走 §6.1 确认刀路仍正常
4. RViz 里跑完整流程（需要交互，脚本代替不了）
5. **注意**：`docker restart` 不会重置被 `docker cp` 改过的文件，但
   `docker compose up -d`（`restart_demo.sh` 用的就是它）**会重建容器**，
   把镜像里的原始文件恢复回来 —— 所以 `install-into-sim.sh` 之后不要再用
   `cfgtest.sh` / `abtest.sh` 那种 `docker cp` 手法去做**需要长期保留**的改动。

### 9.5 若 §9.3 判出「无界」，下一步的排查方向

按嫌疑从高到低：

1. **`DiscreteContactCheckTask`**（§4.5）。摘掉的办法：改
   `/opt/snp/install/snp_motion_planning/share/snp_motion_planning/config/task_composer_plugins.yaml`，
   把 `DiscreteContactCheckTask` 从 `SNPCartesianPipeline` / `SNPFreespacePipeline`
   的边里旁路掉（`TrajOptMotionPlannerTask → ConstantTCPSpeedTimeParameterizationTask`）。
   用 `scripts/cfgtest.sh` 的同款手法（`docker cp` + 重启）验证。
2. **刀路姿态**（§4.8）。执行已备好但未运行的 `scripts/oneplan.py`：
   ```bash
   # 只把姿态完全一致的那条路径（下标 2）送去规划
   oneplan.py <tpp> <mesh> 2
   ```
   能成功 ⟹ 姿态是元凶，回去修 `NormalsFromMeshFaces` / 网格法向一致性。
3. **`octree_resolution` 与 `max_convex_hulls`**：只在 `collision_object_type` 为
   `octree` / `convex_mesh` 时才起作用，本机已证 `mesh` 无效，但可以配合 1 一起试。

---

## 10. 2026-09-16 新增的两个故障模式（上服务器前必读）

§4 讲的是"内存不够"这一个根因。但 2026-09-16 的现场还踩到另外两个**与内存无关**的
故障，它们会在新机器上原样复现，所以单独立一节。

### 10.1 results_mesh.ply 被截断 导致 RPly 报错与节点段错误

**症状**（两条同时出现，很容易误判成两个问题）：

```
RPly: Error reading 'vertex_indices' of 'face' number 0
```
紧接着运动规划节点 `exit code -11`（SIGSEGV）。

**根因**：`runtime/snp_home/snp/meshes/results_mesh.ply` 是个**只有顶点、没有面片数据**
的截断文件（6605 字节）。`snp_motion_planning_node` 启动时要把它读进碰撞环境，
RPly 解析到面片段发现数据不完整 → 抛异常穿过 C 边界 → 段错误。

**这就是"为什么换机前它一直起不来"的原因之一**，和内存无关。

**修复**：

```bash
# 备份坏文件（不要直接覆盖）
mv runtime/snp_home/snp/meshes/results_mesh.ply \
   runtime/snp_home/snp/meshes/results_mesh.ply.bak-20260916-truncated
# 用坐面网格顶上
cp meshes/seat_only.ply runtime/snp_home/snp/meshes/results_mesh.ply
```

**判据**：正常文件是 **125494 字节 / 2256 顶点 / 4082 面**。
`meshes/seat_only.ply` 与 `results_mesh.ply` 现在**逐字节相同**（md5 `9af94bbe…`）。

> 这个文件是"扫描重建"的输出落点。走了完整流程后它会被真实重建结果覆盖，
> 在此之前用坐面网格占位即可——反正两者本来就该是同一个形状。

### 10.2 服务 unreachable —— 服务名被真机容器抢了

**症状**：

```
[ Start Reconstruction ] -> FAILED  Service 'start_reconstruction' is unreachable
[ Stop  Reconstruction ] -> FAILED  The TSDF is empty
[ Remove Scan Link ]     -> FAILED  Service 'remove_scan_link' is unreachable
```

**根因（重要，且曾经误判过）**：仿真容器和**真机容器**都跑在
`network_mode: host` 上，共享同一个 DDS 域。两边都提供同名服务
（`start_reconstruction`、`stop_reconstruction`、`remove_scan_link`……），
于是 BF（行为树）调到的可能是真机那个节点。

- 真机节点没在跑 ⟹ `unreachable`
- 真机节点在跑但没扫过东西 ⟹ TSDF 是空的 ⟹ `The TSDF is empty`

⚠️ **曾误判为 `ir_min_faces`（188 < 3000）导致的。已通过读源码排除**：
`reconstruction_sim_node` 的 `stop_cb` **根本不检查** `min_faces`，
`min_faces` 只影响真机的 `industrial_reconstruction` 节点。所以调那个参数没用。

> **`remove_scan_link` 的归属**：它由 **`snp_motion_planning_node` 自己提供**
> （定义见 `snp_application/config/snp.xml:264`，`EmptyService` 类型，无参数）。
> 所以它 `unreachable` 还有**第二种**成因：运动规划节点已经被 OOM 杀死了，
> 服务随之消失（见 §4.10）。**排查时先看节点死没死，再怀疑服务名冲突。**

**修复**：给仿真独立 ROS 域。`docker/compose.sim.yml` 已加：

```yaml
environment:
  ROS_DOMAIN_ID: "42"
```

**连带注意事项**：
- 容器**重建后**节点要 ~30 秒才注册好服务，这期间调服务同样是 `unreachable`。
  别把"没等够"当成故障。
- 容器内跑脚本时也要带上同一个域，否则连不上自己的节点：
  ```bash
  export ROS_DOMAIN_ID=42
  ```
- **真机容器必须停掉**，否则域隔离只解决一半问题（真机容器若也改了域才彻底）。

### 10.3 三个故障的鉴别顺序（照这个顺序查，别乱试）

现场三个症状长得很像，但根因完全不同。按这个顺序排：

| 顺序 | 先查什么 | 命令 | 若是它 |
|---|---|---|---|
| 1 | 运动规划**节点活着吗** | `docker exec $C bash -lc 'ps -eo pid,comm \| grep snp_motion'` | 死了 → 是 OOM（§4），`remove_scan_link` 等连带 unreachable |
| 2 | 服务**到底注册了没** | `ros2 service list \| grep -E 'scan\|motion_plan'` | 空 → 等 30s 再看；仍空 → 节点没起来 |
| 3 | 网格**能不能读** | `RPly` 报错 / 文件字节数 | 6605 字节 → 截断（§10.1），换 `seat_only.ply` |
| 4 | 容器 **OOM 计数** | `docker exec $C cat /sys/fs/cgroup/memory.events` | `oom_kill` > 0 → 回 §4 |
