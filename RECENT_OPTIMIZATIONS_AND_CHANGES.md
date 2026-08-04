# 最近 VDSuit / SkillCore 优化与改动记录

记录日期：2026-07-08  
涉及工程：

- `/home/cat/vdsuitfull-bvh-exporter`：独立 C++ VDSuit BVH exporter。
- `/home/cat/skill_core`：SkillCore 主程序，运行时直接通过 Python `ctypes` 加载 VDSuit SDK `.so`。

## 1. 关键结论

### 1.1 SkillCore 和 exporter 的关系

这轮排查后确认：

- SkillCore 当前并不是调用 `/home/cat/vdsuitfull-bvh-exporter` 这个独立 exporter 程序。
- SkillCore 自己在 `VDSuitMocapInterface` 的 worker 子进程里加载 VDSuit SDK `.so`，并通过 `ctypes` 直接调用 `Initial()`、`Connect()`、`DisConnect()`、`StartCalibrationFast()`、`GetCalibrationProgress()` 等 SDK 接口。
- 因此，“多次启动/关闭 SkillCore 后连接不上 VDSuit”的主问题应按 SkillCore 的 SDK 生命周期问题处理，不应把独立 exporter 当成主问题。

独立 exporter 也做过清理和重试增强，但这些改动只影响 standalone exporter，不会被 SkillCore 自动调用。

### 1.2 P-pose 误通过的根因

之前现实中没有摆 P-pose，但程序仍然读满进度条并通过，根因是：

- 原逻辑主要信任 VDSuit SDK 自己返回的 P-pose 标定进度和成功状态。
- SDK 的 P-pose 标定在测试中存在“没有严格确认真实 P-pose 也可能推进/成功”的情况。
- 程序侧没有在调用 SDK 标定前后做本地姿态校验。
- A-pose 在测试表现上更严格，所以“不摆 A-pose 不会成功”，但 P-pose 不能依赖 SDK 单独保证。

修复方向是增加本地 P-pose guard：先用实时动捕帧确认双手确实向前、接近水平，并稳定保持一段时间，再允许调用 SDK 标定；SDK 返回成功后再做一次最终姿态校验。

### 1.3 多次启动/关闭后连接不上的根因

SkillCore 侧确认到几个生命周期风险：

- `display_interface.py` 的 `_cleanup_interfaces()` 之前漏掉了 `_mocap_interface`，关机/重启路径不会显式关闭 VDSuit worker。
- SkillCore 的安全关机/重启最后会走 `os._exit(0)`，这会绕过部分 Python 对象析构和正常退出清理。
- VDSuit worker 内部原先更依赖 `connected == True` 才断开，如果 `Connect()` 失败、或者 device-break 回调提前把 `connected` 置为 false，可能跳过 SDK `DisConnect()`。
- 失败连接后没有先 `DisConnect()` 清理 SDK 内部状态再重试，反复启动/关闭后可能留下串口或 SDK 线程状态，导致下一次连接失败。

修复方向是：只要尝试过连接，就要允许 cleanup 调用 `DisConnect()`；worker 所有退出路径都统一 cleanup；SkillCore shutdown/reboot 必须把 mocap interface 纳入清理。

## 2. 独立 exporter 改动

文件：`/home/cat/vdsuitfull-bvh-exporter/src/main.cpp`

### 2.1 SDK 连接生命周期清理

新增/强化点：

- `SdkApi` 增加 `connectionTouched_` 状态。
- 每次实际调用 `Connect()` 前标记连接已触碰。
- `cleanupConnection()` 在连接被触碰后调用 SDK `DisConnect()`，即使 `Connect()` 没有成功，也会尝试释放 SDK/串口状态。
- cleanup 后等待约 `300ms`，给 SDK 内部线程和 USB/串口资源释放时间。
- `SdkApi` 析构时执行：
  - `cleanupConnection()`
  - `clearCallbacks()`
  - `dlclose(handle_)`

改动目的：

- 避免失败连接后 SDK 内部状态残留。
- 避免程序退出或动态库卸载时回调仍指向旧函数。
- 降低多次启动/关闭后串口资源无法重新打开的概率。

### 2.2 连接失败后的自动清理与重试

连接流程调整为：

1. 如果之前已经触碰过连接，先 cleanup。
2. 每次连接前重新调用 `Initial()`。
3. 调用 `Connect()` 后再检查 `GetConnectState()`。
4. 首次失败时：
   - 调用 `DisConnect()` 清理。
   - 等待约 `500ms`。
   - 再执行一次 `Initial()` + `Connect()`。
5. 最终失败时再次 cleanup，并输出 USB/权限/驱动检查提示。

行为影响：

- 正常成功连接几乎无额外影响。
- 失败路径会多一次 cleanup 和最多一次重试，失败耗时会略增加。
- 这是有意的，用少量延迟换取更可靠的 SDK 状态复位。

### 2.3 断开和退出路径增强

`disconnectDevice()` 现在会：

- 清除 `g_connected`。
- cancel 当前 exporter。
- 调用 `cleanupConnection()`。
- 重置最近一帧 mocap 快照。

新增信号处理：

- 捕获 `SIGINT`、`SIGTERM`、`SIGHUP`。
- 循环和标定等待过程会检查 shutdown 请求。
- 信号退出码按 `128 + signal` 返回。

改动目的：

- 用 Ctrl+C、systemd 停止、终端挂起等方式退出时，也尽量走 SDK cleanup。

### 2.4 P-pose 本地 guard

新增 P-pose 本地校验：

- 等待实时 mocap 帧。
- 确认帧是更新帧。
- 检查左右上臂、前臂传感器是否有有效数据。
- 根据左右肩计算身体朝向。
- 检查左右手相对肩部是否向前伸出、水平高度合理、平面距离足够。
- 连续稳定若干采样后才允许开始 SDK P-pose 标定。
- SDK 返回成功后，再执行一次最终姿态检查。

可通过环境变量临时跳过：

```bash
VDSUIT_SKIP_PPOSE_GUARD=1
```

这个环境变量只建议用于诊断或对比 SDK 原始行为，不建议作为正式使用配置。

### 2.5 StartCalibration 说明

独立 exporter 里同时尝试加载：

- `StartCalibration`
- `StartCalibrationFast`
- `CancelCalibration`
- `GetCalibrationProgress`

当前逻辑是：

- 如果 SDK 导出了 `StartCalibration`，优先使用它。
- 如果 `StartCalibration` 不存在，但 `StartCalibrationFast` 存在，则 fallback 到 fast calibration。

这部分不是菜单优化时改的；它属于 exporter 标定路径的兼容处理。

## 3. SkillCore 改动

### 3.1 VDSuit SDK wrapper 清理增强

文件：`/home/cat/skill_core/src/skill_core/vdsuit_sdk.py`

新增/强化点：

- 增加 `self._connection_touched`。
- `connect()` 调用前标记 `_connection_touched = True`。
- `disconnect()` 改为调用 `cleanup_connection()`。
- `cleanup_connection()` 只要连接曾经被触碰，就调用 SDK `DisConnect()`。
- cleanup 后默认等待约 `0.3s`。
- `__del__()` 兜底调用 cleanup，避免对象销毁时完全跳过 SDK 断开。

行为影响：

- 没调用过 `connect()` 时，cleanup 是 no-op。
- 只要调用过 `connect()`，即使连接失败，也会允许 `DisConnect()` 复位 SDK 状态。

### 3.2 VDSuit worker 连接流程修复

文件：`/home/cat/skill_core/src/skill_core/mocap_interface.py`

worker 内新增统一 cleanup：

- `cleanup_sdk_connection()` 不再只依赖 `connected == True`。
- 所有退出路径、异常路径、连接失败路径都尽量调用 cleanup。
- device break 后也会进入 cleanup。

连接流程调整：

- 如果本地认为已连接，会先用 `sdk.get_connect_state()` 复核。
- 每次连接尝试前都调用 `sdk.initial(WS_Geo)`。
- 首次 `connect()` 失败后：
  - cleanup SDK connection。
  - 重新检测 USB receiver。
  - 等待约 `0.5s`。
  - 再 retry 一次。
- retry 仍失败则 cleanup，并向主进程发送 `connect_failed`。

改动目的：

- 解决 SDK 内部状态不干净时，下一次连接直接失败的问题。
- 解决 device-break / Connect 失败导致 `connected` 状态不可信的问题。

### 3.3 Manual reconnect 与 worker restart

文件：`/home/cat/skill_core/src/skill_core/mocap_interface.py`

`manual_reconnect()` 现在区分几种情况：

- 如果仍有实时帧，认为已经连接，跳过重复重连。
- 如果已有重连请求正在进行，15 秒内忽略重复请求。
- 如果曾经成功连接过，但现在没有新帧，认为 worker/SDK 状态不可信，重启 worker。
- 如果 worker 已死亡或被标记需要 restart，重启 worker。
- 如果还没启动 worker，则启动 worker。
- 其他情况下只向现有 worker 发送 `("connect",)`。

新增 standby worker：

- 默认维护 3 个 standby worker。
- 数量由环境变量控制：

```bash
SKILLCORE_VDSUIT_STANDBY_WORKERS=3
```

- runtime reconnect 时优先 promote standby worker，减少重新 fork / 初始化的等待。

### 3.4 shutdown / reboot 清理补漏

文件：`/home/cat/skill_core/src/skill_core/display_interface.py`

`_cleanup_interfaces()` 的清理列表新增：

```python
("mocap", "_mocap_interface")
```

目的：

- SkillCore 关机/重启时显式关闭 VDSuit worker。
- 避免 `os._exit(0)` 前漏掉 mocap 清理。
- 避免 recorder、camera、tactile、timecode、bluetooth 中任意一个 cleanup 抛异常时影响其他接口清理。

### 3.5 SkillCore P-pose 本地 guard

文件：`/home/cat/skill_core/src/skill_core/mocap_interface.py`

新增/强化点：

- `_vdsuit_looks_like_ppose(frame)` 本地判断 P-pose 是否成立。
- 检查实时帧、更新状态、手臂传感器状态、肩部方向、双手前伸和水平度。
- `wait_for_local_ppose_ready()` 在 SDK P-pose 标定前执行。
- SDK 返回 P-pose success 后，`local_ppose_final_check()` 再验一次。
- 本地 guard 的进度会通过 `calibration_progress` 发回 UI。

可通过环境变量跳过：

```bash
VDSUIT_SKIP_PPOSE_GUARD=1
```

正式使用不建议跳过。

### 3.6 标定 UI 状态显示

文件：`/home/cat/skill_core/src/skill_core/display_interface.py`

新增 `draw_vdsuit_calibration_status()`：

- 标定进行中显示全屏遮罩。
- 显示当前标定类型：A-pose / P-pose。
- 显示进度条。
- P-pose guard 阶段显示“正在确认真实 P-pose...”。
- 对常见 guard 失败原因做中文提示：
  - 暂无实时动捕数据。
  - 动捕数据未更新。
  - 手臂传感器数据不可用。
  - 身体位置数据无效。
  - 肩部方向不可用。
  - 双臂需要向前伸直并保持水平。
- 标定成功或失败后保留结果约 3 秒。

### 3.7 可视化菜单优化

文件：`/home/cat/skill_core/src/skill_core/display_interface.py`

原问题：

- verbose 菜单把所有系统信息、录制、相机、触觉、OTA、关机、VDSuit A/P/R 操作都放在一页。
- 选项过多，实际操作时不够清晰。

新交互：

- `3/V` 不再只是布尔开关，而是三态循环：
  1. 关闭菜单。
  2. 详情页。
  3. VDSuit 操作页。
  4. 再按回到关闭菜单。

详情页保留：

- 当前软件版本。
- 屏幕刷新率。
- FPV 分辨率。
- 局域网 IP。
- `1` 开始/停止录制。
- `2` 切换可见光/深度图像。
- `3` 下一页。
- `4` 触觉显示开关。
- `5` 触觉显示时手套标定，否则拍摄图像。
- `6` 升级程序。
- `7` 转移数据。
- `8` 退出并关机。

VDSuit 操作页新增：

- VDSuit 当前状态。
- 标定接口是否可用。
- `1`：VDSuit A-pose 标定。
- `2`：VDSuit P-pose 标定。
- `4`：后台手动连接/重连 VDSuit。
- `3`：关闭菜单。

兼容性：

- `A/P/R` 字母快捷键仍然保留。
- 只有在 VDSuit 操作页打开时，数字 `1/2/4` 才被映射为 A/P/R。
- `S/D/F` 字母别名不变，仍然对应原来的录制、图像切换、触觉显示。
- 菜单页状态内部用 `_info_page` 表示，`verbose` 仍保留为兼容布尔状态。

行为风险：

- 在 VDSuit 操作页打开时，数字 `1/2/4` 的含义会临时变为 VDSuit 操作。
- 这是按当前交互方案有意设计的；屏幕上会明确显示该页用途。
- 关闭菜单或回到详情页后，数字键恢复原行为。

## 4. 测试与验证

### 4.1 SkillCore 已执行验证

语法检查：

```bash
cd /home/cat/skill_core
.venv/bin/python -c "import ast, pathlib; ast.parse(pathlib.Path('src/skill_core/display_interface.py').read_text(encoding='utf-8'))"
```

结果：通过。

VDSuit 回归测试：

```bash
cd /home/cat/skill_core
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest \
  tests/test_vdsuit_reconnect.py \
  tests/test_vdsuit_bvh.py \
  -o cache_dir=/tmp/skillcore-pytest-cache
```

结果：

```text
24 passed
```

覆盖重点：

- runtime stale frames 后 manual reconnect 会重启 worker。
- standby worker 可被 promote，不必总是重新创建 worker。
- 重复 R / manual reconnect 请求会被 guard，避免并发重连。
- reconnect 超时后允许再次发起。
- 初始连接失败时可向 worker 发送 connect。
- connected 消息会清除 reconnect guard。
- P-pose guard 能接受正确 P-pose，拒绝手未前伸和手臂传感器无数据。
- `VDSUIT_SKIP_PPOSE_GUARD` 环境变量行为正确。
- SDK connect 失败后 cleanup 会调用 `DisConnect()`。
- connect 前未触碰连接时 cleanup 是 no-op。
- shutdown 会 graceful stop worker 和 standby worker。
- A/P-pose 标定请求和失败路径有覆盖。

### 4.2 独立 exporter 验证状态

standalone exporter 此前已执行过 `./build.sh` 构建验证。

本次文档整理没有重新编译 exporter。

### 4.3 SkillCore 打包状态

SkillCore 打包没有由我完成。之前尝试：

```bash
cd /home/cat/skill_core
.venv/bin/python build.py skill-core-vdsuit
```

失败原因：

- `build.py` 内部调用 `pyinstaller`。
- 当时 shell 的 `PATH` 没包含 `.venv/bin`，所以找不到 `pyinstaller`。

建议打包命令：

```bash
cd /home/cat/skill_core
source .venv/bin/activate
python build.py skill-core-vdsuit
```

或：

```bash
cd /home/cat/skill_core
PATH="$PWD/.venv/bin:$PATH" .venv/bin/python build.py skill-core-vdsuit
```

## 5. 需要真机确认的项目

### 5.1 连接生命周期

建议流程：

1. 启动 SkillCore。
2. 确认 VDSuit realtime。
3. 关闭程序。
4. 再启动程序。
5. 重复 5 到 10 次。
6. 每次确认 VDSuit 能稳定连接。
7. 中途可拔插 VDSuit receiver，再按 `R` 或在 VDSuit 菜单页按 `4` 测试重连。

如果仍然出现连接失败，下一步优先查：

- `/dev/vdsuit`
- `/dev/ttyXRUSB*`
- `lsusb` 是否能看到 VDSuit receiver，例如 `04e2:1410`
- udev 规则是否生效。
- 是否有残留 SkillCore / VDSuit worker 进程。
- USB serial driver 是否需要 rebind。

### 5.2 P-pose 防误通过

建议流程：

1. 进入 VDSuit 菜单页：按 `3` 到详情页，再按 `3` 到 VDSuit 页。
2. 不摆 P-pose，按 `2`。
3. 应该卡在“正在确认真实 P-pose”或提示具体等待原因，不应进入 SDK 成功。
4. 正确摆 P-pose 并保持，进度应推进。
5. 标定成功前松开或姿态明显错误，应被最终 guard 拦截或失败。

### 5.3 菜单交互

建议确认：

- 菜单关闭时：
  - `1/2/4` 仍是录制、图像切换、触觉显示。
- 详情页时：
  - `1/2/4` 仍是原行为。
  - `3` 进入 VDSuit 页。
- VDSuit 页时：
  - `1` 触发 A-pose。
  - `2` 触发 P-pose。
  - `4` 触发手动连接/重连。
  - `3` 关闭菜单。
- `A/P/R` 字母快捷键始终可用。

## 6. 当前改动的影响范围

### 6.1 正向影响

- VDSuit 失败连接后会主动清 SDK 状态。
- 多次启动/关闭 SkillCore 后，VDSuit worker 更有机会被正常 shutdown。
- runtime USB 掉线或无帧时，manual reconnect 不再只向可能卡死的 worker 发 connect，而是会重启 worker。
- P-pose 标定不再完全信任 SDK 成功状态。
- UI 菜单选项分层，常用系统操作和 VDSuit 操作分开。
- 旧的 `A/P/R` 快捷键未移除。

### 6.2 代价和注意事项

- 连接失败路径会增加约 `0.3s` 到 `0.5s` 的 cleanup / retry 等待。
- P-pose 标定需要真实保持姿态，不能像之前那样直接读满进度条。
- VDSuit 页打开时，数字 `1/2/4` 临时变成 VDSuit 操作。
- standby worker 会多占用少量进程资源，默认 3 个；可用 `SKILLCORE_VDSUIT_STANDBY_WORKERS` 调整。

## 7. 主要相关文件

SkillCore：

- `/home/cat/skill_core/src/skill_core/display_interface.py`
- `/home/cat/skill_core/src/skill_core/mocap_interface.py`
- `/home/cat/skill_core/src/skill_core/vdsuit_sdk.py`
- `/home/cat/skill_core/src/skill_core/vdsuit_bvh.py`
- `/home/cat/skill_core/tests/test_vdsuit_reconnect.py`
- `/home/cat/skill_core/tests/test_vdsuit_bvh.py`

独立 exporter：

- `/home/cat/vdsuitfull-bvh-exporter/src/main.cpp`
- `/home/cat/vdsuitfull-bvh-exporter/src/bvh_exporter.cpp`
- `/home/cat/vdsuitfull-bvh-exporter/src/bvh_exporter.h`
- `/home/cat/vdsuitfull-bvh-exporter/include/VDMocapSDK_VDSuitMini_DataType.h`

## 8. 当前状态摘要

- SkillCore VDSuit 连接生命周期：已修复并有单元测试覆盖。
- SkillCore P-pose 防误通过：已加本地 guard，并有单元测试覆盖。
- SkillCore VDSuit 标定 UI：已显示进度、中文状态和结果。
- SkillCore 菜单优化：已完成三态菜单和 VDSuit 二级页。
- SkillCore 打包：未完成，由用户自行打包。
- 独立 exporter：已做 cleanup/retry/P-pose guard 增强，不影响 SkillCore 主程序。
