# Windows 原生部署与采集

本文用于把 PICO + MANUS + 双手触觉 + VST 采集端部署在一台 Windows 10/11 x64 电脑上。
运行期不依赖 WSL；服务、串口、网络接收和离线管线都在 Windows 原生 Python 中执行。

## 1. 当前平台边界

- PICO tracking/VST 接收端是 Python socket 程序，可在 Windows 原生运行。
- 触觉使用 CH340 COM 口和 pyserial，已支持 Windows。
- 对齐、HDF5 导出和视频验收可在 Windows 运行。
- MANUS 必须使用厂商发布的 **Windows C++ SDK** 中的 `ManusSDK.dll` 和导入库。
  仓库原有 `libManusSDK_Integrated.so` 是 Linux ELF，不能在 Windows 加载或转换成 DLL。
- MANUS Integrated SDK 还要求具有 SDK Integrated 功能的 MANUS license/dongle。

## 2. 一次性安装

以普通 PowerShell 打开项目目录：

```powershell
cd D:\pico_controller_withTac\pico_controller_withTac

# Python 科学计算、HDF5、OpenCV、触觉与可视化依赖；同时安装 FFmpeg。
powershell -ExecutionPolicy Bypass -File .\scripts\setup_windows.ps1 -InstallFFmpeg
```

为 PICO 开放私有网络入站端口时，在 PowerShell 运行以下脚本；它会自动弹出 UAC
管理员确认，并把执行结果写入 `.run/pico_firewall_setup.log`：

```powershell
cd D:\pico_controller_withTac\pico_controller_withTac
powershell -ExecutionPolicy Bypass -File .\scripts\configure_pico_firewall.ps1
```

开放范围仅为 Private profile 且来源限制为 `LocalSubnet`：TCP 63901（tracking）、
TCP 63902（VST）和 UDP 29888（发现）。
控制端口 63910/63911/63912 只监听 `127.0.0.1`，不需要防火墙规则。

## 3. 准备 MANUS Windows SDK

1. 登录 MANUS Download Center，下载包含 `SDKMinimalClient_Windows` 或
   `SDKClient_Windows` 的 C++ SDK 压缩包。
2. 解压到：

```text
vendor/manus_sdk_windows/
```

3. 安装 Visual Studio 2022 Build Tools，勾选“使用 C++ 的桌面开发”。
4. 在普通 PowerShell 中进入项目后运行（脚本会自动加载 VS 2022 x64 编译环境）：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\setup_manus_bridge.ps1
```

脚本会递归查找以下资产并编译：

```text
ManusSDK.h
ManusSDKTypes.h
ManusSDKTypeInitializers.h
ManusSDK.lib（或 ManusSDK_Integrated.lib）
ManusSDK.dll（或 ManusSDK_Integrated.dll）
```

产物是：

```text
manus_ndjson_bridge/manus_ndjson_bridge.exe
manus_ndjson_bridge/ManusSDK.dll
```

若 SDK 解压在其它目录：

```powershell
$env:MANUS_SDK_WINDOWS = "D:\path\to\MANUS-SDK"
powershell -ExecutionPolicy Bypass -File .\scripts\setup_manus_bridge.ps1
```

## 4. 部署自检

```powershell
powershell -ExecutionPolicy Bypass -File .\ego_ctl.ps1 doctor
```

完全通过应满足：

- Python 模块导入成功；
- `ffmpeg`、`ffprobe` 可执行；
- 触觉配置有效；连接硬件时列出两个 `1A86:7523` 串口；
- `manus_ndjson_bridge.exe` 与 MANUS DLL 存在；
- PICO 防火墙端口已按需要开放。

## 5. 启动与采集

连接 PICO、MANUS dongle/手套、两只触觉指套后：

### 数采员推荐：一键入口

正式采集时请让数采员只阅读项目根目录的 [`采集员操作说明.md`](../采集员操作说明.md)。
本文其余内容供部署和技术排障使用。

直接双击项目根目录中的：

```text
COLLECT_DATA.cmd
```

也可以在项目目录直接指定前缀启动：

```powershell
.\COLLECT_DATA.cmd S01_pick
```

数采员只操作这一个主窗口。脚本会依次完成：

1. 只输入一次本批任务前缀，例如 `S01_pick`；
2. 检查 Windows 常驻的 PICO 发现广播和接收服务，并启动 MANUS 与触觉服务；
3. 自动打开触觉配对窗口，按窗口提示完成松手基线和左手按压；
4. 触觉配对完成后，主窗口进入待机；按 Enter 后等待 Head、双手柄、双 MANUS 手套、双触觉和 VST 全部就绪；所有数据路必须连续稳定2秒才开始录制；
5. 录制中按 Enter 停止本条；脚本自动封存、质量检查、时间对齐并导出 HDF5；
6. 导出后再次按 Enter 开始下一条，名称自动递增为 `<前缀>_001`、`<前缀>_002`……；
7. 录制中按 `H` 会安全停止并重新采当前编号；待机时按 `H` 会撤销刚完成的上一条。两种误采内容都保存在 `data/rejected/`，不会混入正式数据；
8. 录制中按 `Q` 会停止当前条、导出 HDF5、结束本批并关闭 MANUS/触觉服务；待机时按 `Q` 则直接结束本批。PICO 发现与接收连接保持常驻。

#### 传感器防呆状态机

开始前和采集中采用不同处理，避免“缺一路仍然录”和“瞬时抖动误杀”：

| 阶段 | 检测到一路缺失时 | 后续行为 |
|---|---|---|
| `[待机]` | 尚未创建任何本条文件 | 等数采员按 Enter |
| `[wait]/[stabilize]` 开录前 | 保持等待，不发送任何一路 `START`；恢复后必须连续稳定2秒 | 最长等待默认300秒；超时仍不占编号，Enter可再次等待，Q结束 |
| `[sensor-check]` 录制中 | 先进入1.5秒连续异常确认，短暂状态抖动恢复后继续 | 每0.5秒同时检查PICO头/双手柄/VST、双MANUS、双触觉和写盘错误 |
| 已确认传感器故障 | 立即向全部服务并发发送 `STOP`，本条禁止导出 | 原始尝试连同 `capture.failure.json` 移入 `data/rejected/`；修复后按Enter重采同一编号 |

运行中判断包含：服务/会话状态、PICO姿态和VST新鲜度、Head与双手柄、双MANUS在线及
数据年龄、双触觉原始流、订阅缺口、采集器故障和异步写盘错误。即使短暂异常未达到
1.5秒，离线导出仍执行30 ms逐帧门限和完整帧覆盖率门禁，不会把缺数据行写进正式HDF5。

同一前缀的数据按批次目录保存。例如前缀为 `expert`：

```text
data/sessions/expert/001/
  raw/                               # PICO、MANUS、VST、双手触觉原始数据
  aligned.jsonl                      # 对齐数据
  dataset.hdf5                       # 最终训练包
  manifest.json                      # 本条清单及标定依据
  review/                            # 人工复核资产
```

其中新采集的 `data/sessions/<前缀>/<序号>/raw/` 会同时包含 `vst.ts.jsonl`（墙钟）和
`vst.qpc.ts.jsonl`（高精度 QPC）。PICO、MANUS、触觉原始 JSONL 也都写入
`recv_qpc_ns`。离线导出遵循以下规则：

- 整条数据只使用一个主机时钟域；新数据优先 QPC，旧数据缺字段时整条统一回退墙钟；
- 先取 PICO、双 MANUS、VST、双触觉的公共有效时间段，再生成 30 Hz 训练时间轴；
- HDF5 为 MANUS、视频、触觉保留源帧双时间戳与 `offset_ms`，可逐行复查；
- MANUS、VST 视频和触觉的最近邻门限统一为 30 ms；超过门限的目标帧先标记为无效；
- 默认强制要求各路覆盖率至少 99%、`|offset|` 的 p95 不超过 20 ms、最大值不超过
  30 ms；不合格时本条原始数据仍完整保留，但 HDF5 导出失败并要求重采或检查链路。
- 还要求“全部数据路同时有效”的完整帧覆盖率至少 99%；通过后只把 PICO头/双手柄、
  双MANUS、VST和双触觉均有效的行写入HDF5，剔除行两侧自动切成不同 `segment_id`；
  所以正式HDF5中的每一帧都有全部传感器数据，不完整内容仅留在原始文件中。
- HDF5 同时保存校准前 `left/right_controller_pose`、校准后 `left/right_wrist_pose`，
  以及数值化 `controller_to_wrist_calibration`，可独立进行手腕位置/深度修正。
- p95 是时间偏差的第 95 百分位：95% 的有效匹配帧偏差不超过该值，并不是覆盖率。

旧版 `data/raw`、`data/tactile_raw`、`data/aligned`、`data/export` 等类型优先
目录继续兼容读取；新采集使用任务优先布局，不会自动移动或覆盖旧数据。

### 删除批次中的若干条

双击项目根目录的：

```text
DELETE_BATCH_DATA.cmd
```

依次输入批次前缀和要删除的编号，例如前缀 `expert`、编号 `2,5,7-9`。脚本会先
显示预览并要求确认，然后同时处理原始数据、触觉、aligned、HDF5 和复核资产，
其余数据保持原编号，允许出现编号空缺；后续采集继续使用“现有最大编号 + 1”，
不会填补删除造成的空缺。被删除的内容不会立即物理销毁，而是保存在
`data/deleted/<前缀>/<时间戳>/`，其中 `batch_edit.json` 记录删除编号和归档来源，
便于误操作后恢复。运行删除工具前必须先结束采集程序。

触觉配对窗口由脚本自动打开，采集期间必须保持开启。除该自动窗口外，不需要数采员
手工打开其它终端。前缀建议只使用字母、数字、中文、下划线和连字符，例如
`S01_pick`，不要自行填写末尾序号，也不要使用空格。如果磁盘中已经存在
`S01_pick_001` 和 `S01_pick_002`，脚本会从 `S01_pick_003` 继续，绝不覆盖旧数据。

### 维护人员：分步命令

```powershell
# 确保常驻 PICO 发现/接收在线，再启动 MANUS/触觉；触觉会打开可见窗口供人工配对。
powershell -ExecutionPolicy Bypass -File .\ego_ctl.ps1 start

# 状态
powershell -ExecutionPolicy Bypass -File .\ego_ctl.ps1 status

# 正式采集；在本终端按 Enter 停止并封存
python pico_record.py start 任务名

# 离线对齐、质量检查和 HDF5 导出
powershell -ExecutionPolicy Bypass -File .\ego_ctl.ps1 pipeline 任务名

# 当天结束：停止落盘、MANUS 和触觉；PICO 连接层保持常驻
powershell -ExecutionPolicy Bypass -File .\ego_ctl.ps1 stop
```

触觉配对窗口在常驻期间必须保持开启。触觉 USB 重插、换口或服务重启后，应运行
`ego_ctl.ps1 restart` 重新做静止基线和左手按压识别。

## 6. PICO 接入检查

- Windows 网络类型应设为“专用网络”。
- 头显和电脑必须在同一局域网，且 AP 不启用客户端隔离。
- PICO App 开启 Send + Head + Controller。
- 部署脚本会安装名为 `PICOEgoDiscovery` 的 Windows 当前用户登录自启动项，并常驻
  两个进程：地址发现广播和 PICO tracking/VST 接收服务。它们不属于单条或单批
  采集进程，`ego_ctl.ps1 stop` 和退出一键采集都不会关闭它们。
- 发现进程默认每 1 秒广播一次，并每 5 秒重新枚举 Windows 网卡地址；PICO 接收
  服务始终监听 tracking/VST，空闲时不写入会话数据。头显开机、休眠恢复、USB
  网络重连或网卡地址变化后无需重启采集脚本。
- `ego_ctl.ps1 status` 的 `PICO-DISCOVERY` 行应为 `pid=...`，且 `ips=` 不应为
  `(none)`；`sent=` 应持续增加，`errors=` 应保持为 0 或不持续增长。
- `PICO-LINK` 行应为 `pid=... OK idle/REC`；头显连接后 `devices=(none)` 会变为设备 SN。
- `ego_ctl.ps1 status` 的 PICO 行应从 `devices=(none)` 变为设备 SN。
- VST 实际帧率仍以 `data/sessions/<任务>/<序号>/raw/vst.ts.jsonl` 为准。

发现广播只负责让 PICO 找到电脑，常驻 PICO-LINK 才负责保持实际 TCP 接收连接。
网线/USB 物理断开、PICO App 退出、Windows 网卡被禁用或 AP 开启客户端隔离时，
常驻进程不能替代这些链路条件；链路恢复后会继续广播并接受 PICO 重连。

单独维护 PICO 常驻连接层（通常不需要数采员执行）：

```powershell
# 安装/更新 Windows 登录自启动项，并立即启动发现广播与接收服务
powershell -ExecutionPolicy Bypass -File .\scripts\pico_discovery_autostart.ps1 install

# 查看常驻连接层
powershell -ExecutionPolicy Bypass -File .\scripts\pico_discovery_autostart.ps1 status
```

## 7. 日志与故障定位

```text
.run/pico.windows.out.log
.run/pico.windows.err.log
.run/pico_discovery.autostart.log
.run/pico_discovery.status.json
.run/manus.windows.out.log
.run/manus.windows.err.log
.run/tactile_service.windows.log
```

常见情况：

| 现象 | 检查 |
|---|---|
| doctor 缺 NumPy/OpenCV | 重新运行 `setup_windows.ps1` |
| 找不到 ffmpeg | 安装后新开 PowerShell，再运行 doctor |
| MANUS bridge 缺失 | 放入 Windows SDK，并从 VS x64 工具命令行编译 |
| MANUS DLL load failed | DLL 放在 bridge `.exe` 同目录；确认 SDK/bridge 都是 x64 |
| PICO 看不到电脑 | 专用网络、防火墙三条规则、同网段、关闭 AP 隔离 |
| 触觉识别不足 2 个串口 | 重新插紧两只触觉 USB；在设备管理器确认两只 CH340/COM 均出现后再启动 |
| 触觉配对信号弱 | 基线时完全松开；倒计时结束后持续按左手多个感应区 |
| 触觉出现单次 timeout/parser_discarded | 记为可恢复告警并继续；`status` 查看 `warnings`，封存后查看 `tactile.meta.json` |
| 触觉进入 FAULT_LATCHED | 默认同侧5秒内累计3次瞬时告警，或发生 CRC/协议/I/O/真实帧缺口；检查USB后 `ego_ctl.ps1 restart` 重新配对 |

触觉运行期默认容错策略：单次响应超时或解析器重同步丢弃字节不会立即锁存，
而是记录 `warnings/warning_l/warning_r`。同一侧在5秒滚动窗口内累计3次才升级为
`FAULT_LATCHED`；CRC、长度/尾标、功能码、I/O、writer错误和订阅帧缺口仍立即
fail-closed。正式会话内的可恢复告警会写入
`tactile.meta.json -> summary.transient_health_warnings`，并在HDF5属性
`tactile_quality_status` 与 `tactile_transient_warning_count` 中保留。

如需临时调整阈值，应在启动服务前设置当前 PowerShell 环境变量：

```powershell
$env:TACTILE_TRANSIENT_LIMIT = "3"
$env:TACTILE_TRANSIENT_WINDOW_S = "5"
.\ego_ctl.ps1 restart
```

## 8. 官方 MANUS 说明

MANUS 官方说明 Windows SDK 包含 Windows Minimal Client 工程，并通过 Visual Studio
构建；Integrated 模式在 Windows 上直接连接手套，但需要相应 SDK license。下载和
许可证属于厂商分发内容，本仓库不会复制或替代这些二进制资产。
