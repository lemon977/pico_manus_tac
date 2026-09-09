# 可选：双目 PnP（叠图增强）

> **本文是可选增强，不改动已完结的主流程。**  
> 主标定仍以 [`CALIB_QA_ZH.md`](CALIB_QA_ZH.md) 为准（左眼 PnP + `T_calib` + 导出）。  
> **训练包仍然只有一套世界系 pose**；本文不产生第二套腕位姿。

---

## 0. PnP 是什么

**PnP = Perspective-n-Point（透视 n 点）**。

已知：

- 若干 **3D 点**（这里：手柄原点在「头局部系」里的坐标，来自 Tracking）
- 它们在 **某一只眼图像** 上的 **2D 像素**（你点出来的）
- 该眼 **内参 K**（针孔）

求解：

- 头系 → 该眼相机系的固定安装 **`[R|t]`**

一句话：用「手柄在 3D 哪儿 + 在画面哪儿」算出「相机相对头怎么装的」。

| | |
|--|--|
| 输入 | 多帧：3D 手柄点 ↔ 像素点击 |
| 输出 | `head_to_cam_solved_*.json` 里的 `R,t` |
| **进不进训练 pose** | **不进**。只给 `overlay_skeleton` 叠图用 |
| 和 `T_calib` | 无关。腕偏移是另一条 3D 链 |

主流程里的会话名 `pnp1` 就是「采一段给左眼做这次 PnP」的意思。

---

## 1. 为什么要单独写「双目」版

主流程只解 **左眼** `head_to_cam_solved.json`，叠左眼验收够用，**训练不依赖右眼 PnP**。

若你还想：右眼叠图也像素级贴手柄，可以 **额外** 做右眼一份 PnP。  
这是验收/叠图增强，**不是**让模型吃两套 pose。

**与主方案同一条硬规则**（见 [`CALIB_QA_ZH.md`](CALIB_QA_ZH.md) §1）：

- 只用左眼拧 `T_calib.pos` → 右眼常不贴，且 **会污染训练**（唯一一套 `*_wrist_pose`）→ **禁止**  
- `pos` 只许尺量 / 3D；叠图（含右眼）不贴 → 查 PnP，**禁止**再拧 `pos`

```text
模型吃：一套 3D pose + 左右图像 + vst_cam 基线
叠图用：左眼 [R|t] 和/或 右眼 [R|t]（可选）
```

---

## 2. 做法（在主流程之外加做）

前提：已按 `CALIB_QA_ZH.md` 采过 `pnp1` 并对齐；有显示器做 annotate（181 无屏可把数据拉到本机点）。

### 2.1 左眼（与主流程相同；若已有可跳过）

```bash
cd ~/pico_controller

python3 calibrate_headcam.py annotate \
  data/aligned/pnp1.jsonl data/raw/pnp1/vst.h264 \
  --eye left --n 12 \
  -o config/pico_cam/clicks_left.json

python3 calibrate_headcam.py solve data/aligned/pnp1.jsonl \
  --clicks config/pico_cam/clicks_left.json
# 默认写出 config/pico_cam/head_to_cam_solved_left.json
# 门禁: reproj_px_mean < 8
```

兼容：也可继续写主流程文件名：

```bash
python3 calibrate_headcam.py solve data/aligned/pnp1.jsonl \
  --clicks config/pico_cam/clicks_left.json \
  -o config/pico_cam/head_to_cam_solved.json
```

### 2.2 右眼（本文新增；主流程不要求）

同一段 `pnp1`，裁右半幅再点一遍手柄硬特征（先左柄后右柄）：

```bash
python3 calibrate_headcam.py annotate \
  data/aligned/pnp1.jsonl data/raw/pnp1/vst.h264 \
  --eye right --n 12 \
  -o config/pico_cam/clicks_right.json

python3 calibrate_headcam.py solve data/aligned/pnp1.jsonl \
  --clicks config/pico_cam/clicks_right.json
# 默认写出 config/pico_cam/head_to_cam_solved_right.json
```

交互：左键每帧点 2 点（先左柄原点、后右柄）；`n` 跳过、`r` 重标、`q` 结束。

### 2.3 叠图验收（可选）

```bash
# 左
python3 overlay_skeleton.py data/aligned/vc1.jsonl data/raw/vc1/vst.h264 \
  --eye left --no-manus --out-fps 30 \
  -o data/review/B_vc1_left.mp4

# 右（自动找 head_to_cam_solved_right.json；没有则用工厂外参）
python3 overlay_skeleton.py data/aligned/vc1.jsonl data/raw/vc1/vst.h264 \
  --eye right --no-manus --out-fps 30 \
  -o data/review/B_vc1_right.mp4
```

**通过（叠图）**：左右眼 `--no-manus` 时 L/R 圆点都贴手柄。  
**不通过也不要**用右眼画面去拧 `calib_wrist.json` 的 `pos`——那会污染训练用的唯一 3D 腕位姿。

---

## 3. 和主方案的边界

| 项目 | 主方案（已完结） | 本文（可选） |
|------|------------------|--------------|
| 左眼 PnP | 要做 | 可复用 |
| 右眼 PnP | 不做 | 可加做 |
| `T_calib` | 要做（3D / 尺量 pos） | 不改规则 |
| 训练 HDF5 pose | 一套世界系 | **仍一套** |
| 是否必须 | 是 | **否** |

后续若做「双目联合 solve / 三角化验 3D」，也放本文扩展，不塞进主 CALIB 流程。
