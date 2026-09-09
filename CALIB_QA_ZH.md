# 标定与视频验收（文档 3/3）

操作见 [`../README.md`](../README.md)；数据含义与示例图见 [`PIPELINE_ZH.md`](PIPELINE_ZH.md)。

**VST 规格速查**（详见 README §3）：整幅 **2160×810** SBS（每眼 **1080×810**）；请求 60fps，**真实≈50fps**（以 `vst.ts.jsonl` 计；ffprobe 对裸流常误报 25）。转预览 mp4 用真实 `FR`：

```bash
FR=50   # python3 -c "from make_review_video import load_sidecar,sidecar_fps; print(sidecar_fps(load_sidecar('data/raw/S/vst.ts.jsonl')))"
ffmpeg -y -framerate $FR -i data/raw/S/vst.h264 -c copy data/raw/S/vst.mp4
# 左眼（叠骨架验收默认）/ 右眼（立体深度预览）:
ffmpeg -y -framerate $FR -i data/raw/S/vst.h264 \
  -vf "crop=1080:810:0:0" -c:v libx264 -pix_fmt yuv420p data/raw/S/vst_left.mp4
ffmpeg -y -framerate $FR -i data/raw/S/vst.h264 \
  -vf "crop=1080:810:1080:0" -c:v libx264 -pix_fmt yuv420p data/raw/S/vst_right.mp4
```

---

## 1. 要标什么

| 标定 | 文件 | 影响 |
|------|------|------|
| 头→相机（叠图） | `head_to_cam_solved_{left,right}.json` | **只影响叠视频**；左右眼**分开** PnP；不进腕位姿 |
| 双目内外参（训练） | `config/pico_cam/vst_cam.json` | 写入 HDF5 `attrs.video_cam`；深度用左右 R/t |
| 手柄→手套腕 `T_calib` | `config/calib_wrist.json` | **进训练包** `*_wrist_pose`（含 quat 与腕偏移 pos） |
| MANUS 手套 `.mcal` | `config/manus_left.mcal`、`config/manus_right.mcal` | 手指精度；换人必换；每条会话记录哈希与 SDK 加载结果 |

### 只靠左眼叠图 —— 其实不够（重要）

若你是**盯着左眼画面**把骨架拧到贴手（尤其拧 `T_calib.pos`），那确实容易变成：

> 对齐的是「左眼里的手」，不是「物理 3D 手」。

右眼不贴，往往就是这个症状：左眼误差被吃进了 3D / 或只标了左眼 `head→cam`。

三条链必须拆开：

| 量 | 正确求法 | 错误求法 |
|----|----------|----------|
| `T_calib`（进训练腕位姿） | 3D：基准姿 `--pose forward` 解 quat；`pos` 用尺量/3D 原点差 | 看左眼视频拧 `pos` 直到「看起来贴」 |
| `head→cam`（只影响叠图） | **左右眼分别**（或联合）PnP | 只标左眼，右眼套左眼外参 |
| `vst_cam` 左右基线 | 工厂外参 / 双目联合约束 | 用单目误差去改腕偏移 |

训练包里的 `*_wrist_pose` **不吃**左眼 PnP；但若你把投影误差拧进了 `T_calib.pos`，训练标签的 3D 就偏了，双目深度/右眼都会吃亏。

### 方案（按推荐顺序）

**方案 A — 分清职责（立刻可做，成本最低）**

1. `T_calib.pos`：**禁止**用视频拧。尺量（手柄局部：前/左/上，厘米级）或 3D `--show-origins` 看原点差。  
2. 叠图验收：`--no-manus` 先看 **L/R 圆点是否贴手柄**（验相机）；再开手指验 `T_calib`。  
3. 左、右眼**分开**叠图门禁：两边圆点都贴才算相机链过关。

```bash
# 左眼 PnP（已有）
python3 calibrate_headcam.py annotate ... --eye left  -o config/pico_cam/clicks_left.json
python3 calibrate_headcam.py solve ... --clicks config/pico_cam/clicks_left.json \
  -o config/pico_cam/head_to_cam_solved_left.json

# 右眼 PnP（同样点一手柄硬特征，裁的是右半幅）
python3 calibrate_headcam.py annotate ... --eye right -o config/pico_cam/clicks_right.json
python3 calibrate_headcam.py solve ... --clicks config/pico_cam/clicks_right.json \
  -o config/pico_cam/head_to_cam_solved_right.json

# 验收
python3 overlay_skeleton.py ... --eye left  --headcam config/pico_cam/head_to_cam_solved_left.json
python3 overlay_skeleton.py ... --eye right --headcam config/pico_cam/head_to_cam_solved_right.json
```

**方案 B — 双目联合约束（相机更稳）**

同一批 3D 手柄点，在左右眼各点像素，一次优化使  
`e = Σ_eye Σ_i ||π(K_e, T_head→cam_e, P_i) − uv_{e,i}||²` 最小；  
并可把 `T_right ≈ T_left ∘ T_baseline`（`vst_cam` 基线）作软约束，避免左右各飘一套。  
（实现：在 A 的两份 clicks 之上加 `solve-stereo`；需要时再加。）

**方案 C — 立体三角化验 3D（验「是不是真物理」）**

左右眼点同一手柄特征 → 三角化得 3D → 与 Tracking 手柄原点比欧氏误差。  
误差大：Tracking/标定问题；误差小但叠图不贴：纯投影/`head→cam` 问题。

**方案 D — 用双目深度当几何老师（接管线时）**

VST 左右 crop + `video_cam` 出深度；腕/指尖应落在深度表面上。  
这验的是 **世界系标签 vs 立体几何**，比单目叠图更接近「物理」。

**默认落地：先做 A（左右各一份 PnP + 禁止视频拧 pos）**；仍不够再上 B/C。

---

## 2. 干净重跑（推荐会话名）

全程 **同一次佩戴**，`hand_motion=none`，双手在画面**中部**。

| 会话 | 动作 |
|------|------|
| `pnp1` | 15–25s，双手画面中央缓慢动 → 点手柄硬特征解 PnP |
| `wc1` | 15–20s，**掌心朝下、手指朝前**静止 → `--pose forward` 解 T_calib |
| `vc1` | 40–60s，翻掌+握拳 → 叠视频验收 |

```bash
cd ~/pico_controller
bash scripts/ego_ctl.sh stop && bash scripts/ego_ctl.sh start
bash scripts/ego_ctl.sh check

python3 pico_record.py start pnp1   # … Enter 停
python3 align_pico_manus.py data/raw/pnp1/pico.jsonl data/raw/pnp1/manus.jsonl \
  -o data/aligned/pnp1.jsonl --full
python3 calibrate_headcam.py annotate data/aligned/pnp1.jsonl data/raw/pnp1/vst.h264 \
  --n 12 --out config/pico_cam/clicks_pnp1.json
python3 calibrate_headcam.py solve data/aligned/pnp1.jsonl \
  --clicks config/pico_cam/clicks_pnp1.json \
  -o config/pico_cam/head_to_cam_solved.json
# 门禁: reproj_px_mean < 8

python3 pico_record.py start wc1    # 掌心朝下手指朝前
python3 pico_record.py start vc1    # 翻掌验证（勿摘手套）

python3 align_pico_manus.py data/raw/wc1/pico.jsonl data/raw/wc1/manus.jsonl \
  -o data/aligned/wc1.jsonl --full
python3 align_pico_manus.py data/raw/vc1/pico.jsonl data/raw/vc1/manus.jsonl \
  -o data/aligned/vc1.jsonl --full

python3 calibrate_wrist.py data/raw/wc1/pico.jsonl data/raw/wc1/manus.jsonl \
  --pose forward -o config/calib_wrist.json
# 尺量后填 left/right.pos（米，手柄局部：前/左/上）

# 验收：先圆点贴手柄，再开手指
python3 overlay_skeleton.py data/aligned/vc1.jsonl data/raw/vc1/vst.h264 \
  --no-manus -o data/review/B_vc1.mp4 --out-fps 30
python3 overlay_skeleton.py data/aligned/vc1.jsonl data/raw/vc1/vst.h264 \
  -o data/review/overlay_vc1.mp4 --out-fps 30
# 左=VST+骨架 右=3D（同轴同 fps）
python3 make_review_video.py data/aligned/vc1.jsonl data/raw/vc1/vst.h264 \
  -o data/review/review_vc1_overlay.mp4 --out-fps 30 --overlay
```

**通过**：真手掌心朝下 → 骨架掌心朝下（左右都过）；翻掌跟随。

---

## 3. 拆开查（别跳步）

```text
A 时间 Δt  →  B 圆点贴手柄(--no-manus)  →  C 手指/掌心(T_calib)  →  D 导出训练包
```

| 现象 | 先查 |
|------|------|
| 圆点不贴手柄 | PnP（右手勿贴画面边缘） |
| 圆点贴、掌心反了 | `T_calib` / 标定姿是否 forward |
| 只有右手歪 | 右点击几何 + 右绑带 |
| 叠加拧、曾开过 imu | 保持 none；代码已 `hand_local` |

---

## 4. 清空重来时删什么

```bash
# 旧会话（按需）
rm -rf data/raw/<旧名> data/aligned/<旧名>.jsonl data/review/*<旧名>*
# 重标时
# 重置 calib_wrist 为单位阵后按 §2 重做；clicks / head_to_cam 随 pnp1 重做
```

不要用 imu「修翻掌」。不要长期 `pos=[0,0,0]` 又不接受「腕=手柄点」的语义。
