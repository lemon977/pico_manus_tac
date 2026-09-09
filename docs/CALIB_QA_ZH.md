# 标定与视频验收（文档 3/3）

操作见 [`../README.md`](../README.md)；数据含义与示例图见 [`PIPELINE_ZH.md`](PIPELINE_ZH.md)。

**VST 规格速查**（详见 README §3）：整幅 **4096×1536** SBS（每眼 **2048×1536**）；请求 30fps，**真实≈30fps**（以 `vst.ts.jsonl` 计；裸流标称 fps 不作依据）。转预览 mp4 用真实 `FR`：

```bash
FR=30   # python3 -c "from make_review_video import load_sidecar,sidecar_fps; print(sidecar_fps(load_sidecar('data/sessions/S/raw/vst.ts.jsonl')))"
ffmpeg -y -framerate $FR -i data/sessions/S/raw/vst.h264 -c copy data/sessions/S/raw/vst.mp4
# 左眼（叠骨架验收默认）/ 右眼（立体深度预览）:
ffmpeg -y -framerate $FR -i data/raw/vc1/vst.h264 \
  -vf "crop=2048:1536:0:0" -c:v libx264 -pix_fmt yuv420p data/sessions/S/raw/vst_left.mp4

# ffmpeg -y -framerate 30 -f h264 -i data/raw/vc1/vst.h264   -vf "crop=1080:810:0:0" -c:v libx264 -pix_fmt yuv420p data/raw/vc1/vst_left.mp4
# 如若忘记跑包装，那就直接指定，方便


ffmpeg -y -framerate $FR -i data/raw/vc1/vst.h264 \
  -vf "crop=2048:1536:2048:0" -c:v libx264 -pix_fmt yuv420p data/sessions/S/raw/vst_right.mp4
```

---

## 1. 要标什么

| 标定 | 文件 | 影响 |
|------|------|------|
| 头→左眼相机（叠图） | `config/pico_cam/head_to_cam_solved.json` | **只影响叠视频验收**（默认左眼）；**不进**腕位姿 |
| 双目内外参（训练） | `config/pico_cam/vst_cam.json` | 写入 HDF5 `attrs.video_cam`；深度用左右 R/t |
| 手柄→手套腕 `T_calib` | `config/calib_wrist.json` | **进训练包** `*_wrist_pose`（含 quat 与腕偏移 pos），并以数值写入 `attrs.controller_to_wrist_calibration` |
| MANUS 手套 `.mcal` | `config/*.mcal` | 手指精度；换人必换 |

### 硬规则：`T_calib.pos` 怎么定（影响训练）

结论已确认：**不能**靠「只看左眼叠图把骨架拧贴」来改 `pos`。

| | |
|--|--|
| **会怎样** | `pos` 进训练包唯一的 `*_wrist_pose`。左眼拧出来的偏移常是在补投影误差，不是物理腕偏移 → **右眼叠不上**，且 **模型学偏的 3D** |
| **正确做法** | `calibrate_wrist.py --pose forward` 解 **quat**；`pos` 用**尺量**（米，手柄局部：前/左/上）或 3D 原点差填写 |
| **叠图干什么** | 只验收：先 `--no-manus` 看圆点贴不贴手柄（相机），再开手指看相对手柄（`T_calib`）。**发现不贴 → 查 PnP / 重标，禁止改 pos 硬凑左眼** |
| **可选右眼叠图** | [`STEREO_PNP_ZH.md`](STEREO_PNP_ZH.md)（不改主流程、不增加第二套训练 pose） |

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
# 填 left/right.pos：尺量（米，手柄局部 前/左/上）。禁止对着左眼视频拧 pos。

# 验收：先圆点贴手柄，再开手指（不贴 → 查 PnP/重标，勿拧 pos）
python3 overlay_skeleton.py data/aligned/vc1.jsonl data/raw/vc1/vst.h264 \
  --no-manus -o data/review/B_vc1.mp4 --out-fps 30
python3 overlay_skeleton.py data/aligned/vc1.jsonl data/raw/vc1/vst.h264 \
  -o data/review/overlay_vc1.mp4 --out-fps 30
# 左=VST+骨架 右=3D（同轴同 fps）
python3 make_review_video.py data/aligned/vc1.jsonl data/raw/vc1/vst.h264 \
  -o data/review/review_vc1_overlay.mp4 --out-fps 30 --overlay
```

**通过**：真手掌心朝下 → 骨架掌心朝下；翻掌跟随。圆点贴手柄靠 PnP，不靠拧 `pos`。

---

## 3. 拆开查（别跳步）

```text
A 时间 Δt  →  B 圆点贴手柄(--no-manus)  →  C 手指/掌心(T_calib)  →  D 导出训练包
```

| 现象 | 先查 |
|------|------|
| 圆点不贴手柄 | PnP（右手勿贴画面边缘）；**不要改 pos** |
| 左眼贴、右眼不贴 | 多半曾用左眼拧过 `pos` → **重置 pos 为尺量值**，查相机；见 §1 硬规则 |
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
**不要**为了左眼叠图好看去改 `pos`（会污染训练 `*_wrist_pose`）。
